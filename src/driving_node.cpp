#include "../include/driving_node.h"

using namespace geometry_msgs;

DrivingNode::DrivingNode() :
  coeff_velocity_(1.0),
  line_num_(0)
{
  initParams();
  allocateMemory();
  resetParams();
  subscribeAndPublish();
}

DrivingNode::~DrivingNode()
{}

void DrivingNode::initParams()
{
  nh_.param<bool>("/waypoint_driving/velocity_filter", velocity_filter_, true);
  nh_.param<int>("/waypoint_driving/filter_size", filter_size_, 20);
  nh_.param<int>("/waypoint_driving/max_filter_size", max_filter_size_, 50);
  nh_.param<float>("/waypoint_driving/robot_width", robot_width_, 1.0);

  nh_.param<float>("/waypoint_driving/beta", beta_, 0.5);
  nh_.param<float>("/waypoint_driving/mean_velocity", mean_vel_, 1.0);
  nh_.param<float>("/waypoint_driving/turn_radius", turn_rad_, 1.25); //0.5
  nh_.param<float>("/waypoint_driving/boundary_distance", border_dist_, 1.25); //turn_radius*2
  nh_.param<double>("/waypoint_driving/border_angle", border_ang_, 30);
  border_ang_ *= DEG2RAD;
  nh_.param<float>("/waypoint_driving/virtual_point_distance", virtual_point_dist_, 1.5);

  left_velocity_.resize(max_filter_size_);
  right_velocity_.resize(max_filter_size_);
}

void DrivingNode::allocateMemory()
{
  cloud_in_.reset(new pcl::PointCloud<PointType>());
  roi_cloud_.reset(new pcl::PointCloud<PointType>());
  smoothing_cloud_.reset(new pcl::PointCloud<PointType>());
  clustered_cloud_.reset(new pcl::PointCloud<PointType>());
}

void DrivingNode::resetParams()
{
  cloud_in_->clear();
  roi_cloud_->clear();
  smoothing_cloud_->clear();
  clustered_cloud_->clear();

  centroid_info_.clear();
  obstacle_points_.points.clear();
}

void DrivingNode::subscribeAndPublish()
{
  sub_waypoint_flag_   = nh_.subscribe<rviz_flag_plugin::PointArray>("rviz/flag_points", 10, &DrivingNode::waypointFlagHandler, this);
  sub_current_odom_    = nh_.subscribe<nav_msgs::Odometry>("hdl_localization/odom", 100, &DrivingNode::odomHandler, this);
  sub_point_cloud_     = nh_.subscribe<sensor_msgs::PointCloud2>("waypoint_driving/segmented_cloud_pure", 1, &DrivingNode::cloudHandler, this, ros::TransportHints().tcpNoDelay());

  pub_motor_vel_       = nh_.advertise<geometry_msgs::Twist>("waypoint_driving/motor_vel", 10);
  pub_line_strip_      = nh_.advertise<visualization_msgs::Marker>("waypoint_driving/flag_line", 1);
  pub_virtual_point_   = nh_.advertise<visualization_msgs::Marker>("waypoint_driving/robot/virtual_point", 1);
  pub_object_point_    = nh_.advertise<visualization_msgs::Marker>("waypoint_driving/object/centroid_point", 1);
  pub_reference_angle_ = nh_.advertise<nav_msgs::Odometry>("waypoint_driving/reference_angle", 1);

  pub_roi_cloud_       = nh_.advertise<sensor_msgs::PointCloud2>("waypoint_driving/roi_cloud", 1);
  pub_clustered_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("waypoint_driving/clustered_cloud", 1);
}

void DrivingNode::waypointFlagHandler(const rviz_flag_plugin::PointArrayConstPtr &point_msg)
{
  target_points_.clear();
  target_points_.assign(point_msg->points.begin(), point_msg->points.end());

  calculateLineSegment();

  ROS_INFO("Line size: %d", (int)line_info_.size());

  visualizationSegmentedLine();
}

void DrivingNode::odomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  current_pose_.position    = odom_msg->pose.pose.position;
  current_pose_.orientation = odom_msg->pose.pose.orientation;
  current_yaw_ = getYawFromQuaternion(current_pose_.orientation);

  geometry_msgs::Twist vel;

  if(line_num_ < (int)line_info_.size())
  {
    double left_vel, right_vel;
    calculateTargetVel(left_vel, right_vel);

    vel.linear.x = left_vel;
    vel.linear.y = right_vel;

    if(line_num_ >= (int)line_info_.size()) return;

    visualizationVirtualPoint();
    visualizationReferenceAngle();
  }
  else
  {
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
  }

  vel.linear.x *= coeff_velocity_;
  vel.linear.y *= coeff_velocity_;

  if(velocity_filter_) velocityFilter(vel);

  std::cout << "velocity l: " << vel.linear.x << std::endl;
  std::cout << "velocity r: " << vel.linear.y << std::endl << std::endl;
  pub_motor_vel_.publish(vel);
}

void DrivingNode::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{  
  cloud_header_ = cloud_msg->header;
  pcl::fromROSMsg(*cloud_msg, *cloud_in_);

  limitCloudDegree(45.0, 135.0, *cloud_in_, roi_cloud_);

  //smoothingCloud(*roi_cloud_, smoothing_cloud_);

  euclideanClusteredCloud(*roi_cloud_, clustered_cloud_);

  perceptionObstacle();

  visualizationObstacle();

  if (pub_clustered_cloud_.getNumSubscribers() != 0)
  {
    sensor_msgs::PointCloud2 temp_cloud;
    pcl::toROSMsg(*clustered_cloud_, temp_cloud);
    temp_cloud.header.stamp = cloud_header_.stamp;
    temp_cloud.header.frame_id = "base_link";

    pub_clustered_cloud_.publish(temp_cloud);
  }

  resetParams();
}

void DrivingNode::limitCloudDegree(float start_deg, float end_deg, pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  float x,y,theta;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointType> extract;

  for (size_t i=0; i<cloud_in.points.size(); i++)
  {
    //cloud_in.points[i].y -= 1.5;

    //change axis
    x = cloud_in.points[i].y;
    y = cloud_in.points[i].x;

    //change origin
    if(y>0)      y += robot_width_/2.f + 1.f;
    else if(y<0) y -= robot_width_/2.f + 1.f;

    theta = atan2(y, x);
    if(theta < 0) theta += 2*PI;

    if (((theta*RAD2DEG) > start_deg) && ((theta*RAD2DEG) < end_deg))
    {
      inliers->indices.push_back(i);
    }
  }

  extract.setInputCloud(cloud_in.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_out);

  //  pcl::PassThrough<PointType> pass_y;
  //  pass_y.setInputCloud(cloud_out);
  //  pass_y.setFilterFieldName("y");
  //  pass_y.setFilterLimits(0-3.0, 0+3.0);
  //  pass_y.filter(*cloud_out);
}

void DrivingNode::smoothingCloud(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  if(cloud_in.empty())
    return;

  pcl::copyPointCloud(cloud_in, *cloud_out);

  pcl::KdTreeFLANN<PointType> tree;
  size_t K = 5;

  tree.setInputCloud(cloud_in.makeShared());

  for(size_t i=0; i<cloud_in.points.size(); i++)
  {
    std::vector<std::pair<float, float>> leftPoint, rightPoint;
    int leastCnt=0, rightCnt=0, leftCnt=0;
    float tempX = cloud_in.points[i].x;
    float tempY = cloud_in.points[i].y;

    std::vector<int>   pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    //tree.radiusSearch(cloud_in->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
    tree.nearestKSearch(cloud_in.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);

    float refAngle = 0;
    for(size_t j=0; j<pointIdxNKNSearch.size(); j++)
    {
      float angle = atan2((cloud_in)[pointIdxNKNSearch[j]].y, (cloud_in)[pointIdxNKNSearch[j]].x);

      if(angle < 0) angle += 2*M_PI;

      if(j == 0) { refAngle = angle; }
      else
      {
        std::pair<float, float> p = std::make_pair((cloud_in)[pointIdxNKNSearch[j]].x, (cloud_in)[pointIdxNKNSearch[j]].y);

        float opRefAngle = refAngle + M_PI;

        if(refAngle < M_PI) {
          if(refAngle < angle && angle < opRefAngle) {
            leftPoint.push_back(p);
            leftCnt++;
          }
          else {
            rightPoint.push_back(p);
            rightCnt++;
          }
        }
        else {
          opRefAngle = fmodf(opRefAngle, 2*M_PI);

          if(opRefAngle < angle && angle < refAngle) {
            rightPoint.push_back(p);
            rightCnt++;
          }
          else {
            leftPoint.push_back(p);
            leftCnt++;
          }
        }
      }
    }

    if(rightCnt < leftCnt) leastCnt = rightCnt;
    else leastCnt = leftCnt;

    //cout << "cnt: " << cnt << " left: " << left_cnt << " right: " << right_cnt << endl;

    for(int j=0; j<leastCnt; j++) {
      tempX += (leftPoint[j].first + rightPoint[j].first);
      tempY += (leftPoint[j].second + rightPoint[j].second);
    }

    leastCnt = leastCnt*2 + 1;

    cloud_out->points[i].x = tempX / (float)leastCnt;
    cloud_out->points[i].y = tempY / (float)leastCnt;
  }
}

void DrivingNode::euclideanClusteredCloud(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  if(cloud_in.empty())
    return;

  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_in.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (0.4); // 0.35m
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in.makeShared());
  ec.extract (cluster_indices);

  //std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

  pcl::PointCloud<PointType> total_cloud;
  int i = 1;
  for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it!=cluster_indices.end(); ++it)
  {
    pcl::CentroidPoint<pcl::PointXYZ> centroid;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      PointType pt = cloud_in.points[*pit];
      pt.intensity = (float)i*5;

      centroid.add (pcl::PointXYZ (pt.x, pt.y, pt.z));
      total_cloud.push_back(pt);
    }

    pcl::PointXYZ cp;
    centroid.get(cp);
    centroid_info_.push_back(cp);
    i++;
  }

  pcl::copyPointCloud(total_cloud, *cloud_out);
}

void DrivingNode::perceptionObstacle()
{
  //scoring obstacle
  //0 -> stop
  //1 -> very slow
  //2 -> slow
  //3 -> nothing
  std::vector<std::pair<int, int>> index;

  for(int i=0; i<centroid_info_.size(); i++)
  {
    double x = centroid_info_[i].x;
    double y = centroid_info_[i].y;
    double dist = sqrt(x*x + y*y) - robot_width_/2.f;

    if(abs(y) < (robot_width_/2.f + 1.f)) /*straight case*/
    {
      if(x<3)      index.push_back(std::make_pair(0, i));
      else if(x<5) index.push_back(std::make_pair(1, i));
      else if(x<7) index.push_back(std::make_pair(2, i));
      else         index.push_back(std::make_pair(3, i));
    }
    else /*round case*/
    {
      if(dist<1)      index.push_back(std::make_pair(0, i));
      else if(dist<2) index.push_back(std::make_pair(1, i));
      else if(dist<3) index.push_back(std::make_pair(2, i));
      else            index.push_back(std::make_pair(3, i));
    }
  }

  std::sort(index.begin(), index.end());

  //change velocity
  int score, idx;

  if(index.empty()) {
    score = 3;
  }
  else {
    score = index[0].first;
    idx = index[0].second;

    geometry_msgs::Point p;
    p.x = centroid_info_[idx].x;
    p.y = centroid_info_[idx].y;
    p.z = centroid_info_[idx].z;
    obstacle_points_.points.push_back(p);
  }

  switch(score) {
  case 0:
    coeff_velocity_ = 0;
    filter_size_ = 5;
    wait_flag_ = true;
    wait_cnt_  = 0;
    break;

  case 1:
    coeff_velocity_ = 0.4;
    filter_size_ = 10;
    break;

  case 2:
    coeff_velocity_ = 0.8;
    filter_size_ = 10;
    break;

  default:
    coeff_velocity_ = 1.0;
    filter_size_ = 20;
    break;
  }

  if(wait_flag_) {
    wait_cnt_++;
    coeff_velocity_ = 0;
    filter_size_ = 5;
  }

  if(wait_cnt_ > 20) //2s
  {
    fill(left_velocity_.begin(), left_velocity_.end(), 0);
    fill(right_velocity_.begin(), right_velocity_.end(), 0);
    wait_flag_ = false;
    wait_cnt_  = 0;
  }
}

void DrivingNode::visualizationObstacle()
{
  obstacle_points_.header.stamp = cloud_header_.stamp;
  obstacle_points_.header.frame_id = "base_link";
  obstacle_points_.type = visualization_msgs::Marker::SPHERE_LIST;
  obstacle_points_.action = visualization_msgs::Marker::ADD;
  obstacle_points_.pose.orientation.w = 1.0;
  obstacle_points_.scale.x = obstacle_points_.scale.y = obstacle_points_.scale.z = 1.2;
  obstacle_points_.color.a = 1.0;
  obstacle_points_.color.r = 0.752;
  obstacle_points_.color.g = 1.0;
  obstacle_points_.color.b = 0.933;

  pub_object_point_.publish(obstacle_points_);
}

void DrivingNode::calculateLineSegment()
{
  line_strip_.points.clear();
  line_info_.clear();

  for(size_t i=0; i<target_points_.size(); i++)
  {
    line_strip_.points.push_back(target_points_.at(i));

    if(i == 0)
    {
      if((origin_point_.x != target_points_.at(i).x) && (origin_point_.y != target_points_.at(i).y))
      {
        line_num_ = 0; // init target line
      }
      origin_point_ = target_points_.at(i);
    }
    else
    {
      Line line;

      line.start_point = target_points_.at(i-1);
      line.end_point = target_points_.at(i);
      line.angle_rad = atan2(line.end_point.y - line.start_point.y,
                             line.end_point.x - line.start_point.x);
      line.angle_deg = line.angle_rad*RAD2DEG;

      line_info_.push_back(line);
    }
  }
}

void DrivingNode::visualizationSegmentedLine()
{
  line_strip_.header.frame_id = "map";
  line_strip_.header.stamp = ros::Time::now();

  line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip_.scale.x = 0.5;
  line_strip_.color.a = 1.0;
  line_strip_.color.r = (float)0.09412;
  line_strip_.color.g = (float)0.74118;
  line_strip_.color.b = (float)0.89412;
  line_strip_.pose.orientation.w = 1.0;

  pub_line_strip_.publish(line_strip_);
}

void DrivingNode::calculateTargetVel(double &left_vel, double &right_vel)
{
  //Projection distance from line segment
  double projection_value = projectionLengthToLine(line_info_.at(line_num_).start_point,
                                                   line_info_.at(line_num_).end_point,
                                                   current_pose_.position);

  double virtual_angle = atan2(projection_value, virtual_point_dist_);
  reference_angle_ = line_info_.at(line_num_).angle_rad - virtual_angle;
  double delta_angle = current_yaw_ - reference_angle_;

  if(delta_angle < -PI) delta_angle += 2*PI;
  else if(delta_angle > PI) delta_angle -= 2*PI;

  double delta_x = current_pose_.position.x-line_info_.at(line_num_).end_point.x;
  double delta_y = current_pose_.position.y-line_info_.at(line_num_).end_point.y;
  double distance = sqrt(pow(delta_x,2)+pow(delta_y,2));

  if(distance < turn_rad_)
  {
    ROS_INFO("line: %d, distance: %.2f, turn radius: %.2f", (int)line_info_.size(), distance, turn_rad_);
    line_num_++;
  }

  double alpha = 0.05 + beta_*abs(tanh(delta_angle/border_ang_));
  right_vel = (1-alpha)*mean_vel_ + alpha*mean_vel_*(-tanh(delta_angle/border_ang_));
  left_vel  = (1-alpha)*mean_vel_ + alpha*mean_vel_*(+tanh(delta_angle/border_ang_));
}

void DrivingNode::visualizationVirtualPoint()
{
  Point project_p, virtual_p;
  project_p = projectionPointToLine(line_info_.at(line_num_).start_point,
                                    line_info_.at(line_num_).end_point,
                                    current_pose_.position);

  virtual_p.x = project_p.x + virtual_point_dist_*cos(line_info_.at(line_num_).angle_rad);
  virtual_p.y = project_p.y + virtual_point_dist_*sin(line_info_.at(line_num_).angle_rad);

  visualization_msgs::Marker points;
  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.scale.x = points.scale.y = points.scale.z = 1.0;
  points.color.a = points.color.r = 1.0;
  points.points.push_back(virtual_p);

  pub_virtual_point_.publish(points);
}

void DrivingNode::visualizationReferenceAngle()
{
  nav_msgs::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp = ros::Time::now();

  tf::Quaternion quat;
  geometry_msgs::Quaternion quat_msg;
  quat.setRPY(0, 0, reference_angle_);
  tf::quaternionTFToMsg(quat, quat_msg);

  odom.pose.pose.position = current_pose_.position;
  odom.pose.pose.orientation = quat_msg;

  pub_reference_angle_.publish(odom);
}

double DrivingNode::projectionLengthToLine(Point s_p, Point e_p, Point c_p)
{
  double angle  = atan2(e_p.y-s_p.y, e_p.x-s_p.x);
  double value = -(c_p.x-s_p.x)*sin(angle) + (c_p.y-s_p.y)*cos(angle);

  /*
    double delta_x = e_p.x - s_p.x;
    double delta_y = e_p.y - s_p.y;
    double delat_l = sqrt(pow(delta_x,2)+pow(delta_y,2));
    double sin_theta = delta_y / delat_l;
    double cos_theta = delta_x / delat_l;
    double value = -(c_p.x-s_p.x)*sin_theta + (c_p.y-s_p.y)*cos_theta;
    */

  return value;
}

Point DrivingNode::projectionPointToLine(Point s_p, Point e_p, Point c_p)
{
  double angle  = atan2(e_p.y-s_p.y, e_p.x-s_p.x);
  double value = -(c_p.x-s_p.x)*sin(angle) + (c_p.y-s_p.y)*cos(angle);

  Point p;
  p.x = c_p.x + value * sin(angle);
  p.y = c_p.y + value * (-cos(angle));

  return p;
}

double DrivingNode::getYawFromQuaternion(Quaternion q_msg)
{
  double roll, pitch, yaw;
  tf::Quaternion quat;

  tf::quaternionMsgToTF(q_msg, quat);
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

void DrivingNode::velocityFilter(Twist &vel)
{
  left_velocity_.pop_front();
  right_velocity_.pop_front();

  left_velocity_.push_back(vel.linear.x);
  right_velocity_.push_back(vel.linear.y);

  double left_sum=0, right_sum=0;
  for (size_t i=max_filter_size_-1; i>=(max_filter_size_-filter_size_); i--)
  {
    left_sum += left_velocity_[i];
    right_sum += right_velocity_[i];
  }

  double left_value = left_sum / filter_size_;
  double right_value = right_sum / filter_size_;

  vel.linear.x = left_value;
  vel.linear.y = right_value;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driving_node");

  ROS_INFO("\033[1;32m---->\033[0m Driving node is Started.");

  DrivingNode dn;

  ros::spin();
  return 0;
}
