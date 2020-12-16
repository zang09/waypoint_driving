#include "../include/driving_node.h"

using namespace geometry_msgs;

DrivingNode::DrivingNode() :
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
  nh_.param<int>("/waypoint_driving/filter_size", filter_size_, 30);
  nh_.param<float>("/waypoint_driving/beta", beta_, 0.5);
  nh_.param<float>("/waypoint_driving/mean_velocity", mean_vel_, 1.0);
  nh_.param<float>("/waypoint_driving/turn_radius", turn_rad_, 1.25); //0.5
  nh_.param<float>("/waypoint_driving/boundary_distance", border_dist_, 1.25); //turn_radius*2
  nh_.param<double>("/waypoint_driving/border_angle", border_ang_, 30);
  border_ang_ *= DEG2RAD;
  nh_.param<float>("/waypoint_driving/virtual_point_distance", virtual_point_dist_, 1.5);

  left_velocity_.resize(filter_size_);
  right_velocity_.resize(filter_size_);
}

void DrivingNode::allocateMemory()
{
  cloud_in_.reset(new pcl::PointCloud<PointType>());
  roi_cloud_.reset(new pcl::PointCloud<PointType>());
  ground_removal_cloud_.reset(new pcl::PointCloud<PointType>());
  clustered_cloud_.reset(new pcl::PointCloud<PointType>());
}

void DrivingNode::resetParams()
{
  cloud_in_->clear();
  roi_cloud_->clear();
  ground_removal_cloud_->clear();
  clustered_cloud_->clear();

  centroid_info_.clear();
}

void DrivingNode::subscribeAndPublish()
{
  sub_waypoint_flag_   = nh_.subscribe<rviz_flag_plugin::PointArray>("flag/points", 10, &DrivingNode::waypointFlagHandler, this);
  sub_current_odom_    = nh_.subscribe<nav_msgs::Odometry>("odom", 100, &DrivingNode::odomHandler, this);
  sub_point_cloud_     = nh_.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 5, &DrivingNode::cloudHandler, this, ros::TransportHints().tcpNoDelay());

  pub_motor_vel_       = nh_.advertise<geometry_msgs::Twist>("argos_mr/motor_vel", 10);
  pub_line_strip_      = nh_.advertise<visualization_msgs::Marker>("flag/line", 1);
  pub_virtual_point_   = nh_.advertise<visualization_msgs::Marker>("virtual/point", 1);
  pub_object_point_    = nh_.advertise<visualization_msgs::Marker>("object/centroid_point", 1);
  pub_reference_angle_ = nh_.advertise<nav_msgs::Odometry>("reference/angle", 1);

  pub_roi_cloud_            = nh_.advertise<sensor_msgs::PointCloud2>("roi_cloud", 1);
  pub_ground_removal_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_removal_cloud", 1);
  pub_clustered_cloud_      = nh_.advertise<sensor_msgs::PointCloud2>("clustered_cloud", 1);
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

  if(velocity_filter_) velocityFilter(vel);

  pub_motor_vel_.publish(vel);
}

void DrivingNode::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{  
  cloud_header_ = cloud_msg->header;
  pcl::fromROSMsg(*cloud_msg, *cloud_in_);

  limitCloudDegree(20.0, 160.0, *cloud_in_, roi_cloud_);

  extractNonGround(*roi_cloud_, ground_removal_cloud_);

  clusteredCloud(*ground_removal_cloud_, clustered_cloud_);

  visualization_msgs::Marker points;
  for(int i=0; i<centroid_info_.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = centroid_info_[i].x;
    p.y = centroid_info_[i].y;
    p.z = centroid_info_[i].z;

    if(abs(p.y)<1 && abs(p.z)<0.8)
    {
      if(p.x<3)
      {
        std::cout << "stop!" << std::endl;
        points.points.push_back(p);
        break;
      }
      else if(p.x<5){
        std::cout << "very slow!" << std::endl;
      }
      else if(p.x<8){
        std::cout << "slow!" << std::endl;
      }
      else {
        std::cout << "something appear!" << std::endl;
      }

      points.points.push_back(p);
    }
  }

  points.header.frame_id = "map";
  points.header.stamp = ros::Time::now();
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = points.scale.y = points.scale.z = 1.0;
  points.color.a = points.color.g = 1.0;

  pub_object_point_.publish(points);


  sensor_msgs::PointCloud2 temp_cloud;
  if (pub_roi_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*roi_cloud_, temp_cloud);
    temp_cloud.header.stamp = cloud_header_.stamp;
    temp_cloud.header.frame_id = "map";
    pub_roi_cloud_.publish(temp_cloud);
  }

  if (pub_ground_removal_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*ground_removal_cloud_, temp_cloud);
    temp_cloud.header.stamp = cloud_header_.stamp;
    temp_cloud.header.frame_id = "map";
    pub_ground_removal_cloud_.publish(temp_cloud);
  }

  if (pub_clustered_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*clustered_cloud_, temp_cloud);
    temp_cloud.header.stamp = cloud_msg->header.stamp;
    temp_cloud.header.frame_id = "map";
    pub_clustered_cloud_.publish(temp_cloud);
  }

  resetParams();
}

void DrivingNode::limitCloudDegree(float start_deg, float end_deg, pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  float x,y,theta;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointType> extract;

  for (size_t i = 0; i<cloud_in.points.size(); i++)
  {
    //change axis
    x = cloud_in.points[i].y;
    y = cloud_in.points[i].x;

    theta = atan2(y, x);
    if(theta < 0) theta += 2*PI;

    if (((theta*RAD2DEG) < start_deg) || ((theta*RAD2DEG) > end_deg))
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud_in.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_out);

  //    pcl::PassThrough<PointType> pass_x;
  //    pass_x.setInputCloud(cloud_out);
  //    pass_x.setFilterFieldName("x");
  //    pass_x.setFilterLimits(0, 0+20.0);
  //    pass_x.filter(*cloud_out);
}

void DrivingNode::extractNonGround(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients (true);          //(옵션) // Enable model coefficient refinement (optional).
  seg.setInputCloud (cloud_in.makeShared());   //입력
  seg.setModelType (pcl::SACMODEL_PLANE);      //적용 모델   // Configure the object to look for a plane.
  seg.setMethodType (pcl::SAC_RANSAC);         //적용 방법   // Use RANSAC method.
  seg.setDistanceThreshold (0.4);              //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
  seg.segment (*inliers, *coefficients);       //세그멘테이션 적용

  //    pcl::copyPointCloud(cloud_in, *inliers, *cloud_out);

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud (cloud_in.makeShared());
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_out);
}

void DrivingNode::clusteredCloud(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out)
{
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_in.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance (0.35); // 0.35m
  ec.setMinClusterSize (30);
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
      pt.intensity = (float)i;

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
  for (size_t i=0; i<filter_size_; i++)
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
