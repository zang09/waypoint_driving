#include "../include/cloud_segment.h"

CloudSegment::CloudSegment() :
  use_cloud_ring_(true),
  scan_num_(16),
  horizon_scan_(1800),
  ground_scan_idx_(7),
  ang_res_x_(0.2),
  ang_res_y_(2.0),
  ang_bottom_(15.0+0.1),
  sensor_minimum_range_(0.4),
  sensor_mount_angle_(0.0),
  segment_theta_(60.0*DEG2RAD),
  segment_alpha_x_(ang_res_x_*DEG2RAD),
  segment_alpha_y_(ang_res_y_*DEG2RAD),
  segment_valid_point_num_(5),
  segment_valid_line_num_(3)
{
  initParams();
  allocateMemory();
  resetParameters();
  subscribeAndPublish();
}

CloudSegment::~CloudSegment()
{}

void CloudSegment::initParams()
{
  nh_.param<std::string>("/waypoint_driving/pointcloud_topic", pointcloud_topic_, " ");

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = -1;
}

void CloudSegment::allocateMemory()
{
  laser_cloud_in_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_in_ring_.reset(new pcl::PointCloud<PointXYZIR>());

  full_cloud_.reset(new pcl::PointCloud<PointType>());
  full_info_cloud_.reset(new pcl::PointCloud<PointType>());

  ground_cloud_.reset(new pcl::PointCloud<PointType>());
  segmented_cloud_.reset(new pcl::PointCloud<PointType>());
  segmented_cloud_pure_.reset(new pcl::PointCloud<PointType>());
  outlier_cloud_.reset(new pcl::PointCloud<PointType>());

  full_cloud_->points.resize(scan_num_*horizon_scan_);
  full_info_cloud_->points.resize(scan_num_*horizon_scan_);

  std::pair<int8_t, int8_t> neighbor;
  neighbor.first = -1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);
  neighbor.first =  0; neighbor.second =  1; neighbor_iterator_.push_back(neighbor);
  neighbor.first =  0; neighbor.second = -1; neighbor_iterator_.push_back(neighbor);
  neighbor.first =  1; neighbor.second =  0; neighbor_iterator_.push_back(neighbor);

  all_pushed_idx_x_ = new uint16_t[scan_num_*horizon_scan_];
  all_pushed_idx_y_ = new uint16_t[scan_num_*horizon_scan_];

  queue_idx_x_ = new uint16_t[scan_num_*horizon_scan_];
  queue_idx_y_ = new uint16_t[scan_num_*horizon_scan_];
}

void CloudSegment::resetParameters()
{
  laser_cloud_in_->clear();
  ground_cloud_->clear();
  segmented_cloud_->clear();
  segmented_cloud_pure_->clear();
  outlier_cloud_->clear();

  range_mat_  = cv::Mat(scan_num_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));
  ground_mat_ = cv::Mat(scan_num_, horizon_scan_, CV_8S, cv::Scalar::all(0));
  label_mat_  = cv::Mat(scan_num_, horizon_scan_, CV_32S, cv::Scalar::all(0));
  label_cnt_  = 1;

  std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), nan_point_);
  std::fill(full_info_cloud_->points.begin(), full_info_cloud_->points.end(), nan_point_);
}

void CloudSegment::subscribeAndPublish()
{
  sub_laser_cloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, 1, &CloudSegment::cloudHandler, this);

  pub_full_cloud_      = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/full_cloud_projected", 1);
  pub_full_info_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/full_cloud_info", 1);

  pub_ground_cloud_         = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/ground_cloud", 1);
  pub_segmented_cloud_      = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/segmented_cloud", 1);
  pub_segmented_cloud_pure_ = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/segmented_cloud_pure", 1);
  pub_outlier_cloud_        = nh_.advertise<sensor_msgs::PointCloud2> ("waypoint_driving/outlier_cloud", 1);
}

void CloudSegment::copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  cloud_header_ = laserCloudMsg->header;
  // cloud_header_.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line

  pcl::fromROSMsg(*laserCloudMsg, *laser_cloud_in_);

  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laser_cloud_in_, *laser_cloud_in_, indices);

  // have "ring" channel in the cloud
  if (use_cloud_ring_ == true)
  {
    pcl::fromROSMsg(*laserCloudMsg, *laser_cloud_in_ring_);
    if (laser_cloud_in_ring_->is_dense == false)
    {
      ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
      ros::shutdown();
    }
  }
}

void CloudSegment::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  // 1. Convert ros message to pcl point cloud
  copyPointCloud(laserCloudMsg);

  // 2. Range image projection
  projectPointCloud();

  // 3. Mark ground points
  groundRemoval();

  // 4. Point cloud segmentation
  cloudSegmentation();

  // 5. Publish all clouds
  publishCloud();

  // 6. Reset parameters for next iteration
  resetParameters();
}

void CloudSegment::projectPointCloud()
{
  // range image projection
  float verticalAngle, horizonAngle, range;
  size_t rowIdn, columnIdn, index, cloudSize;
  PointType thisPoint;

  cloudSize = laser_cloud_in_->points.size();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    thisPoint.x = laser_cloud_in_->points[i].x;
    thisPoint.y = laser_cloud_in_->points[i].y;
    thisPoint.z = laser_cloud_in_->points[i].z;

    // find the row and column index in the iamge for this point
    if (use_cloud_ring_ == true)
    {
      rowIdn = laser_cloud_in_ring_->points[i].ring;
    }
    else
    {
      verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * RAD2DEG;
      rowIdn = (verticalAngle + ang_bottom_) / ang_res_y_;
    }
    if (rowIdn < 0 || rowIdn >= scan_num_)
      continue;

    horizonAngle = atan2(thisPoint.x, thisPoint.y) * RAD2DEG;

    columnIdn = -round((horizonAngle-90.0)/ang_res_x_) + horizon_scan_/2;
    if (columnIdn >= horizon_scan_)
      columnIdn -= horizon_scan_;

    if (columnIdn < 0 || columnIdn >= horizon_scan_)
      continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
    if (range < sensor_minimum_range_)
      continue;

    range_mat_.at<float>(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    index = columnIdn  + rowIdn * horizon_scan_;
    full_cloud_->points[index] = thisPoint;
    full_info_cloud_->points[index] = thisPoint;
    full_info_cloud_->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
  }
}

void CloudSegment::groundRemoval()
{
  size_t lowerInd, upperInd;
  float diffX, diffY, diffZ, angle;

  // ground_mat_
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < horizon_scan_; ++j){
    for (size_t i = 0; i < ground_scan_idx_; ++i){

      lowerInd = j + ( i )*horizon_scan_;
      upperInd = j + (i+1)*horizon_scan_;

      if (full_cloud_->points[lowerInd].intensity == -1 ||
          full_cloud_->points[upperInd].intensity == -1){
        // no info to check, invalid points
        ground_mat_.at<int8_t>(i,j) = -1;
        continue;
      }

      diffX = full_cloud_->points[upperInd].x - full_cloud_->points[lowerInd].x;
      diffY = full_cloud_->points[upperInd].y - full_cloud_->points[lowerInd].y;
      diffZ = full_cloud_->points[upperInd].z - full_cloud_->points[lowerInd].z;

      angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

      if (abs(angle - sensor_mount_angle_) <= 10){
        ground_mat_.at<int8_t>(i,j) = 1;
        ground_mat_.at<int8_t>(i+1,j) = 1;
      }
    }
  }

  // extract ground cloud (ground_mat_ == 1)
  // mark entry that doesn't need to label (ground and invalid point) for segmentation
  // note that ground remove is from 0~scan_num_-1, need range_mat_ for mark label matrix for the 16th scan
  for (size_t i = 0; i < scan_num_; ++i){
    for (size_t j = 0; j < horizon_scan_; ++j){
      if (ground_mat_.at<int8_t>(i,j) == 1 || range_mat_.at<float>(i,j) == FLT_MAX){
        label_mat_.at<int>(i,j) = -1;
      }
    }
  }

  if (pub_ground_cloud_.getNumSubscribers() != 0){
    for (size_t i = 0; i <= ground_scan_idx_; ++i){
      for (size_t j = 0; j < horizon_scan_; ++j){
        if (ground_mat_.at<int8_t>(i,j) == 1)
          ground_cloud_->push_back(full_cloud_->points[j + i*horizon_scan_]);
      }
    }
  }
}

void CloudSegment::cloudSegmentation()
{
  // segmentation process
  for (size_t i = 0; i < scan_num_; ++i)
    for (size_t j = 0; j < horizon_scan_; ++j)
      if (label_mat_.at<int>(i,j) == 0)
        labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < scan_num_; ++i)
  {
    for (size_t j = 0; j < horizon_scan_; ++j)
    {
      if (label_mat_.at<int>(i,j) > 0 || ground_mat_.at<int8_t>(i,j) == 1)
      {
        // outliers that will not be used for optimization (always continue)
        if (label_mat_.at<int>(i,j) == 999999)
        {
          if (i > ground_scan_idx_ && j % 5 == 0)
          {
            outlier_cloud_->push_back(full_cloud_->points[j + i*horizon_scan_]);
            continue;
          }
          else
            continue;
        }

        // majority of ground points are skipped
        if (ground_mat_.at<int8_t>(i,j) == 1)
        {
          if (j%5!=0 && j>5 && j<horizon_scan_-5)
            continue;
        }

        // save seg cloud
        segmented_cloud_->push_back(full_cloud_->points[j + i*horizon_scan_]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }
  }

  // extract segmented cloud for visualization
  if (pub_segmented_cloud_pure_.getNumSubscribers() != 0){
    for (size_t i = 0; i < scan_num_; ++i){
      for (size_t j = 0; j < horizon_scan_; ++j){
        if (label_mat_.at<int>(i,j) > 0 && label_mat_.at<int>(i,j) != 999999){
          segmented_cloud_pure_->push_back(full_cloud_->points[j + i*horizon_scan_]);
          //segmented_cloud_pure_->points.back().intensity = label_mat_.at<int>(i,j);
          segmented_cloud_pure_->points.back().intensity = j;
        }
      }
    }
  }
}

void CloudSegment::labelComponents(int row, int col)
{
  // use std::queue std::vector std::deque will slow the program down greatly
  float d1, d2, alpha, angle;
  int fromIndX, fromIndY, thisIndX, thisIndY;
  bool lineCountFlag[16] = {false}; //scan_num_

  queue_idx_x_[0] = row;
  queue_idx_y_[0] = col;
  int queueSize = 1;
  int queueStartInd = 0;
  int queueEndInd = 1;

  all_pushed_idx_x_[0] = row;
  all_pushed_idx_y_[0] = col;
  int allPushedIndSize = 1;

  while(queueSize > 0)
  {
    // Pop point
    fromIndX = queue_idx_x_[queueStartInd];
    fromIndY = queue_idx_y_[queueStartInd];
    --queueSize;
    ++queueStartInd;
    // Mark popped point
    label_mat_.at<int>(fromIndX, fromIndY) = label_cnt_;
    // Loop through all the neighboring grids of popped grid
    for (auto iter = neighbor_iterator_.begin(); iter != neighbor_iterator_.end(); ++iter){
      // new index
      thisIndX = fromIndX + (*iter).first;
      thisIndY = fromIndY + (*iter).second;
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= scan_num_)
        continue;
      // at range image margin (left or right side)
      if (thisIndY < 0)
        thisIndY = horizon_scan_ - 1;
      if (thisIndY >= horizon_scan_)
        thisIndY = 0;
      // prevent infinite loop (caused by put already examined point back)
      if (label_mat_.at<int>(thisIndX, thisIndY) != 0)
        continue;

      d1 = std::max(range_mat_.at<float>(fromIndX, fromIndY),
                    range_mat_.at<float>(thisIndX, thisIndY));
      d2 = std::min(range_mat_.at<float>(fromIndX, fromIndY),
                    range_mat_.at<float>(thisIndX, thisIndY));

      if ((*iter).first == 0)
        alpha = segment_alpha_x_;
      else
        alpha = segment_alpha_y_;

      angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

      if (angle > segment_theta_){

        queue_idx_x_[queueEndInd] = thisIndX;
        queue_idx_y_[queueEndInd] = thisIndY;
        ++queueSize;
        ++queueEndInd;

        label_mat_.at<int>(thisIndX, thisIndY) = label_cnt_;
        lineCountFlag[thisIndX] = true;

        all_pushed_idx_x_[allPushedIndSize] = thisIndX;
        all_pushed_idx_y_[allPushedIndSize] = thisIndY;
        ++allPushedIndSize;
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (allPushedIndSize >= 30)
    feasibleSegment = true;
  else if (allPushedIndSize >= segment_valid_point_num_)
  {
    int lineCount = 0;
    for (size_t i = 0; i < scan_num_; ++i)
      if (lineCountFlag[i] == true)
        ++lineCount;
    if (lineCount >= segment_valid_line_num_)
      feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true)
  {
    ++label_cnt_;
  }
  else
  {
    // segment is invalid, mark these points
    for (size_t i = 0; i < allPushedIndSize; ++i)
    {
      label_mat_.at<int>(all_pushed_idx_x_[i], all_pushed_idx_y_[i]) = 999999;
    }
  }
}

void CloudSegment::publishCloud()
{
  // 2. Publish clouds
  sensor_msgs::PointCloud2 laserCloudTemp;
  pcl::toROSMsg(*outlier_cloud_, laserCloudTemp);
  laserCloudTemp.header.stamp = cloud_header_.stamp;
  laserCloudTemp.header.frame_id = "base_link";
  pub_outlier_cloud_.publish(laserCloudTemp);

  // segmented cloud with ground
  pcl::toROSMsg(*segmented_cloud_, laserCloudTemp);
  laserCloudTemp.header.stamp = cloud_header_.stamp;
  laserCloudTemp.header.frame_id = "base_link";
  pub_segmented_cloud_.publish(laserCloudTemp);

  // projected full cloud
  if (pub_full_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*full_cloud_, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_header_.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pub_full_cloud_.publish(laserCloudTemp);
  }

  // original dense ground cloud
  if (pub_ground_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*ground_cloud_, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_header_.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pub_ground_cloud_.publish(laserCloudTemp);
  }

  // segmented cloud without ground
  if (pub_segmented_cloud_pure_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*segmented_cloud_pure_, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_header_.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pub_segmented_cloud_pure_.publish(laserCloudTemp);
  }

  // projected full cloud info
  if (pub_full_info_cloud_.getNumSubscribers() != 0)
  {
    pcl::toROSMsg(*full_info_cloud_, laserCloudTemp);
    laserCloudTemp.header.stamp = cloud_header_.stamp;
    laserCloudTemp.header.frame_id = "base_link";
    pub_full_info_cloud_.publish(laserCloudTemp);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_segment");

  CloudSegment cs;

  ROS_INFO("\033[1;32m---->\033[0m Cloud Segment node is Started.");

  ros::spin();
  return 0;
}
