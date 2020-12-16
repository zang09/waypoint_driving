#ifndef driving_node_DRIVING_NODE_H_
#define driving_node_DRIVING_NODE_H_

#include <ros/ros.h>
#include <rviz_flag_plugin/PointArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>

#define PI       3.1415912
#define RAD2DEG  180/PI
#define DEG2RAD  PI/180

typedef pcl::PointXYZI PointType;

struct Line
{
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  double angle_rad;
  double angle_deg;
};

class DrivingNode
{
public:
  DrivingNode();
  ~DrivingNode();

  void initParams();
  void allocateMemory();
  void resetParams();
  void subscribeAndPublish();

  void waypointFlagHandler(const rviz_flag_plugin::PointArrayConstPtr &point_msg);
  void odomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void limitCloudDegree(float start_deg, float end_deg, pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out);
  void extractNonGround(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out);
  void clusteredCloud(pcl::PointCloud<PointType> cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out);
  void calculateLineSegment();
  void visualizationSegmentedLine();
  void calculateTargetVel(double &left_vel, double &right_vel);
  void visualizationVirtualPoint();
  void visualizationReferenceAngle();
  double projectionLengthToLine(geometry_msgs::Point s_p, geometry_msgs::Point e_p, geometry_msgs::Point c_p);
  geometry_msgs::Point projectionPointToLine(geometry_msgs::Point s_p, geometry_msgs::Point e_p, geometry_msgs::Point c_p);
  double getYawFromQuaternion(geometry_msgs::Quaternion q_msg);
  void velocityFilter(geometry_msgs::Twist &vel);

private:
  // ROS
  ros::NodeHandle nh_;

  ros::Subscriber sub_waypoint_flag_;
  ros::Subscriber sub_current_odom_;
  ros::Subscriber sub_point_cloud_;
  ros::Publisher  pub_motor_vel_;
  ros::Publisher  pub_line_strip_;
  ros::Publisher  pub_virtual_point_;
  ros::Publisher  pub_object_point_;
  ros::Publisher  pub_reference_angle_;

  ros::Publisher  pub_roi_cloud_;
  ros::Publisher  pub_ground_removal_cloud_;
  ros::Publisher  pub_clustered_cloud_;

  // Variable
  double reference_angle_;
  double current_yaw_;
  geometry_msgs::Pose  current_pose_;

  std::vector<Line> line_info_;
  visualization_msgs::Marker line_strip_;
  geometry_msgs::Point origin_point_;
  std::vector<geometry_msgs::Point> target_points_;

  pcl::PointCloud<PointType>::Ptr cloud_in_;
  pcl::PointCloud<PointType>::Ptr roi_cloud_;
  pcl::PointCloud<PointType>::Ptr ground_removal_cloud_;
  pcl::PointCloud<PointType>::Ptr clustered_cloud_;
  std::vector<pcl::PointXYZ> centroid_info_;

  std_msgs::Header cloud_header_;

  std::deque<double> left_velocity_;
  std::deque<double> right_velocity_;

  int line_num_;
  int filter_size_;
  bool velocity_filter_;

  // Control Param
  float  beta_;               // chose 0.1~0.9 begger, turn faster
  float  mean_vel_;           // straight speed m/s
  float  turn_rad_;           // turn radius [m]
  float  border_dist_;        // boundary distance
  double border_ang_;         // boundary angle (deg -> rad) turn angle difference satuated at border_ang_
  float  virtual_point_dist_;
};

#endif /* driving_node_DRIVING_NODE_H_ */
