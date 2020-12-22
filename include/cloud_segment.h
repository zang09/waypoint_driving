#ifndef cloud_segment_CLOUD_SEGMENT_H_
#define cloud_segment_CLOUD_SEGMENT_H_

#include <ros/ros.h>
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

#include <opencv/cv.h>

#define PI       3.1415912
#define RAD2DEG  180/PI
#define DEG2RAD  PI/180

struct smoothness_t{
    float value;
    size_t ind;
};

struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef pcl::PointXYZI PointType;
typedef PointXYZIRPYT  PointTypePose;

class CloudSegment
{
public:
  CloudSegment();
  ~CloudSegment();

  void initParams();
  void allocateMemory();
  void resetParameters();
  void subscribeAndPublish();

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laser_cloud_msg);
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laser_cloud_msg);
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishCloud();

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_laser_cloud_;
  ros::Publisher pub_full_cloud_;
  ros::Publisher pub_full_info_cloud_;
  ros::Publisher pub_ground_cloud_;
  ros::Publisher pub_segmented_cloud_;
  ros::Publisher pub_segmented_cloud_pure_;
  ros::Publisher pub_outlier_cloud_;

  pcl::PointCloud<PointType>::Ptr  laser_cloud_in_;
  pcl::PointCloud<PointXYZIR>::Ptr laser_cloud_in_ring_;

  pcl::PointCloud<PointType>::Ptr full_cloud_; // projected velodyne raw cloud, but saved in the form of 1-D matrix
  pcl::PointCloud<PointType>::Ptr full_info_cloud_; // same as fullCloud, but with intensity - range

  pcl::PointCloud<PointType>::Ptr ground_cloud_;
  pcl::PointCloud<PointType>::Ptr segmented_cloud_;
  pcl::PointCloud<PointType>::Ptr segmented_cloud_pure_;
  pcl::PointCloud<PointType>::Ptr outlier_cloud_;

  PointType nan_point_; // fill in fullCloud at each iteration

  cv::Mat range_mat_;   // range matrix for range image
  cv::Mat ground_mat_;  // ground matrix for ground cloud marking
  cv::Mat label_mat_;   // label matrix for segmentaiton marking
  int label_cnt_;

  std_msgs::Header cloud_header_;

  std::vector<std::pair<int8_t, int8_t> > neighbor_iterator_; // neighbor iterator for segmentaiton process

  uint16_t *all_pushed_idx_x_;     // array for tracking points of a segmented object
  uint16_t *all_pushed_idx_y_;

  uint16_t *queue_idx_x_;          // array for breadth-first search process of segmentation, for speed
  uint16_t *queue_idx_y_;

  //Parmameters
  std::string pointcloud_topic_;
  bool use_cloud_ring_;            // if true, ang_res_y and ang_bottom are not used

  // for vlp-16
  int scan_num_;
  int horizon_scan_;
  int ground_scan_idx_;
  float ang_res_x_;
  float ang_res_y_;
  float ang_bottom_;

  float sensor_minimum_range_;
  float sensor_mount_angle_;
  float segment_theta_;            // decrese this value may improve accuracy
  float segment_alpha_x_;
  float segment_alpha_y_;
  int   segment_valid_point_num_;
  int   segment_valid_line_num_;
};

#endif /* cloud_segment_CLOUD_SEGMENT_H_ */
