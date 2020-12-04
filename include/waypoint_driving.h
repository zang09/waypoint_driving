#ifndef waypoint_driving_WAYPOINT_DRIVING_H_
#define waypoint_driving_WAYPOINT_DRIVING_H_

#include <ros/ros.h>
#include <rviz_flag_plugin/PointArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#define PI       3.1415912
#define RAD2DEG  180/PI
#define DEG2RAD  PI/180

struct Line
{
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;
    double angle_rad;
    double angle_deg;
};

class WaypointDriving
{
public:
    WaypointDriving();
    ~WaypointDriving();

    void subscribeAndPublish();

    void waypointFlagHandler(const rviz_flag_plugin::PointArrayConstPtr &point_msg);

    void odomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg);

    void calculateLineSegment();

    void calculateTargetVel();

    double projectionLengthToLine(geometry_msgs::Point s_p, geometry_msgs::Point e_p, geometry_msgs::Point c_p);

    geometry_msgs::Point projectionToLine(geometry_msgs::Point s_p, geometry_msgs::Point e_p, geometry_msgs::Point c_p);

    double getYawFromQuaternion(geometry_msgs::Quaternion q_msg);
private:
    // ROS
    ros::NodeHandle nh_;

    ros::Subscriber sub_waypoint_flag_;
    ros::Subscriber sub_current_odom_;
    ros::Publisher  pub_motor_vel_;
    ros::Publisher  pub_line_strip_;
    ros::Publisher pub_virtual_points_;

    // Variable
    double current_yaw_;
    geometry_msgs::Pose  current_pose_;

    geometry_msgs::Point origin_point_;
    std::vector<Line> line_info_;
    std::vector<geometry_msgs::Point> target_points_;

    // Control Param
    bool turn_mode_;
    int line_num_;
    const double turn_rad_;    // turn radius [m]
    const double border_dist_; // boundary distance
    const double border_ang_;  // boundary angle (deg -> rad) turn angle difference satuated at border_ang_
    const double beta_;        // chose 0.1~0.9 begger, turn faster
    double mean_vel_;          // straight speed m/s
};

#endif /* waypoint_driving_WAYPOINT_DRIVING_H_ */
