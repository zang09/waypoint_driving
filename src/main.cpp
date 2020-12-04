#include "../include/waypoint_driving.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_driving");

    ROS_INFO("\033[1;32m---->\033[0m Waypoint Driving node is Started.");

    WaypointDriving wd;

    ros::spin();

    return 0;
}
