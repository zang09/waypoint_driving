#include "../include/waypoint_driving.h"

using namespace geometry_msgs;

WaypointDriving::WaypointDriving() :
    turn_mode_(false),
    line_num_(0),
    turn_rad_(1.25), //0.5
    border_dist_(turn_rad_*2),
    border_ang_(30*DEG2RAD),
    beta_(0.5)
{
    subscribeAndPublish();
}

WaypointDriving::~WaypointDriving()
{}

void WaypointDriving::subscribeAndPublish()
{
    sub_waypoint_flag_  = nh_.subscribe<rviz_flag_plugin::PointArray>("flag/points", 10, &WaypointDriving::waypointFlagHandler, this);
    sub_current_odom_   = nh_.subscribe<nav_msgs::Odometry>("odom", 100, &WaypointDriving::odomHandler, this);
    pub_motor_vel_      = nh_.advertise<geometry_msgs::Twist>("argos_mr/motor_vel", 10);
    pub_line_strip_     = nh_.advertise<visualization_msgs::Marker>("flag/line", 1);
    pub_virtual_points_ = nh_.advertise<visualization_msgs::Marker>("virtual/points", 1);
}

void WaypointDriving::waypointFlagHandler(const rviz_flag_plugin::PointArrayConstPtr &point_msg)
{
    target_points_.clear();
    target_points_.assign(point_msg->points.begin(), point_msg->points.end());

    calculateLineSegment();
}

void WaypointDriving::odomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    current_pose_.position    = odom_msg->pose.pose.position;
    current_pose_.orientation = odom_msg->pose.pose.orientation;
    current_yaw_ = getYawFromQuaternion(current_pose_.orientation);

    if(line_num_ < line_info_.size())
        calculateTargetVel();

    /*
    geometry_msgs::Twist vel;
    vel.linear.x = current_pose_.position.x;
    vel.linear.y = current_pose_.position.y;
    pub_motor_vel_.publish(vel);
    */
}

void WaypointDriving::calculateLineSegment()
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.5;
    line_strip.color.a = 1.0;
    line_strip.color.r = (float)0.09412;
    line_strip.color.g = (float)0.74118;
    line_strip.color.b = (float)0.89412;
    line_strip.pose.orientation.w = 1.0;

    line_info_.clear();
    for(size_t i=0; i<target_points_.size(); i++)
    {
        line_strip.points.push_back(target_points_.at(i));

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

    pub_line_strip_.publish(line_strip);
}

void WaypointDriving::calculateTargetVel()
{
    //Projection distance from line segment
    double projection_value = projectionLengthToLine(line_info_.at(line_num_).start_point,
                                                     line_info_.at(line_num_).end_point,
                                                     current_pose_.position);

    double virtual_point_dist = 2.0;
    double virtual_angle = atan2(projection_value, virtual_point_dist);
    double reference_angle = line_info_.at(line_num_).angle_rad - virtual_angle;
    double delta_angle = current_yaw_ - reference_angle;

    if(delta_angle < -PI) delta_angle += 2*PI;
    else if(delta_angle > PI) delta_angle -= 2*PI;

    double delta_x = current_pose_.position.x-line_info_.at(line_num_).end_point.x;
    double delta_y = current_pose_.position.y-line_info_.at(line_num_).end_point.y;
    double distance = sqrt(pow(delta_x,2)+pow(delta_y,2));

    if(distance < turn_rad_) {
        std::cout << "Distance: " << distance << ", " << turn_rad_ << std::endl;
        line_num_++;
    }
    if(line_num_ >= line_info_.size()) return;

    //not used
    double alpha = 0.05*beta_*abs(tanh(delta_angle/border_ang_));
    double right_vel = (1-alpha)*mean_vel_ + alpha*mean_vel_*(-tanh(delta_angle/border_ang_));
    double left_vel  = (1-alpha)*mean_vel_ + alpha*mean_vel_*(+tanh(delta_angle/border_ang_));

    Point project_p, virtual_p;
    project_p = projectionToLine(line_info_.at(line_num_).start_point,
                                 line_info_.at(line_num_).end_point,
                                 current_pose_.position);

    virtual_p.x = project_p.x + virtual_point_dist*cos(line_info_.at(line_num_).angle_rad);
    virtual_p.y = project_p.y + virtual_point_dist*sin(line_info_.at(line_num_).angle_rad);

    //Visualization Pointl
    visualization_msgs::Marker points;
    points.header.frame_id = "map";
    points.header.stamp = ros::Time::now();
    points.type = visualization_msgs::Marker::SPHERE_LIST;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.scale.x = points.scale.y = points.scale.z = 1.0;
    points.color.a = points.color.r = 1.0;
    points.points.push_back(virtual_p);

    pub_virtual_points_.publish(points);
}

double WaypointDriving::projectionLengthToLine(Point s_p, Point e_p, Point c_p)
{
    double delta_x = e_p.x - s_p.x;
    double delta_y = e_p.y - s_p.y;
    double delat_l = sqrt(pow(delta_x,2)+pow(delta_y,2));
    double sin_theta = delta_y / delat_l;
    double cos_theta = delta_x / delat_l;

    double result = -(c_p.x-s_p.x)*sin_theta + (c_p.y-s_p.x)*cos_theta;

    return result;
}

Point WaypointDriving::projectionToLine(Point s_p, Point e_p, Point c_p)
{
    double angle  = atan2(e_p.y-s_p.y, e_p.x-s_p.x);
    double value = -(c_p.x-s_p.x)*sin(angle) + (c_p.y-s_p.y)*cos(angle);

    Point p;
    p.x = c_p.x + value * sin(angle);
    p.y = c_p.y + value * (-cos(angle));

    return p;
}

double WaypointDriving::getYawFromQuaternion(Quaternion q_msg)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;

    tf::quaternionMsgToTF(q_msg, quat);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    return yaw;
}
