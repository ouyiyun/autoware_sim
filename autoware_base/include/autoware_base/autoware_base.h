#ifndef AUTOWARE_BASE_VEHICLE_BASE_H_
#define AUTOWARE_BASE_VEHICLE_BASE_H_

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

//! service
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetModelState.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

//! TODO: 直接向Gazebo 获取底层的信息
class AutowareBase {
public:
    AutowareBase();
    ~AutowareBase();
    void loop();

private:
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void ackermann_vel_callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
    void send_speed_callback(const ros::TimerEvent& e);
    void get_base_info_callback(const ros::TimerEvent& e);

    ros::Time last_vel_time_;
    ros::Time now_;
    ros::Timer send_speed_timer_;
    ros::Timer get_base_info_timer_;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber ackermann_vel_sub_;

    //! pub vel
    ros::Publisher wheel_right_rear_pub_;
    ros::Publisher wheel_left_rear_pub_;
    ros::Publisher steering_right_front_pub_;
    ros::Publisher steering_left_front_pub_;

    ros::ServiceClient get_link_state_client_;
    ros::ServiceClient get_model_state_client_;

    //! pub odom
    ros::Publisher odom_pub_;
    geometry_msgs::TransformStamped transformStamped_;
    tf2_ros::TransformBroadcaster br_;
    nav_msgs::Odometry odom_;

    //! pub pose vel and steer angle
    geometry_msgs::PoseStamped out_put_pose_; //! base_footprint
    std_msgs::Float64 out_put_vel_;
    std_msgs::Float64 out_put_steer_angle_;

    double control_rate_;

    double linear_speed_;
    double steering_angle_;
    //! maximum steering angle for ideal middle tire
    double ideal_max_steer_angle_;

    boost::mutex cmd_vel_mutex_;
    boost::mutex ackermann_vel_mutex_;

    std::string odom_frame_;
    std::string imu_frame_;
    std::string base_frame_;
    std::string robot_name_;

    //! vehicle params
    double wheel_tread_;
    double wheel_base_;
    double wheel_radius_;
    double max_steer_angle_;
};
#endif