#include "autoware_base/autoware_base.h"

#include <boost/math/constants/constants.hpp>

const double zero = 10 * std::numeric_limits<double>::epsilon();

AutowareBase::AutowareBase() {
    ros::NodeHandle private_nh;

    private_nh.param<double>("wheel_base", wheel_base_, 2.95);
    private_nh.param<double>("wheel_tread", wheel_tread_, 1.55);
    private_nh.param<double>("wheel_radius", wheel_radius_, 0.341);
    private_nh.param<double>("str_angle", max_steer_angle_, 0.60);
    private_nh.param<double>("control_rate", control_rate_, 50.0);

    private_nh.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    private_nh.param<std::string>("base_frame", base_frame_, std::string("base_footprint"));
    private_nh.param<std::string>("imu_frame", imu_frame_, std::string("base_imu_link"));
    private_nh.param<std::string>("robot_name", robot_name_, std::string("autoware"));

    //! sub
    cmd_vel_sub_ = private_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &AutowareBase::cmd_vel_callback, this);
    ackermann_vel_sub_ = private_nh.subscribe<ackermann_msgs::AckermannDrive>("ackermann_cmd", 10, &AutowareBase::ackermann_vel_callback, this);
    send_speed_timer_ = private_nh.createTimer(ros::Duration(1.0 / control_rate_), &AutowareBase::send_speed_callback, this);

    //! pub vel
    wheel_right_rear_pub_ = private_nh.advertise<std_msgs::Float64>("rear_right_velocity_controller/command", 1, true);
    wheel_left_rear_pub_ = private_nh.advertise<std_msgs::Float64>("rear_left_velocity_controller/command", 1, true);
    steering_right_front_pub_ = private_nh.advertise<std_msgs::Float64>("front_right_steering_position_controller/command", 1, true);
    steering_left_front_pub_ = private_nh.advertise<std_msgs::Float64>("front_left_steering_position_controller/command", 1, true);

    //! service
    get_model_state_client_ = private_nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    //! get base info and pub
    get_base_info_timer_ = private_nh.createTimer(ros::Duration(1.0 / control_rate_), &AutowareBase::get_base_info_callback, this);
    odom_pub_ = private_nh.advertise<nav_msgs::Odometry>("odom", 10);

    //! init vel
    linear_speed_ = .0;
    steering_angle_ = .0;

    //! turning radius for maximum steer angle just with the inside tire
    //! tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
    double max_radius = wheel_base_ / std::tan(max_steer_angle_);

    // radius of inside tire is rMax, so radius of the ideal middle tire(rIdeal) is rMax + treadwidth / 2
    double ideal_radius = max_radius + wheel_tread_ / 2.0;

    //! maximum steering angle for ideal middle tire whice we use
    // tan(angle) = wheelbase / radius
    ideal_max_steer_angle_ = std::atan2(wheel_base_, ideal_radius);
}

AutowareBase::~AutowareBase() {}

void AutowareBase::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    try {
        cmd_vel_mutex_.lock();
        linear_speed_ = msg->linear.x / wheel_radius_;
        steering_angle_ = std::max(-ideal_max_steer_angle_, std::min(ideal_max_steer_angle_, msg->angular.z));
        last_vel_time_ = ros::Time::now();
        cmd_vel_mutex_.unlock();
    } catch (...) {
        cmd_vel_mutex_.unlock();
    }
}

void AutowareBase::ackermann_vel_callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
    try {
        ackermann_vel_mutex_.lock();
        linear_speed_ = msg->speed / wheel_radius_;
        steering_angle_ = std::max(-ideal_max_steer_angle_, std::min(ideal_max_steer_angle_, (double)msg->steering_angle));
        last_vel_time_ = ros::Time::now();
        ackermann_vel_mutex_.unlock();
    } catch (...) {
        ackermann_vel_mutex_.unlock();
    }
}

void AutowareBase::send_speed_callback(const ros::TimerEvent& e) {
    //! now that these values are published, we
    //! reset the velocity, so that if we don't hear new
    //! ones for the next timestep that we time out;
    std_msgs::Float64 output_wheel_right_rear, output_wheel_left_rear, output_steering_right_front, output_steering_left_front;

    if ((ros::Time::now() - last_vel_time_).toSec() > 1.0) {
        linear_speed_ = 0.0;
        // steering_angle_ = 0.0;
        std_msgs::Float64 vel, steer;
        // ROS_INFO("cmd is too old!");

        output_wheel_right_rear.data = linear_speed_;
        output_wheel_left_rear.data = linear_speed_;

        output_steering_right_front.data = steering_angle_;
        output_steering_left_front.data = steering_angle_;

        //! pub
        wheel_left_rear_pub_.publish(output_wheel_left_rear);
        wheel_right_rear_pub_.publish(output_wheel_right_rear);
        steering_left_front_pub_.publish(output_steering_left_front);
        steering_right_front_pub_.publish(output_steering_right_front);

        return;
    }

    //! go straight
    if (std::fabs(steering_angle_) <= zero) {
        steering_angle_ = .0;

        output_wheel_left_rear.data = linear_speed_;
        output_wheel_right_rear.data = linear_speed_;

        output_steering_left_front.data = steering_angle_;
        output_steering_right_front.data = steering_angle_;
    } else {
        double radius = wheel_base_ / std::tan(std::fabs(steering_angle_));

        double radius_left_rear = radius - (std::copysign(1.0, steering_angle_)) * wheel_tread_ / 2.0;
        double radius_right_rear = radius + (std::copysign(1.0, steering_angle_)) * wheel_tread_ / 2.0;
        double radius_left_front = radius_left_rear;
        double radius_right_front = radius_right_rear;

        output_wheel_left_rear.data = linear_speed_ * radius_left_rear / radius;
        output_wheel_right_rear.data = linear_speed_ * radius_right_rear / radius;

        output_steering_left_front.data = std::copysign(1.0, steering_angle_) * std::atan(wheel_base_ / radius_left_front);
        output_steering_right_front.data = std::copysign(1.0, steering_angle_) * std::atan(wheel_base_ / radius_right_front);
    }
    //! pub
    wheel_left_rear_pub_.publish(output_wheel_left_rear);
    wheel_right_rear_pub_.publish(output_wheel_right_rear);
    steering_left_front_pub_.publish(output_steering_left_front);
    steering_right_front_pub_.publish(output_steering_right_front);
}

//! TODO:
void AutowareBase::get_base_info_callback(const ros::TimerEvent& e) {
    //! model state: get base_footprint position and pub odom
    now_ = ros::Time::now();
    gazebo_msgs::GetModelState base_footprint_srv;
    base_footprint_srv.request.model_name = robot_name_;
    get_model_state_client_.call(base_footprint_srv);

    if (!base_footprint_srv.response.success) {
        return;
    }

    transformStamped_.header.stamp = now_;
    transformStamped_.header.frame_id = odom_frame_;
    transformStamped_.child_frame_id = base_frame_;
    transformStamped_.transform.translation.x = base_footprint_srv.response.pose.position.x;
    transformStamped_.transform.translation.y = base_footprint_srv.response.pose.position.y;
    transformStamped_.transform.translation.z = 0.0;
    transformStamped_.transform.rotation.x = base_footprint_srv.response.pose.orientation.x;
    transformStamped_.transform.rotation.y = base_footprint_srv.response.pose.orientation.y;
    transformStamped_.transform.rotation.z = base_footprint_srv.response.pose.orientation.z;
    transformStamped_.transform.rotation.w = base_footprint_srv.response.pose.orientation.w;
    br_.sendTransform(transformStamped_);

    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;
    odom_.header.stamp = now_;
    odom_.pose.pose.position = base_footprint_srv.response.pose.position;
    odom_.pose.pose.orientation = base_footprint_srv.response.pose.orientation;
    odom_.twist.twist = base_footprint_srv.response.twist;
    odom_.twist.covariance = {1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                              0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 0.10};
    odom_.pose.covariance = {1e-9, 0, 0, 0,   0, 0, 0, 1e-3, 1e-9, 0, 0,   0, 0, 0, 1e6, 0, 0, 0,
                             0,    0, 0, 1e6, 0, 0, 0, 0,    0,    0, 1e6, 0, 0, 0, 0,   0, 0, 1e3};
    odom_pub_.publish(odom_);
}

void AutowareBase::loop() {}

int main(int argc, char** argv) {
    ros::init(argc, argv, "autoware_base_node");

    AutowareBase autoware;
    ros::spin();
    return 0;
}