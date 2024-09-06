#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdometryCalculator : public rclcpp::Node
{
public:
    OdometryCalculator() : Node("odom"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // Initialize wheelbase (distance between wheels)
        wheelbase_ = 0.5; // meters (example value)

        // Subscribing to wheel speed topics
        left_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/microROS/wheel_speed_left", 10,
            std::bind(&OdometryCalculator::leftWheelCallback, this, std::placeholders::_1));
        
        right_wheel_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/microROS/wheel_speed_right", 10,
            std::bind(&OdometryCalculator::rightWheelCallback, this, std::placeholders::_1));

        // Publisher for odometry
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Timer for periodic odometry updates
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&OdometryCalculator::updateOdometry, this));

        // Transform broadcaster for publishing the transform between /odom and /base_link
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void leftWheelCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        v_L_ = msg->data;
    }

    void rightWheelCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        v_R_ = msg->data;
    }

    void updateOdometry()
    {
        double dt = 0.1; // time step in seconds (this should be synchronized to the actual timer period)

        // Compute linear and angular velocities
        double v = (v_L_ + v_R_) / 2.0;
        double omega = (v_R_ - v_L_) / wheelbase_;

        // Update the robot's pose
        x_ += v * std::cos(theta_) * dt;
        y_ += v * std::sin(theta_) * dt;
        theta_ += omega * dt;

        // Normalize theta to the range [-pi, pi]
        theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

        // Publish the odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // Orientation (convert theta to quaternion)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Velocities
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = omega;

        // Publish the odometry message
        odometry_publisher_->publish(odom_msg);

        // Publish the transform from odom to base_link
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = this->now();
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_wheel_subscriber_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // Timer for periodic odometry updates
    rclcpp::TimerBase::SharedPtr timer_;

    // TF broadcaster for odom to base_link
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Robot state variables
    double x_, y_, theta_;  // Position and orientation
    double v_L_ = 0.0, v_R_ = 0.0;  // Left and right wheel speeds
    double wheelbase_;  // Distance between wheels
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
