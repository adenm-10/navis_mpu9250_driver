#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

using namespace std::chrono_literals;

class OdomTrackerNode : public rclcpp::Node
{
public:
  OdomTrackerNode()
  : Node("odom_tracker_node"), cumulative_distance_(0.0)
  {
    // Declare and get parameter for target distance
    this->declare_parameter<double>("target_distance_m", 10.0);
    target_distance_m_ = this->get_parameter("target_distance_m").as_double();

    // Create odometry subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&OdomTrackerNode::odom_callback, this, std::placeholders::_1));

    // Create reset service
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset_distance",
      std::bind(&OdomTrackerNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Odometry tracker node started. Target distance: %.2f m", target_distance_m_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time current_time = msg->header.stamp;

    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }

    double dt = (current_time - last_time_).seconds();

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;

    double delta_dist = std::sqrt(vx * vx + vy * vy) * dt;
    cumulative_distance_ += delta_dist;

    if (cumulative_distance_ >= target_distance_m_) {
      RCLCPP_INFO(this->get_logger(), "Target distance reached: %.2f m", cumulative_distance_);
      // Optionally, you can take action here (e.g., publish stop command)
    }

    last_time_ = current_time;
    RCLCPP_INFO(this->get_logger(), "Cumulative Distance: %.2f m", cumulative_distance_);
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    cumulative_distance_ = 0.0;
    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    RCLCPP_INFO(this->get_logger(), "Cumulative distance reset!");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

  double cumulative_distance_;
  double target_distance_m_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
