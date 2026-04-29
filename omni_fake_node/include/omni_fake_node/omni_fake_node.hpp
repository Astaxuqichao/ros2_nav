#ifndef OMNI_FAKE_NODE_HPP_
#define OMNI_FAKE_NODE_HPP_

#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OmniFakeNode : public rclcpp::Node
{
public:
  explicit OmniFakeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OmniFakeNode();

private:
  // 回调函数
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();

  // 发布者和订阅者
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // TF 广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 状态变量
  double x_, y_, theta_;           // 位置和姿态
  double vx_, vy_, wz_;            // 速度命令
  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_vel_time_;  // 最后一次收到 cmd_vel 的时间
  bool is_first_update_;

  // 参数
  double update_rate_;             // Hz
  double cmd_vel_timeout_;         // cmd_vel 超时时间 (s)
  std::string odom_frame_;
  std::string base_link_frame_;
};

#endif  // OMNI_FAKE_NODE_HPP_
