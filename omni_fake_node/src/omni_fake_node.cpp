#include "omni_fake_node/omni_fake_node.hpp"

OmniFakeNode::OmniFakeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("omni_fake_node", options),
  x_(0.0), y_(0.0), theta_(0.0),
  vx_(0.0), vy_(0.0), wz_(0.0),
  is_first_update_(true)
{
  // 声明参数
  declare_parameter<double>("update_rate", 50.0);  // Hz
  declare_parameter<double>("cmd_vel_timeout", 0.5);  // 秒
  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_link_frame", "base_link");

  // 获取参数
  update_rate_ = get_parameter("update_rate").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_link_frame_ = get_parameter("base_link_frame").as_string();

  // 创建订阅者
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10,
    std::bind(&OmniFakeNode::cmd_vel_callback, this, std::placeholders::_1)
  );

  // 创建发布者
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/odom",
    10
  );

  // 创建 TF 广播器
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 创建定时器
  auto period_ms = static_cast<int32_t>(1000.0 / update_rate_);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&OmniFakeNode::timer_callback, this)
  );

  last_update_time_ = now();
  last_cmd_vel_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "OmniFakeNode started with update_rate=%.1f Hz, cmd_vel_timeout=%.1f s, odom_frame='%s', base_link_frame='%s'",
    update_rate_, cmd_vel_timeout_, odom_frame_.c_str(), base_link_frame_.c_str()
  );
}

OmniFakeNode::~OmniFakeNode()
{
}

void OmniFakeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx_ = msg->linear.x;
  vy_ = msg->linear.y;
  wz_ = msg->angular.z;
  last_cmd_vel_time_ = now();
}

void OmniFakeNode::timer_callback()
{
  rclcpp::Time current_time = now();

  // 检查 cmd_vel 是否超时
  rclcpp::Duration time_since_cmd = current_time - last_cmd_vel_time_;
  if (time_since_cmd.seconds() > cmd_vel_timeout_) {
    // 超时，停止机器人运动
    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;
  }

  // 计算时间差
  rclcpp::Duration dt = current_time - last_update_time_;
  double dt_sec = dt.seconds();

  if (dt_sec <= 0.0) {
    last_update_time_ = current_time;
    return;
  }

  // 简单的积分模型：更新位置和姿态
  // 在全向底盘的情况下，我们需要考虑当前的姿态来处理速度
  // 这里使用世界坐标系中的积分（假设 vx, vy 已经是世界坐标）
  // 或者在机器人坐标系中进行转换

  // 方案：假设 vx, vy 是相对于机器人的，需要转换到世界坐标系
  double cos_theta = std::cos(theta_);
  double sin_theta = std::sin(theta_);

  // 在机器人坐标系中的速度转换到世界坐标系
  double vx_world = vx_ * cos_theta - vy_ * sin_theta;
  double vy_world = vx_ * sin_theta + vy_ * cos_theta;

  // 更新位置
  x_ += vx_world * dt_sec;
  y_ += vy_world * dt_sec;
  theta_ += wz_ * dt_sec;

  // 归一化 theta 到 [-pi, pi]
  while (theta_ > M_PI) {
    theta_ -= 2 * M_PI;
  }
  while (theta_ < -M_PI) {
    theta_ += 2 * M_PI;
  }

  last_update_time_ = current_time;

  // 计算四元数
  double half_theta = theta_ * 0.5;
  double qz = std::sin(half_theta);
  double qw = std::cos(half_theta);

  // 发布里程表信息
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp = current_time;
  odom_msg->header.frame_id = odom_frame_;
  odom_msg->child_frame_id = base_link_frame_;

  // 设置位置
  odom_msg->pose.pose.position.x = x_;
  odom_msg->pose.pose.position.y = y_;
  odom_msg->pose.pose.position.z = 0.0;

  // 设置姿态（四元数）
  odom_msg->pose.pose.orientation.x = 0.0;
  odom_msg->pose.pose.orientation.y = 0.0;
  odom_msg->pose.pose.orientation.z = qz;
  odom_msg->pose.pose.orientation.w = qw;

  // 设置位置的协方差（可选）
  odom_msg->pose.covariance[0] = 0.1;
  odom_msg->pose.covariance[7] = 0.1;
  odom_msg->pose.covariance[14] = 0.1;
  odom_msg->pose.covariance[21] = 0.1;
  odom_msg->pose.covariance[28] = 0.1;
  odom_msg->pose.covariance[35] = 0.1;

  // 设置速度
  odom_msg->twist.twist.linear.x = vx_;
  odom_msg->twist.twist.linear.y = vy_;
  odom_msg->twist.twist.linear.z = 0.0;
  odom_msg->twist.twist.angular.x = 0.0;
  odom_msg->twist.twist.angular.y = 0.0;
  odom_msg->twist.twist.angular.z = wz_;

  // 发布里程表
  odom_pub_->publish(*odom_msg);

  // 发布 TF：odom -> base_link
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = current_time;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_link_frame_;

  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = qz;
  transform.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniFakeNode>());
  rclcpp::shutdown();
  return 0;
}
