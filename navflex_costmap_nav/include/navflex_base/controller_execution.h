#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navflex_base/navflex_execution_base.h"
#include "navflex_utility/robot_information.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace navflex_costmap_nav {

/**
 * @class ControllerExecution
 * @brief Execution class that runs a nav2_core::Controller plugin in a thread.
 *
 * Manages the velocity-command computation loop for path following.
 * Wraps a Controller plugin and drives its execution cycle, publishing
 * velocity commands and tracking execution state.
 *
 * Follows the same pattern as PlannerExecution for consistency.
 */
class ControllerExecution : public NavflexExecutionBase {
 public:
  typedef std::shared_ptr<ControllerExecution> Ptr;

  /**
   * @brief Internal controller execution states
   */
  enum ControllerState {
    INITIALIZED,   ///< Execution created, waiting for plan
    STARTED,       ///< Thread started, about to enter the control loop
    PLANNING,      ///< Computing velocity commands
    NO_PLAN,       ///< Started without a valid plan
    MAX_RETRIES,   ///< Exceeded maximum retries
    PAT_EXCEEDED,  ///< Exceeded patience time
    EMPTY_PLAN,    ///< Received an empty plan
    INVALID_PLAN,  ///< Controller rejected the plan
    NO_LOCAL_CMD,  ///< No valid velocity command this cycle
    GOT_LOCAL_CMD, ///< Got a valid velocity command
    ARRIVED_GOAL,  ///< Goal reached
    CANCELED,      ///< Canceled by action client or server
    STOPPED,       ///< Stopped rigorously (via should_exit_)
    INTERNAL_ERROR ///< Unknown error
  };

  /**
   * @brief Constructor
   * @param name Plugin name (used for logging and parameter namespacing)
   * @param controller_ptr Pointer to the loaded controller plugin
   * @param goal_checker Raw (non-owning) pointer to the goal checker
   * @param robot_info Robot state provider
   * @param node Lifecycle node for parameter access and logging
   * @param vel_pub Publisher for velocity commands (geometry_msgs::Twist)
   * @param current_goal_pub Publisher for the current goal pose
   */
  ControllerExecution(
      const std::string& name,
      const nav2_core::Controller::Ptr& controller_ptr,
      nav2_core::GoalChecker* goal_checker,
      const navflex_utility::RobotInformation::ConstPtr& robot_info,
      const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
      const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& vel_pub,
      const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& current_goal_pub);

  virtual ~ControllerExecution();

  /**
   * @brief Return current execution state (thread-safe)
   */
  ControllerState getState() const;

  /**
   * @brief Return the last computed velocity command (thread-safe)
   */
  geometry_msgs::msg::TwistStamped getVelocityCmd() const;

  /**
   * @brief Whether the controller is still in its motion loop
   */
  bool isMoving() const;

  /**
   * @brief Whether the patience duration has been exceeded
   */
  bool isPatienceExceeded() const;

  /**
   * @brief Return the configured loop frequency (Hz)
   */
  double getFrequency() const { return frequency_; }

  /**
   * @brief Set a new plan for the controller.
   * Can be called while the controller is running (plan update).
   */
  void setNewPlan(const nav_msgs::msg::Path& path);

  /**
   * @brief Check if a new plan has been set since last getNewPlan() call
   */
  bool hasNewPlan();

  /**
   * @brief Retrieve the pending plan and clear the new_plan_ flag
   */
  nav_msgs::msg::Path getNewPlan();

  /**
   * @brief Request cancellation of the control loop
   */
  virtual bool cancel() override;

  /**
   * @brief Start the controller execution thread
   */
  virtual bool start() override;

  /**
   * @brief Dynamic reconfigure callback
   */
  rcl_interfaces::msg::SetParametersResult reconfigure(
      std::vector<rclcpp::Parameter> parameters);

 protected:
  /// Main execution loop - runs in dedicated thread
  virtual void run() override;

 private:
  uint32_t computeVelocityCmd(const geometry_msgs::msg::PoseStamped& robot_pose,
                              const geometry_msgs::msg::TwistStamped& robot_velocity,
                              geometry_msgs::msg::TwistStamped& vel_cmd,
                              std::string& message);
  void setState(ControllerState state);
  void publishZeroVelocity();
  bool setControllerFrequency(double frequency);

  // Plugin
  nav2_core::Controller::Ptr controller_;

  // Non-owning plugin references (owned by ControllerServer)
  nav2_core::GoalChecker* goal_checker_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

  // Plan
  nav_msgs::msg::Path plan_;
  bool new_plan_;
  mutable std::mutex plan_mtx_;

  // State
  mutable std::mutex state_mtx_;
  ControllerState state_;
  bool moving_;

  // Timing
  mutable std::mutex lct_mtx_;
  rclcpp::Time last_call_time_;
  rclcpp::Time last_valid_cmd_time_;
  rclcpp::Time start_time_;

  // Last computed velocity
  mutable std::mutex vel_cmd_mtx_;
  geometry_msgs::msg::TwistStamped vel_cmd_stamped_;

  // Robot pose (updated each cycle)
  geometry_msgs::msg::PoseStamped robot_pose_;

  // Parameters
  mutable std::mutex configuration_mutex_;
  double frequency_;
  rclcpp::Duration patience_;
  int max_retries_;

  std::string robot_frame_;
  std::string global_frame_;

  std::shared_ptr<rclcpp::Rate> loop_rate_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_handle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      dyn_params_handler_;
};

}  // namespace navflex_costmap_nav
