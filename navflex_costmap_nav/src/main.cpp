#include <memory>

#include "navflex_costmap_nav/navflex_costmap_nav.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navflex_costmap_nav::CostmapNavNode>();

  // MultiThreadedExecutor is required so that lifecycle transition callbacks
  // (on_configure / on_activate / on_deactivate) can run concurrently with
  // bond heartbeat and other callbacks.  With a single-threaded executor the
  // createBond() / destroyBond() calls inside the lifecycle callbacks need the
  // executor to keep spinning to process their internal pub/sub, which causes a
  // deadlock → service response timeout → lifecycle_manager retry loop.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
