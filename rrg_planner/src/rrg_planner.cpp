#include "rrg_planner/rrg_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rrg_planner
{

void RRGPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(node_->get_logger(), "Konfiguracja pluginu: %s", name_.c_str());
}

void RRGPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Czyszczenie pluginu: %s", name_.c_str());
}

void RRGPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Aktywacja pluginu: %s", name_.c_str());
}

void RRGPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Dezaktywacja pluginu: %s", name_.c_str());
}

nav_msgs::msg::Path RRGPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  RCLCPP_INFO(node_->get_logger(), "Żądanie ścieżki z (%.2f, %.2f) do (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // === TUTAJ W PRZYSZŁOŚCI POJAWI SIĘ LOGIKA RRG ===
  // Zwracana linia prosta póki co
  global_path.poses.push_back(start);
  global_path.poses.push_back(goal);

  return global_path;
}

}  // namespace rrg_planner

//  REJESTRUJE PLUGIN
PLUGINLIB_EXPORT_CLASS(rrg_planner::RRGPlanner, nav2_core::GlobalPlanner)