#include "rrg_planner/rrg_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace rrg_planner
{

void RRGPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  RCLCPP_INFO(node_->get_logger(), "Konfiguracja pluginu RRG: %s", name_.c_str());
}

void RRGPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Czyszczenie pluginu RRG: %s", name_.c_str());
}

void RRGPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Aktywacja pluginu RRG: %s", name_.c_str());
}

void RRGPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Dezaktywacja pluginu RRG: %s", name_.c_str());
}

double RRGPlanner::distance(double x1, double y1, double x2, double y2)
{
  return std::hypot(x2 - x1, y2 - y1);
}

std::pair<double, double> RRGPlanner::sampleRandomPoint()
{
  double map_width_meters = costmap_->getSizeInMetersX();
  double map_height_meters = costmap_->getSizeInMetersY();

  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();

  std::uniform_real_distribution<double> x_dist(origin_x, origin_x + map_width_meters);
  std::uniform_real_distribution<double> y_dist(origin_y, origin_y + map_height_meters);

  return {x_dist(rng_gen_), y_dist(rng_gen_)};
}

int RRGPlanner::getNearestNodeId(double x, double y)
{
  int nearest_id = -1;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & node : graph_) {
    double dist = distance(x, y, node.x, node.y);

    if (dist < min_dist) {
      min_dist = dist;
      nearest_id = node.id;
    }
  }

  return nearest_id;
}

std::vector<int> RRGPlanner::getNearNeighbors(double x, double y, double radius)
{
  std::vector<int> neighbors;

  for (const auto & node : graph_) {
    double dist = distance(x, y, node.x, node.y);

    if (dist <= radius) {
      neighbors.push_back(node.id);
    }
  }

  return neighbors;
}

bool RRGPlanner::isCollisionFree(double x1, double y1, double x2, double y2)
{
  double dist = distance(x1, y1, x2, y2);
  double resolution = costmap_->getResolution();

  int num_steps = std::ceil(dist / (resolution * 0.5));

  if (num_steps == 0) {
    unsigned int mx, my;

    if (!costmap_->worldToMap(x1, y1, mx, my)) {
      return false;
    }

    unsigned char cost = costmap_->getCost(mx, my);

    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    return true;
  }

  double x_step = (x2 - x1) / num_steps;
  double y_step = (y2 - y1) / num_steps;

  for (int i = 0; i <= num_steps; ++i) {
    double current_x = x1 + i * x_step;
    double current_y = y1 + i * y_step;

    unsigned int mx, my;

    if (!costmap_->worldToMap(current_x, current_y, mx, my)) {
      return false;
    }

    unsigned char cost = costmap_->getCost(mx, my);

    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
  }

  return true;
}

nav_msgs::msg::Path RRGPlanner::extractPath(int start_id, int goal_id)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (start_id < 0 || goal_id < 0) {
    return path;
  }

  if (start_id >= static_cast<int>(graph_.size()) || goal_id >= static_cast<int>(graph_.size())) {
    return path;
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;

  int current_id = goal_id;
  int safety_counter = 0;

  while (current_id != -1 && safety_counter < static_cast<int>(graph_.size()) + 5) {
    const auto & node = graph_[current_id];

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    poses.push_back(pose);

    if (current_id == start_id) {
      break;
    }

    current_id = node.parent_id;
    safety_counter++;
  }

  std::reverse(poses.begin(), poses.end());
  path.poses = poses;

  return path;
}

nav_msgs::msg::Path RRGPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;

  RCLCPP_INFO(
  node_->get_logger(),
  "RRG: żądanie ścieżki z (%.2f, %.2f) do (%.2f, %.2f)",
  start_x,
  start_y,
  goal_x,
  goal_y
);

RCLCPP_INFO(node_->get_logger(), "RRG_DEBUG_NOWA_WERSJA_BEZ_PROSTEJ_LINII");
  graph_.clear();

  if (!isCollisionFree(start_x, start_y, start_x, start_y)) {
    RCLCPP_WARN(node_->get_logger(), "RRG: punkt startowy znajduje się w kolizji albo poza mapą.");
    return global_path;
  }

  if (!isCollisionFree(goal_x, goal_y, goal_x, goal_y)) {
    RCLCPP_WARN(node_->get_logger(), "RRG: punkt docelowy znajduje się w kolizji albo poza mapą.");
    return global_path;
  }


  int start_id = 0;
  graph_.push_back(GraphNode(start_x, start_y, start_id));
  graph_[start_id].cost_from_start = 0.0;
  graph_[start_id].parent_id = -1;

  int goal_id = -1;

  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    double random_x;
    double random_y;

    if (iteration % 10 == 0) {
      random_x = goal_x;
      random_y = goal_y;
    } else {
      auto random_point = sampleRandomPoint();
      random_x = random_point.first;
      random_y = random_point.second;
    }

    int nearest_id = getNearestNodeId(random_x, random_y);

    if (nearest_id < 0) {
      continue;
    }

    const auto & nearest_node = graph_[nearest_id];

    double dist_to_random = distance(nearest_node.x, nearest_node.y, random_x, random_y);

    if (dist_to_random < 1e-6) {
      continue;
    }

    double new_x;
    double new_y;

    if (dist_to_random <= step_size_) {
      new_x = random_x;
      new_y = random_y;
    } else {
      double scale = step_size_ / dist_to_random;
      new_x = nearest_node.x + (random_x - nearest_node.x) * scale;
      new_y = nearest_node.y + (random_y - nearest_node.y) * scale;
    }

    if (!isCollisionFree(nearest_node.x, nearest_node.y, new_x, new_y)) {
      continue;
    }

    int new_id = static_cast<int>(graph_.size());
    GraphNode new_node(new_x, new_y, new_id);

    int best_parent_id = nearest_id;
    double best_cost = graph_[nearest_id].cost_from_start +
      distance(nearest_node.x, nearest_node.y, new_x, new_y);

    std::vector<int> near_neighbors = getNearNeighbors(new_x, new_y, search_radius_);

    for (int neighbor_id : near_neighbors) {
      const auto & neighbor = graph_[neighbor_id];

      if (!isCollisionFree(neighbor.x, neighbor.y, new_x, new_y)) {
        continue;
      }

      double candidate_cost = neighbor.cost_from_start +
        distance(neighbor.x, neighbor.y, new_x, new_y);

      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        best_parent_id = neighbor_id;
      }
    }

    new_node.parent_id = best_parent_id;
    new_node.cost_from_start = best_cost;

    graph_.push_back(new_node);

    graph_[best_parent_id].neighbors.push_back(new_id);
    graph_[new_id].neighbors.push_back(best_parent_id);

    for (int neighbor_id : near_neighbors) {
      if (neighbor_id == best_parent_id) {
        continue;
      }

      const auto & neighbor = graph_[neighbor_id];

      if (!isCollisionFree(neighbor.x, neighbor.y, new_x, new_y)) {
        continue;
      }

      graph_[neighbor_id].neighbors.push_back(new_id);
      graph_[new_id].neighbors.push_back(neighbor_id);

      double cost_through_new = graph_[new_id].cost_from_start +
        distance(new_x, new_y, neighbor.x, neighbor.y);

      if (cost_through_new < graph_[neighbor_id].cost_from_start) {
        graph_[neighbor_id].cost_from_start = cost_through_new;
        graph_[neighbor_id].parent_id = new_id;
      }
    }

    double dist_to_goal = distance(new_x, new_y, goal_x, goal_y);

    if (dist_to_goal <= step_size_ && isCollisionFree(new_x, new_y, goal_x, goal_y)) {
      goal_id = static_cast<int>(graph_.size());

      GraphNode goal_node(goal_x, goal_y, goal_id);
      goal_node.parent_id = new_id;
      goal_node.cost_from_start = graph_[new_id].cost_from_start + dist_to_goal;

      graph_.push_back(goal_node);

      graph_[new_id].neighbors.push_back(goal_id);
      graph_[goal_id].neighbors.push_back(new_id);

      RCLCPP_INFO(
        node_->get_logger(),
        "RRG: znaleziono ścieżkę po %d iteracjach. Liczba węzłów: %zu",
        iteration,
        graph_.size()
      );

      global_path = extractPath(start_id, goal_id);

      if (!global_path.poses.empty()) {
        global_path.poses.front().pose.orientation = start.pose.orientation;
        global_path.poses.back().pose.orientation = goal.pose.orientation;
      }

      return global_path;
    }
  }

  RCLCPP_WARN(
    node_->get_logger(),
    "RRG: nie znaleziono ścieżki po %d iteracjach. Liczba węzłów: %zu",
    max_iterations_,
    graph_.size()
  );

  return global_path;
}

}  // namespace rrg_planner

PLUGINLIB_EXPORT_CLASS(rrg_planner::RRGPlanner, nav2_core::GlobalPlanner)