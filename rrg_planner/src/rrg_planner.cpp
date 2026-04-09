#include "rrg_planner/rrg_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <limits>
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
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());
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

double RRGPlanner::distance(double x1, double y1, double x2, double y2)
{
  // std::hypot - długość przeciwprostokątnej w trójkącie prostokątnym o przyprostokątnych (x2-x1) i (y2-y1)
  return std::hypot(x2 - x1, y2 - y1);
}

std::pair<double, double> RRGPlanner::sampleRandomPoint()
{
  // Pobieranie rozmiarów mapy w metrach
  double map_width_meters = costmap_->getSizeInMetersX();
  double map_height_meters = costmap_->getSizeInMetersY();
  // Pobieranie pozycję początkową mapy, aby ustalić dokładne granice, może wystąpić przesunięcie względem (0,0)
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();

  // Tworzenie dystrybucji losowych punktów w obrębie mapy
  std::uniform_real_distribution<double> x_dist(origin_x, origin_x + map_width_meters);
  std::uniform_real_distribution<double> y_dist(origin_y, origin_y + map_height_meters);

  // Pobieranie losowego punktu
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

bool RRGPlanner::isCollisionFree(double x1, double y1, double x2, double y2)
{
  
  double dist = distance(x1, y1, x2, y2);
  double resolution = costmap_->getResolution();
  
  // Dzielenie linii na segmenty o długości mniejszej niż połowa rozdzielczości mapy, by nie przegapić wąskich przeszkód
  int num_steps = std::ceil(dist / (resolution * 0.5));
  if (num_steps == 0) {
    return true; // Punkty leżą dokładnie w tym samym miejscu
  }
  
  // Obliczanie odległości kroków wzdłuż linii
  double x_step = (x2 - x1) / num_steps;
  double y_step = (y2 - y1) / num_steps;
  
  // Ruch po linii i sprawdzanie każdego punktu
  for (int i = 0; i <= num_steps; ++i) {
    double current_x = x1 + i * x_step;
    double current_y = y1 + i * y_step;
    
    unsigned int mx, my;
    
    // 5. Konwersja z metrów (world) na piksele (map)
    // Jeśli punkt wychodzi poza krawędź mapy, worldToMap zwraca false
    if (!costmap_->worldToMap(current_x, current_y, mx, my)) {
      return false; // Punkt poza mapą - traktowane jako kolizję
    }
    
    // 6. Odczytanie kosztu z mapy
    unsigned char cost = costmap_->getCost(mx, my);
    
    // W Nav2: 
    // 254 (LETHAL_OBSTACLE) to fizyczna przeszkoda
    // 253 (INSCRIBED_INFLATED_OBSTACLE) to obrys robota (uderzyłby brzegiem)
    // 255 (NO_INFORMATION) to teren nieznany
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      // Jeśli koszt to 253, 254 lub 255 - traktowane jako kolizję
      return false; 
    }
  }
  
  return true; 
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