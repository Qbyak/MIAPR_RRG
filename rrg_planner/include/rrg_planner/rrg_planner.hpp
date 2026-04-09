#ifndef RRG_PLANNER__RRG_PLANNER_HPP_
#define RRG_PLANNER__RRG_PLANNER_HPP_

#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"


#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace rrg_planner
{

class RRGPlanner : public nav2_core::GlobalPlanner
{
public:
  RRGPlanner() = default;
  ~RRGPlanner() = default;

  /**
   * @brief Metoda wywoływana raz podczas ładowania pluginu.
   * Pobieranie parametrów, inicjalizacja zmiennych, itp.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Czyszczenie pamięci po pluginie.
   */
  void cleanup() override;

  /**
   * @brief Aktywacja pluginu (gdy system nawigacji staje się aktywny).
   */
  void activate() override;

  /**
   * @brief Dezaktywacja pluginu.
   */
  void deactivate() override;

  // ========================================================================
  // GŁÓWNA LOGIKA PLANERA
  // ========================================================================

  /**
   * @brief Metoda tworząca ścieżkę. To tutaj zaimplementujesz swój algorytm RRG!
   * @param start Pozycja początkowa robota
   * @param goal Pozycja docelowa zadana przez użytkownika
   * @return Wygenerowana ścieżka (nav_msgs::msg::Path)
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Zmienne systemowe ułatwiające pracę wewnątrz pluginu
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  
  // Wskaźnik na globalną mapę kosztów - niezbędny żeby nie wchodzić w ściany
  nav2_costmap_2d::Costmap2D * costmap_;
  
  std::string global_frame_, name_;

  // ========================================================================
  // TU BĘDĄ ZMIENNE DLA ALGORYTMU RRG
  // ========================================================================

};

}  // namespace rrg_planner

#endif  // RRG_PLANNER__RRG_PLANNER_HPP_