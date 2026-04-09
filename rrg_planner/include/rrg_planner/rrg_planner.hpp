#ifndef RRG_PLANNER__RRG_PLANNER_HPP_
#define RRG_PLANNER__RRG_PLANNER_HPP_

#include <string>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"


#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace rrg_planner
{

struct GraphNode {
  double x;
  double y;
  int id;
  std::vector<int> neighbors; // Lista ID połączonych sąsiadów
  
  // Zmienne przydatne później do algorytmu Dijkstry/A*
  double cost_from_start;
  int parent_id; 

  GraphNode(double x_in, double y_in, int id_in) 
  : x(x_in), y(y_in), id(id_in), cost_from_start(0.0), parent_id(-1) {}
};

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

  // Generator liczb pseudolosowych Mersenne Twister
  std::mt19937 rng_gen_;
  // ========================================================================
  // TU BĘDĄ ZMIENNE DLA ALGORYTMU RRG
  // ========================================================================
  std::vector<GraphNode> graph_;        // graf przechowujący wszystkie węzły
  double step_size_ = 0.5;         // Jak daleko robimy krok od najbliższego węzła
  double search_radius_ = 1.0;     // Promień, w którym szukamy sąsiadów do połączenia
  int max_iterations_ = 10000;     // Zabezpieczenie przed nieskończoną pętlą
  double goal_tolerance_ = 0.25;   // Odległość od mety uznawana za sukces

  // --- FUNKCJE POMOCNICZE ---
  
  // Losowanie punktu na mapie
  std::pair<double, double> sampleRandomPoint();
  
  // Szukanie najbliższego istniejącego węzła do wylosowanego punktu
  int getNearestNodeId(double x, double y);
  
  // Sprawdzenie, czy można połączyć dwa punkty bez kolizji (czy linia między nimi jest wolna od przeszkód)
  bool isCollisionFree(double x1, double y1, double x2, double y2);
  
  // Znajdowanie sąsiadów w promieniu search_radius_ do nowego węzła
  std::vector<int> getNearNeighbors(double x, double y, double radius);
  
  // Obliczanie odległości między dwoma punktami
  double distance(double x1, double y1, double x2, double y2);

  // Ekstrakcja ścieżki z grafu po znalezieniu węzła docelowego
  nav_msgs::msg::Path extractPath(int start_id, int goal_id);
};

}  // namespace rrg_planner

#endif  // RRG_PLANNER__RRG_PLANNER_HPP_