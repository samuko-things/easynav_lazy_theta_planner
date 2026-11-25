#ifndef EASYNAV_COSTMAP_PLANNER__LAZY_THETA_COSTMAP_PLANNER_HPP_
#define EASYNAV_COSTMAP_PLANNER__LAZY_THETA_COSTMAP_PLANNER_HPP_

#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @brief Represents a grid node used by the Lazy Theta* search algorithm.
 *
 * Stores the node's grid coordinates, path cost, heuristic cost,
 * queue status, and parent pointer for path reconstruction.
 */
struct GridNode
{
  int x{0};
  int y{0};

  double g_cost{0.0};
  double h_cost{0.0};
  bool is_in_queue{false};
  GridNode *parent{nullptr};

  GridNode(int _x = 0, int _y = 0) : x(_x), y(_y) {}
};

/**
 * @brief Comparator for the priority queue used by the OPEN list.
 *
 * Orders nodes according to their estimated total cost:
 * f(n) = g(n) + h(n).
 *
 * Nodes with lower estimated total costs are given higher priority.
 */
struct CompareNode
{
  bool operator()(const GridNode* a, const GridNode* b) const
  {
    return (a->g_cost + a->h_cost) > (b->g_cost + b->h_cost);
  }
};

/**
 * @brief Represents a movement direction between neighboring grid cells.
 *
 * Stores the relative movement in the X and Y directions and
 * the corresponding movement distance.
 */
struct Dir
{ int dx; 
  int dy; 
  double dist; 
};

/**
 * @brief Stores costmap metadata required by the planner.
 *
 * Contains map resolution, inverse resolution, origin coordinates,
 * and map dimensions. These values are used for coordinate conversion
 * and grid boundary checks.
 */
struct CostmapMeta {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const Costmap2D & map) {
    // if (!map) return;
    resolution = map.getResolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = map.getOriginX();
    origin_y = map.getOriginY();
    size_x = static_cast<int>(map.getSizeInCellsX());
    size_y = static_cast<int>(map.getSizeInCellsY());
  }
};


class LazyThetaCostmapPlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   *
   * Initializes internal parameters and configuration values.
   */
  explicit LazyThetaCostmapPlanner();

  /**
   * @brief Initializes the planner.
   *
   * Loads planner parameters, sets up ROS publishers,
   * and prepares the costmap-based planning environment.
   *
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

  /**
   * @brief Executes a planning cycle using the current navigation state.
   *
   * Computes a path from the robot's current pose to the goal using A*.
   *
   * @param nav_state Current shared navigation state (input/output).
   */
  void update(NavState & nav_state) override;

protected:
  int los_shortcut_cost_limit_; ///< Strict limit for line-of-sight shortcuts through the inflation layer (value ranges from 1-100)
  double cost_travel_multiplier_; ///< Weight for penalizing traversal through inflation (try values between 1.0 to 3.0)
  bool continuous_replan_;    ///< Wheter replan path at freq time
  nav_msgs::msg::Path current_path_;  ///< Most recently computed path.
  geometry_msgs::msg::Pose current_goal_;  ///< Current goal.

  CostmapMeta costmap_meta_;

  std::vector<GridNode> node_pool_; ///< Pool of grid nodes used during path search.
  std::vector<int> node_visited_id_; ///< Stores visitation identifiers for grid nodes.
  int run_id_ = 0; ///< Identifier for the current search iteration.

  /// Publisher for the computed navigation path (for visualization or monitoring).
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  /**
   * @brief Internal path planning routine.
   *
   * Computes a path on the given costmap from the start pose to the goal pose.
   *
   * @param map The costmap to plan over.
   * @param start The robot's starting pose in world coordinates.
   * @param goal The goal pose in world coordinates.
   * @return A vector of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> plan_path(
    const Costmap2D & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal);

  /**
   * @brief Executes the Lazy Theta* search algorithm.
   *
   * Performs grid-based any-angle path planning using lazy
   * line-of-sight checking and returns the goal node when
   * a valid path is found.
   * 
   * @param start_node Starting grid node.
   * @param goal_node Goal grid node.
   * @param char_map Costmap data.
   * @param size_x Horizontal size of the costmap.
   *
   * @return Goal node if a path is found, nullptr otherwise.
   */
  GridNode* lazy_theta_plan(
    GridNode* start_node,
    GridNode* goal_node,
    const unsigned char* char_map,
    unsigned int size_x);

  GridNode pose_to_grid(const geometry_msgs::msg::Pose &pose);

  geometry_msgs::msg::Pose grid_to_pose(const GridNode &grid);

  int grid_to_map_index(const GridNode &grid);
  int grid_to_map_index(const int x, const int y);

  double get_grid_cost(const GridNode &grid, const unsigned char* char_map);

  bool is_grid_on_map(const GridNode &grid);
  bool is_grid_on_map(const int x, const int y);

  bool is_map_cell_free(const GridNode &grid, const unsigned char* char_map);
  bool is_map_cell_free(const int x, const int y, const unsigned char* char_map);

  double euclidean_distance(const GridNode &a, const GridNode &b);

  bool line_of_sight(
    int x0, int y0, 
    int x1, int y1,
    const unsigned char* char_map,
    unsigned int size_x) const;

  std::vector<geometry_msgs::msg::Pose> add_straight_line_poses(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    double resolution);

  std::vector<geometry_msgs::msg::Pose> densify_path(
    const std::vector<geometry_msgs::msg::Pose> & poses, 
    const geometry_msgs::msg::Pose & goal);

  /**
   * @brief Retrieves a grid node from the node pool.
   *
   * Returns a node corresponding to the specified grid coordinates,
   * reusing allocated nodes where possible.
   *
   * @param x X-coordinate of the grid node.
   * @param y Y-coordinate of the grid node.
   * @return Pointer to the corresponding grid node.
   */
  GridNode* get_node_from_pool(int x, int y);

  /**
   * @brief OPEN list containing nodes awaiting expansion.
   *
   * Nodes are prioritized according to their estimated total cost:
   * f(n) = g(n) + h(n).
   */
  std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode> open_queue_;

  /**
   * @brief Clears the OPEN list.
   *
   * Resets the priority queue after search completion
   * or cancellation.
   */
  void clear_queue(){
    open_queue_ = std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode>();
  };

  /**
   * @brief Defines the eight possible movement directions.
   *
   * Includes four cardinal directions and four diagonal directions.
   * Each direction stores its relative grid movement and distance.
   */
  Dir dirs[8] = {
    {-1,  0, 1.0},
    { 1,  0, 1.0},
    { 0, -1, 1.0},
    { 0,  1, 1.0},
    {-1, -1, 1.4142},
    {-1,  1, 1.4142},
    { 1, -1, 1.4142},
    { 1,  1, 1.4142}
  };

};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_PLANNER__LAZY_THETA_COSTMAP_PLANNER_HPP_
