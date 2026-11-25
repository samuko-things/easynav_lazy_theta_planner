#ifndef EASYNAV_PLANNER__LAZY_THETA_SIMPLE_PLANNER_HPP_
#define EASYNAV_PLANNER__LAZY_THETA_SIMPLE_PLANNER_HPP_

#include <vector>

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"
#include "easynav_simple_common/SimpleMap.hpp"
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
 * @brief Stores the map metadata required by the planner.
 *
 * Contains map resolution, inverse resolution, origin coordinates,
 * and map dimensions. These values are used for coordinate conversion
 * and grid boundary checks.
 */
struct MapMetaData {
  double resolution{0.0};
  double inv_resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  int size_x{0};
  int size_y{0};

  // Helper to update all fields atomically from a costmap pointer
  void update(const SimpleMap &map) {
    resolution = map.resolution();
    inv_resolution = (resolution > 0.0) ? (1.0 / resolution) : 0.0;
    origin_x = map.origin_x();
    origin_y = map.origin_y();
    size_x = static_cast<int>(map.width());
    size_y = static_cast<int>(map.height());
  }
};

/// \brief A planner implementing the LazyTheta*-P algorithm on a SimpleMap grid.
class LazyThetaSimplePlanner : public PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  explicit LazyThetaSimplePlanner();

  /**
   * @brief Initializes the planner.
   *
   * Configures publishers, retrieves parameters, and prepares the planner
   * for path generation using the available map data.
   *
   * @throws std::runtime_error if initialization fails.
   */
  virtual void on_initialize() override;

  /**
   * @brief Updates the planner by computing a new path.
   *
   * Uses the current navigation state (including the robot's position and goal)
   * to generate a path based on the A* algorithm.
   *
   * @param nav_state The current navigation state (contains odometry and goal information).
   */
  void update(NavState & nav_state) override;

protected:
  double robot_radius_;        ///< Radius of the robot used for collision checking.
  double clearance_distance_;  ///< Minimum clearance distance from obstacles in meters.

  nav_msgs::msg::Path current_path_;  ///< The last computed path.

  /// Publisher for the computed navigation path.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::vector<GridNode> node_pool_; ///< Pool of grid nodes used during path search.
  std::vector<int> node_visited_id_; ///< Stores visitation identifiers for grid nodes.
  int run_id_ = 0; ///< Identifier for the current search iteration.

  MapMetaData map_meta_; ///< Cached costmap metadata.

  double clearance_cells_ = 0.2;

  /**
   * @brief Runs the Lazy Theta* algorithm to compute a path.
   *
   * Computes a path on the given simple map from the start pose to the goal pose.
   *
   * @param map The occupancy map used for path planning.
   * @param start The starting pose in world coordinates.
   * @param goal The target pose in world coordinates.
   * @param resolution The cell resolution of the map (in meters).
   * @return A sequence of poses representing the planned path.
   */
  std::vector<geometry_msgs::msg::Pose> plan_path(
    const SimpleMap & map,
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
   * @param map The occupancy map used for path planning.
   *
   * @return Goal node if a path is found, nullptr otherwise.
   */
  GridNode* lazy_theta_plan(
    GridNode* start_node,
    GridNode* goal_node,
    const SimpleMap & map);

  /**
   * @brief Checks whether a map cell is free, considering a clearance area.
   *
   * This function verifies if a cell and its surrounding cells (within the
   * specified clearance radius) are free of obstacles.
   *
   * @param map The occupancy map to query.
   * @param cx The x-coordinate of the cell.
   * @param cy The y-coordinate of the cell.
   * @param clearance_cells The clearance radius expressed in number of cells.
   * @return true if the cell and its clearance area are free, false otherwise.
   */
  bool is_free_with_clearance(
    const SimpleMap & map,
    int cx, int cy,
    double clearance_cells);

  GridNode pose_to_grid(const geometry_msgs::msg::Pose &pose);

  geometry_msgs::msg::Pose grid_to_pose(const GridNode &grid);

  int grid_to_map_index(const GridNode &grid);
  int grid_to_map_index(const int x, const int y);

  bool is_grid_on_map(const GridNode &grid);
  bool is_grid_on_map(const int x, const int y);

  bool is_map_cell_free(const GridNode &grid, const SimpleMap & map);
  bool is_map_cell_free(const int x, const int y, const SimpleMap & map);

  double euclidean_distance(const GridNode &a, const GridNode &b);

  bool line_of_sight(
    int x0, int y0, 
    int x1, int y1,
    const SimpleMap & map);

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

#endif  // EASYNAV_PLANNER__TEST_SIMPLEPLANNER_HPP_
