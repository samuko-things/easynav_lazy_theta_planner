#include <queue>
#include <unordered_map>
#include <cmath>

#include "easynav_lazy_theta_simple_planner/LazyThetaSimplePlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

double compute_path_length(const nav_msgs::msg::Path & path)
{
  double total_length = 0.0;

  if (path.poses.size() < 2) {
    return 0.0;
  }

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    total_length += std::sqrt(dx * dx + dy * dy);
  }

  return total_length;
}


// Simple path smoother: moving average over a sliding window in XY.
// Keeps endpoints unchanged to preserve exact start and goal.
static void smooth_path(std::vector<geometry_msgs::msg::Pose> & poses, int window_size = 5)
{
  if (poses.size() < 3 || window_size <= 1) {
    return;
  }

  // Ensure window_size is odd and at least 3
  if (window_size < 3) {
    window_size = 3;
  }
  if (window_size % 2 == 0) {
    window_size += 1;
  }

  const int half = window_size / 2;
  const size_t n = poses.size();
  std::vector<geometry_msgs::msg::Pose> original = poses;

  // Leave first and last pose untouched
  for (size_t i = 1; i + 1 < n; ++i) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int count = 0;

    const int begin = static_cast<int>(std::max<size_t>(0,
        i > static_cast<size_t>(half) ? i - half : 0));
    const int end = static_cast<int>(std::min<size_t>(n - 1, i + half));

    for (int j = begin; j <= end; ++j) {
      sum_x += original[j].position.x;
      sum_y += original[j].position.y;
      ++count;
    }

    if (count > 0) {
      poses[i].position.x = sum_x / static_cast<double>(count);
      poses[i].position.y = sum_y / static_cast<double>(count);
    }
  }
}


LazyThetaSimplePlanner::LazyThetaSimplePlanner()
{
  NavState::register_printer<nav_msgs::msg::Path>(
    [](const nav_msgs::msg::Path & path) {
      std::ostringstream ret;

      ret << "{ " << rclcpp::Time(path.header.stamp).seconds() << " } Path with " <<
        path.poses.size() << " poses and length " <<
        compute_path_length(path) << " m.";

      return ret.str();
    });
}

void
LazyThetaSimplePlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".robot_radius", 0.3);
  node->declare_parameter<double>(plugin_name + ".clearance_distance", 0.2);
  node->get_parameter<double>(plugin_name + ".robot_radius", robot_radius_);
  node->get_parameter<double>(plugin_name + ".clearance_distance", clearance_distance_);

  path_pub_ = get_node()->create_publisher<nav_msgs::msg::Path>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path", 10);
}

void
LazyThetaSimplePlanner::update(NavState & nav_state)
{
  current_path_.poses.clear();

  if (!nav_state.has("goals")) {return;}
  if (!nav_state.has("robot_pose")) {return;}

  const auto & goals = nav_state.get<nav_msgs::msg::Goals>("goals");

  if (goals.goals.empty()) {
    nav_state.set("path", current_path_);
    return;
  }

  if (!nav_state.has("map")) {
    RCLCPP_WARN(get_node()->get_logger(), "LazyThetaSimplePlanner::update map map not found");
    return;
  }

  SimpleMap map_typed;
  if (nav_state.has("map")) {
    map_typed = nav_state.get<SimpleMap>("map");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "There is yet no a map");
    return;
  }

  const auto & robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto & goal = goals.goals.front().pose;
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  auto downsampled_map = map_typed.downsample(0.1);


  map_meta_.update(*downsampled_map);
  unsigned int map_size = map_meta_.size_x * map_meta_.size_y;

  // Fallback memory check in case the costmap dynamically resizes during runtime
  if (node_pool_.size() != map_size) {
    node_pool_.resize(map_size);
    node_visited_id_.resize(map_size);
  }
  
  clearance_cells_ = (robot_radius_ + clearance_distance_)*map_meta_.inv_resolution;

  const auto clock_type = get_node()->get_clock()->get_clock_type();
  rclcpp::Time latest_stamp(robot_pose.header.stamp, clock_type);
  if (rclcpp::Time(goals.goals.front().header.stamp,
      latest_stamp.get_clock_type()) > latest_stamp)
  {
    latest_stamp = rclcpp::Time(goals.goals.front().header.stamp, latest_stamp.get_clock_type());
  }

  if (goals.header.frame_id != tf_info.map_frame) {
    RCLCPP_WARN(get_node()->get_logger(),
      "LazyThetaSimplePlanner::update goals frame is not map (%s)", goals.header.frame_id.c_str());
    return;
  }

  if (!downsampled_map->check_bounds_metric(goal.position.x, goal.position.y)) {
    RCLCPP_WARN(get_node()->get_logger(),
      "LazyThetaSimplePlanner::update goal (%lf, %lf) outside the map", goal.position.x, goal.position.y);
    return;
  }

  auto poses = plan_path(*downsampled_map, robot_pose.pose.pose, goal);
  if (!poses.empty()) {
    // Apply a light smoothing to the raw grid path
    smooth_path(poses);

    current_path_.poses.clear();
    current_path_.header.stamp = latest_stamp;
    current_path_.header.frame_id = goals.header.frame_id;

    for (const auto & pose : poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = goals.header.frame_id;
      pose_stamped.header.stamp = latest_stamp;
      pose_stamped.pose = pose;
      current_path_.poses.push_back(pose_stamped);
    }

    if (path_pub_->get_subscription_count() > 0) {
      path_pub_->publish(current_path_);
    }
  }

  nav_state.set("path", current_path_);
}


bool
LazyThetaSimplePlanner::is_free_with_clearance(
  const SimpleMap & map,
  int cx, int cy,
  double clearance_cells)
{
  int width = map.width();
  int height = map.height();
  int rad = std::ceil(clearance_cells);

  for (int dx = -rad; dx <= rad; ++dx) {
    for (int dy = -rad; dy <= rad; ++dy) {
      int nx = cx + dx;
      int ny = cy + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
      if (std::hypot(dx, dy) <= clearance_cells && map.at(nx, ny)) {
        return false;
      }
    }
  }
  return true;
}

std::vector<geometry_msgs::msg::Pose>
LazyThetaSimplePlanner::plan_path(
  const SimpleMap & map,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal)
{
  // --- FAST FLAT MEMORY RESETS ---
  std::fill(node_visited_id_.begin(), node_visited_id_.end(), 0);
  run_id_++;

  RCLCPP_DEBUG(get_node()->get_logger(), "Running Lazy Theta Planner ============");
  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%lf m, %lf m) ->  (%lf m, %lf m)",
    start.position.x, start.position.y,
    goal.position.x, goal.position.y);

  auto [sx, sy] = map.metric_to_cell(start.position.x, start.position.y);
  auto [gx, gy] = map.metric_to_cell(goal.position.x, goal.position.y);

  RCLCPP_DEBUG(get_node()->get_logger(), "Path from (%d, %d) ->  (%d, %d)",
    sx, sy, gx, gy);

  // Start & Goal Node Setup
  GridNode raw_start(sx, sy);
  GridNode* start_node = get_node_from_pool(raw_start.x, raw_start.y);

  GridNode raw_goal(gx, gy);
  GridNode* goal_node = get_node_from_pool(raw_goal.x, raw_goal.y);

  GridNode* best_goal = lazy_theta_plan(
    start_node, 
    goal_node, 
    map
  );

  // Path Reconstruction
  std::vector<geometry_msgs::msg::Pose> path;

  if (!best_goal) {
    return path;
  }

  GridNode* node = best_goal;
  while (node)
  {
    geometry_msgs::msg::Pose pose;
    pose = grid_to_pose(*node);
    path.push_back(pose);

    if (node->parent == node) {
      break;
    }
    node = node->parent;
  }

  std::reverse(path.begin(), path.end());
  return densify_path(path, goal);
}




GridNode* LazyThetaSimplePlanner::lazy_theta_plan(
  GridNode* start_node,
  GridNode* goal_node,
  const SimpleMap & map)
{
  start_node->g_cost = 0.0;
  start_node->h_cost = euclidean_distance(*start_node, *goal_node);
  start_node->parent = start_node;
  start_node->is_in_queue = true;

  open_queue_.push(start_node);

  while (!open_queue_.empty())
  {
    GridNode* current = open_queue_.top();
    open_queue_.pop();

    current->is_in_queue = false;

    if (current->parent && current->parent->parent && current->parent != current)
    {
      GridNode* grand_parent = current->parent->parent;
      if (line_of_sight(current->x, current->y, grand_parent->x, grand_parent->y, map))
      {
        double current_g_cost = grand_parent->g_cost + euclidean_distance(*current, *grand_parent);
        if (current_g_cost < current->g_cost)
        {
          current->parent = grand_parent;
          current->g_cost = current_g_cost;
        }
      }
    }

    if (current->x == goal_node->x && current->y == goal_node->y) {
      clear_queue();
      return current;
    }

    for (const auto & d : dirs)
    {
      int nx = current->x + d.dx;
      int ny = current->y + d.dy;
      
      if (is_grid_on_map(nx, ny) && is_map_cell_free(nx, ny, map) && is_free_with_clearance(map, nx, ny, clearance_cells_)) {
        GridNode* nbr_node = get_node_from_pool(nx, ny);
        double g_cost = current->g_cost + d.dist;
        double h_cost = euclidean_distance(*nbr_node, *goal_node);
        double f_cost = g_cost + h_cost;

        if ((nbr_node->g_cost + nbr_node->h_cost) > f_cost)
        {
          nbr_node->g_cost = g_cost;
          nbr_node->h_cost = h_cost;
          nbr_node->parent = current;

          if(!nbr_node->is_in_queue){
            nbr_node->is_in_queue = true;
            open_queue_.push(nbr_node);
          }
        }
      }
    }

  }

  clear_queue();
  return nullptr;
}

GridNode LazyThetaSimplePlanner::pose_to_grid(const geometry_msgs::msg::Pose &pose)
{
  int gx = static_cast<int>((pose.position.x - map_meta_.origin_x) * map_meta_.inv_resolution);
  int gy = static_cast<int>((pose.position.y - map_meta_.origin_y) * map_meta_.inv_resolution);

  return GridNode(gx, gy);
}

geometry_msgs::msg::Pose LazyThetaSimplePlanner::grid_to_pose(const GridNode &grid)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = grid.x * map_meta_.resolution + map_meta_.origin_x;
  pose.position.y = grid.y * map_meta_.resolution + map_meta_.origin_y;
  pose.position.z = 0.0;

  return pose;
}

int LazyThetaSimplePlanner::grid_to_map_index(const GridNode &grid_node)
{
  return static_cast<int>(grid_node.y * map_meta_.size_x + grid_node.x);
}

int LazyThetaSimplePlanner::grid_to_map_index(const int x, const int y)
{
  return static_cast<int>(y * map_meta_.size_x + x);
}

bool LazyThetaSimplePlanner::is_grid_on_map(const GridNode &grid)
{
  return (grid.x >= 0 && grid.x < map_meta_.size_x &&
          grid.y >= 0 && grid.y < map_meta_.size_y);
}

bool LazyThetaSimplePlanner::is_grid_on_map(const int x, const int y)
{
  return (x >= 0 && x < map_meta_.size_x &&
          y >= 0 && y < map_meta_.size_y);
}

bool LazyThetaSimplePlanner::is_map_cell_free(const GridNode &grid, const SimpleMap & map)
{
  return (map.at(grid.x, grid.y) == 0);
}

bool LazyThetaSimplePlanner::is_map_cell_free(const int x, const int y, const SimpleMap & map)
{
  return (map.at(x, y) == 0);
}

double LazyThetaSimplePlanner::euclidean_distance(const GridNode &a, const GridNode &b)
{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}


bool LazyThetaSimplePlanner::line_of_sight(
  int x0, int y0, 
  int x1, int y1,
  const SimpleMap & map)
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int max_x = static_cast<int>(map_meta_.size_x);
  int max_y = static_cast<int>(map_meta_.size_y);

  while (true)
  {
    // Safety Guard: Check map boundaries
    if (x0 < 0 || x0 >= max_x || y0 < 0 || y0 >= max_y) {
      return false;
    }

    if ((map.at(x0, y0) != 0) || !is_free_with_clearance(map, x0, y0, clearance_cells_)) {
      return false;
    }

    if (x0 == x1 && y0 == y1) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) { 
      err -= dy; 
      x0 += sx; 
    }
    if (e2 < dx)  { 
      err += dx; 
      y0 += sy; 
    }
  }

  return true;
}


GridNode* LazyThetaSimplePlanner::get_node_from_pool(int x, int y) {
  int index = grid_to_map_index(x, y);
  GridNode* node = &node_pool_[index];
  if (node_visited_id_[index] != run_id_) 
  {
    node->x = x;
    node->y = y;
    node->g_cost = std::numeric_limits<double>::infinity();
    node->h_cost = 0.0;
    node->parent = nullptr;
    node->is_in_queue = false;

    node_visited_id_[index] = run_id_;
  }
  return node;
};


std::vector<geometry_msgs::msg::Pose>
LazyThetaSimplePlanner::add_straight_line_poses(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  double resolution)
{
  std::vector<geometry_msgs::msg::Pose> out;

  double dx = end.position.x - start.position.x;
  double dy = end.position.y - start.position.y;
  double dist = std::hypot(dx, dy);

  if (dist == 0.0) {
    return out;
  }

  int steps = std::max(1, static_cast<int>(dist / resolution));

  for (int i = 1; i <= steps; ++i)
  {
    geometry_msgs::msg::Pose p = start;
    p.position.x = start.position.x + dx * (static_cast<double>(i) / steps);
    p.position.y = start.position.y + dy * (static_cast<double>(i) / steps);
    out.push_back(p);
  }

  return out;
}



std::vector<geometry_msgs::msg::Pose>
LazyThetaSimplePlanner::densify_path(
  const std::vector<geometry_msgs::msg::Pose> & poses, 
  const geometry_msgs::msg::Pose & goal)
{
  if (poses.empty()) {
    return {};
  }

  // 1. Interpolate and densify positions
  std::vector<geometry_msgs::msg::Pose> dense_poses;
  dense_poses.push_back(poses.front());

  for (size_t i = 1; i < poses.size(); ++i)
  {
    auto seg = add_straight_line_poses(
      poses[i - 1],
      poses[i],
      map_meta_.resolution);

    dense_poses.insert(dense_poses.end(), seg.begin(), seg.end());
  }

  // 2. Calculate yaw orientations for intermediate waypoints
  for (size_t i = 0; i < dense_poses.size() - 1; ++i)
  {
    double dx = dense_poses[i + 1].position.x - dense_poses[i].position.x;
    double dy = dense_poses[i + 1].position.y - dense_poses[i].position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    dense_poses[i].orientation.x = q.x();
    dense_poses[i].orientation.y = q.y();
    dense_poses[i].orientation.z = q.z();
    dense_poses[i].orientation.w = q.w();
  }

  // 3. Force the absolute last waypoint to match the exact goal orientation
  dense_poses.back().orientation = goal.orientation;

  return dense_poses;
}




}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::LazyThetaSimplePlanner, easynav::PlannerMethodBase)
