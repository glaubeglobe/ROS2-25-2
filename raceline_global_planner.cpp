#include "f1tenth_raceline_planner/raceline_global_planner.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace f1tenth_raceline_planner
{

void RacelinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RacelinePlanner: failed to lock parent node in configure()");
  }

  node->declare_parameter(name_ + ".raceline_csv", std::string(""));
  node->get_parameter(name_ + ".raceline_csv", raceline_csv_path_);

  if (raceline_csv_path_.empty()) {
    RCLCPP_WARN(node->get_logger(),
      "RacelinePlanner: raceline_csv parameter is empty. Will use straight-line fallback.");
    raceline_loaded_ = false;
  } else {
    loadRacelineFromCsv(raceline_csv_path_);
  }

  RCLCPP_INFO(node->get_logger(),
    "RacelinePlanner configured. CSV: '%s'", raceline_csv_path_.c_str());
}

void RacelinePlanner::cleanup()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "RacelinePlanner cleanup");
  }
  raceline_path_.poses.clear();
  raceline_loaded_ = false;
}

void RacelinePlanner::activate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "RacelinePlanner activate");
  }
}

void RacelinePlanner::deactivate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "RacelinePlanner deactivate");
  }
}

nav_msgs::msg::Path RacelinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;

  auto node = node_.lock();
  if (!node) {
    return path;
  }

  std::string global_frame = costmap_ros_ ?
    costmap_ros_->getGlobalFrameID() : start.header.frame_id;
  path.header.frame_id = global_frame;
  path.header.stamp = node->now();

  if (raceline_loaded_ && !raceline_path_.poses.empty()) {
    path = raceline_path_;
    path.header.stamp = node->now();
    RCLCPP_DEBUG(node->get_logger(),
      "RacelinePlanner: returning raceline with %zu poses", path.poses.size());
    return path;
  }

  // fallback: start -> goal 직선
  RCLCPP_WARN(node->get_logger(),
    "RacelinePlanner: raceline not loaded. Using straight-line fallback plan.");

  constexpr int N = 50;
  for (int i = 0; i <= N; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(N);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = start.header;
    pose.pose.position.x =
      start.pose.position.x + t * (goal.pose.position.x - start.pose.position.x);
    pose.pose.position.y =
      start.pose.position.y + t * (goal.pose.position.y - start.pose.position.y);
    pose.pose.position.z = 0.0;

    pose.pose.orientation = start.pose.orientation;

    path.poses.push_back(pose);
  }

  return path;
}

void RacelinePlanner::loadRacelineFromCsv(const std::string & file_path)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  raceline_path_.poses.clear();
  raceline_path_.header.frame_id = "map";
  raceline_path_.header.stamp = node->now();

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(),
      "RacelinePlanner: failed to open raceline CSV: %s", file_path.c_str());
    raceline_loaded_ = false;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (const std::exception &) {
        values.clear();
        break;
      }
    }

    if (values.size() < 2) {
      continue;
    }

    double x = values[0];
    double y = values[1];
    double yaw = 0.0;
    if (values.size() >= 3) {
      yaw = values[2];
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = raceline_path_.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    raceline_path_.poses.push_back(pose);
  }

  file.close();

  raceline_loaded_ = !raceline_path_.poses.empty();

  if (raceline_loaded_) {
    RCLCPP_INFO(node->get_logger(),
      "RacelinePlanner: loaded %zu poses from raceline CSV '%s'.",
      raceline_path_.poses.size(), file_path.c_str());
  } else {
    RCLCPP_WARN(node->get_logger(),
      "RacelinePlanner: raceline CSV '%s' had no valid poses.", file_path.c_str());
  }
}

}  // namespace f1tenth_raceline_planner

PLUGINLIB_EXPORT_CLASS(
  f1tenth_raceline_planner::RacelinePlanner,
  nav2_core::GlobalPlanner)