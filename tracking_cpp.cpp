#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "f110_msgs/msg/obstacle_array.hpp"

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_cpp")
  {
    obstacles_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
      "/perception/detection/raw_obstacles_cpp",   // DetectCppNode에서 발행
      rclcpp::QoS(10),
      std::bind(&TrackingNode::obstaclesCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_points",                            // Nav2 costmap에서 사용
      rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(),
      "TrackingNode: subscribing '/perception/detection/raw_obstacles_cpp', publishing 'obstacle_points'.");
  }

private:
  void obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg->header.stamp;
    cloud.header.frame_id = msg->header.frame_id;   // base_link

    const auto & obs = msg->obstacles;
    if (obs.empty()) {
      cloud.height = 1;
      cloud.width = 0;
      cloud.is_bigendian = false;
      cloud.is_dense = true;
      cloud_pub_->publish(cloud);
      return;
    }

    cloud.height = 1;
    cloud.width = obs.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obs.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & o : obs) {
      // DetectCppNode에서 s_center ~ 전방 거리, d_center ~ 옆 방향
      float x = static_cast<float>(o.s_center);
      float y = static_cast<float>(o.d_center);
      float z = 0.0f;

      *iter_x = x;
      *iter_y = y;
      *iter_z = z;

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_pub_->publish(cloud);
  }

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}