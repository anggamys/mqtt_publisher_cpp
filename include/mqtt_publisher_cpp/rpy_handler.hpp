#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <string>
#include "mqtt_publisher_cpp/publisher.hpp"
#include "nlohmann/json.hpp"

namespace mqtt_publisher_cpp {

class RpyHandler {
public:
  RpyHandler(rclcpp::Node* node,
             std::shared_ptr<Publisher> publisher,
             const std::string& ros_topic = "/asv/imu/rpy_deg",
             const std::string& out_sensor_name = "imu_rpy_deg",
             double throttle_hz = 20.0);

  void set_throttle(double throttle_hz);

private:
  void cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  double min_period_{0.0}, last_pub_{0.0};
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  rclcpp::Node* node_;
  std::shared_ptr<Publisher> pub_;
  std::string sensor_;
};

} // namespace mqtt_publisher_cpp
