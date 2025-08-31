#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

namespace mqtt_publisher_cpp {

class Publisher;

class AccFreeHandler {
public:
  AccFreeHandler(rclcpp::Node* node,
                 std::shared_ptr<Publisher> publisher,
                 const std::string& ros_topic = "/asv/imu/acc_free",
                 const std::string& out_sensor_name = "imu_acc_free",
                 double throttle_hz = 20.0);

  void set_throttle(double hz);

private:
  void cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  double min_period_{0.0};
  double last_pub_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  rclcpp::Node* node_{nullptr};
  std::shared_ptr<Publisher> pub_;
  std::string sensor_;
};

} // namespace mqtt_publisher_cpp
