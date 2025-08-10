#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <string>

namespace mqtt_publisher_cpp {

// Forward declare; implementasi ada di publisher.hpp/cpp
class Publisher;

class AccFreeHandler {
public:
  AccFreeHandler(rclcpp::Node* node,
                 std::shared_ptr<Publisher> publisher,
                 std::string ros_topic = "/asv/imu/acc_free",
                 std::string out_sensor_name = "imu_acc_free",
                 double throttle_hz = 20.0);

private:
  void cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  // Throttle (detik)
  double min_period_{0.0};
  double last_pub_{0.0}; // steady clock seconds

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  rclcpp::Node* node_{nullptr};                // tidak di-own
  std::shared_ptr<Publisher> pub_;
  std::string sensor_;
};

}  // namespace mqtt_publisher_cpp
