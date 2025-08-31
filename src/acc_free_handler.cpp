#include "mqtt_publisher_cpp/acc_free_handler.hpp"
#include "mqtt_publisher_cpp/publisher.hpp"
#include <chrono>

using namespace std::chrono;
using json = nlohmann::json;

namespace mqtt_publisher_cpp {

static inline double now_s() {
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

AccFreeHandler::AccFreeHandler(rclcpp::Node* node,
                               std::shared_ptr<Publisher> publisher,
                               const std::string& ros_topic,
                               const std::string& out_sensor_name,
                               double throttle_hz)
: node_(node), pub_(std::move(publisher)), sensor_(out_sensor_name) {
  set_throttle(throttle_hz);

  sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    ros_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&AccFreeHandler::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "AccFreeHandler -> %s @ %.2f Hz",
              ros_topic.c_str(), throttle_hz);
}

void AccFreeHandler::set_throttle(double hz) {
  min_period_ = 1.0 / std::max(0.1, hz);
}

void AccFreeHandler::cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  const double t = now_s();
  if (t - last_pub_ < min_period_) return;
  last_pub_ = t;

  const auto ts_sec = msg->header.stamp.sec;

  json j = {
    {"ver", 1},
    {"robotId", pub_->robot_id()},
    {"ts", ts_sec},
    {"frame", msg->header.frame_id},
    {"acc_free", {{"x", msg->vector.x}, {"y", msg->vector.y}, {"z", msg->vector.z}}},
    {"units", {{"acc", "m_s2"}}}
  };

  pub_->send(sensor_, j.dump());
}

} // namespace mqtt_publisher_cpp
