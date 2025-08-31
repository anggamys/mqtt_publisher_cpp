#include "mqtt_publisher_cpp/rpy_handler.hpp"
#include <chrono>
#include "nlohmann/json.hpp"

using namespace std::chrono;
using json = nlohmann::json;

namespace mqtt_publisher_cpp {

static inline double now_s() {
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

RpyHandler::RpyHandler(rclcpp::Node* node,
                       std::shared_ptr<Publisher> publisher,
                       const std::string& ros_topic,
                       const std::string& out_sensor_name,
                       double throttle_hz)
: node_(node), pub_(std::move(publisher)), sensor_(out_sensor_name) 
{
  set_throttle(throttle_hz);

  sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ros_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&RpyHandler::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "RpyHandler -> %s @ %.2f Hz", ros_topic.c_str(), throttle_hz);
}

void RpyHandler::set_throttle(double hz) {
  min_period_ = 1.0 / std::max(0.1, hz);
  RCLCPP_INFO(node_->get_logger(), "RpyHandler throttle updated -> %.2f Hz", hz);
}

void RpyHandler::cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  double t = now_s();
  if (t - last_pub_ < min_period_) return;
  last_pub_ = t;

  json j = {
    {"ver", 1},
    {"robotId", pub_->robot_id()},
    {"ts", msg->header.stamp.sec},
    {"frame", msg->header.frame_id},
    {"rpy_deg", {
      {"roll", msg->vector.x},
      {"pitch", msg->vector.y},
      {"yaw", msg->vector.z}
    }},
    {"units", {{"angle", "deg"}}}
  };

  pub_->send(sensor_, j.dump());
}

} // namespace mqtt_publisher_cpp
