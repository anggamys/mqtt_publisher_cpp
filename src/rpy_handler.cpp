#include "mqtt_publisher_cpp/rpy_handler.hpp"
#include <chrono>
#include <sstream>
#include <iomanip>

using namespace std::chrono;

namespace mqtt_publisher_cpp {

static inline double now_s2() {
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

RpyHandler::RpyHandler(rclcpp::Node* node,
                       std::shared_ptr<Publisher> publisher,
                       std::string ros_topic,
                       std::string out_sensor_name,
                       double throttle_hz)
: node_(node), pub_(std::move(publisher)), sensor_(std::move(out_sensor_name)) {
  min_period_ = 1.0 / std::max(0.1, throttle_hz);
  sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    ros_topic, rclcpp::SensorDataQoS(),
    std::bind(&RpyHandler::cb, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "RpyHandler -> %s @ %.2f Hz", ros_topic.c_str(), throttle_hz);
}

void RpyHandler::cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  double t = now_s2();
  if (t - last_pub_ < min_period_) return;
  last_pub_ = t;

  const auto ts_sec = static_cast<long>(msg->header.stamp.sec);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "{"
      << "\"ver\":1,"
      << "\"robotId\":\"" << pub_->robot_id() << "\","
      << "\"ts\":" << ts_sec << ","
      << "\"frame\":\"" << msg->header.frame_id << "\","
      << "\"rpy_deg\":{\"roll\":"  << msg->vector.x
      << ",\"pitch\":" << msg->vector.y
      << ",\"yaw\":"   << msg->vector.z << "},"
      << "\"units\":{\"angle\":\"deg\"}"
      << "}";
  pub_->send(sensor_, oss.str());
}

}
