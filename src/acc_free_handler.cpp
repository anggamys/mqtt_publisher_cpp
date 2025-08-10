#include "mqtt_publisher_cpp/acc_free_handler.hpp"
#include "mqtt_publisher_cpp/publisher.hpp"

#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>  // std::max

using namespace std::chrono;

namespace mqtt_publisher_cpp {

static inline double now_s() {
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

AccFreeHandler::AccFreeHandler(rclcpp::Node* node,
                               std::shared_ptr<Publisher> publisher,
                               std::string ros_topic,
                               std::string out_sensor_name,
                               double throttle_hz)
: node_(node),
  pub_(std::move(publisher)),
  sensor_(std::move(out_sensor_name)) {
  min_period_ = 1.0 / std::max(0.1, throttle_hz);

  sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    ros_topic,
    rclcpp::SensorDataQoS(),   // cocok untuk data sensor
    std::bind(&AccFreeHandler::cb, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "AccFreeHandler -> %s @ %.2f Hz",
              ros_topic.c_str(), throttle_hz);
}

void AccFreeHandler::cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  const double t = now_s();
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
      << "\"acc_free\":{\"x\":" << msg->vector.x
      << ",\"y\":" << msg->vector.y
      << ",\"z\":" << msg->vector.z << "},"
      << "\"units\":{\"acc\":\"m_s2\"}"
      << "}";

  pub_->send(sensor_, oss.str());
}

} // namespace mqtt_publisher_cpp
