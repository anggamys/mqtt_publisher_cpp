#include <rclcpp/rclcpp.hpp>
#include "mqtt_publisher_cpp/publisher.hpp"
#include "mqtt_publisher_cpp/acc_free_handler.hpp"
#include "mqtt_publisher_cpp/rpy_handler.hpp"

using mqtt_publisher_cpp::Publisher;
using mqtt_publisher_cpp::AccFreeHandler;
using mqtt_publisher_cpp::RpyHandler;

class Orchestrator : public rclcpp::Node {
public:
  Orchestrator() : Node("mqtt_publisher_node") {
    // MQTT config params
    auto host       = declare_parameter<std::string>("broker_host", "127.0.0.1");
    auto port       = declare_parameter<int>("broker_port", 1883);
    auto username   = declare_parameter<std::string>("username", "mqtt");
    auto password   = declare_parameter<std::string>("password", "mqttpass");
    auto topic_base = declare_parameter<std::string>("topic_base", "site");
    auto robot_id   = declare_parameter<std::string>("robot_id", "robot1");
    auto qos        = declare_parameter<int>("qos", 0);
    auto retain     = declare_parameter<bool>("retain", false);
    auto keepalive  = declare_parameter<int>("keepalive", 60);

    Publisher::Config cfg;
    cfg.host = host;
    cfg.port = port;
    cfg.username = username;
    cfg.password = password;
    cfg.topic_base = topic_base;
    cfg.robot_id = robot_id;
    cfg.qos = qos;
    cfg.retain = retain;
    cfg.keepalive = keepalive;
    cfg.client_id = "ros2-" + robot_id + "-pub";

    publisher_ = std::make_shared<Publisher>(this, cfg);

    // per-handler params
    auto acc_topic = declare_parameter<std::string>("acc_free.ros_topic", "/asv/imu/acc_free");
    auto acc_thz   = declare_parameter<double>("acc_free.throttle_hz", 20.0);
    auto rpy_topic = declare_parameter<std::string>("rpy_deg.ros_topic", "/asv/imu/rpy_deg");
    auto rpy_thz   = declare_parameter<double>("rpy_deg.throttle_hz", 20.0);

    acc_ = std::make_unique<AccFreeHandler>(this, publisher_, acc_topic, "imu_acc_free", acc_thz);
    rpy_ = std::make_unique<RpyHandler>(this, publisher_, rpy_topic, "imu_rpy_deg", rpy_thz);

    RCLCPP_INFO(get_logger(), "mqtt_publisher_cpp ready.");
  }

private:
  std::shared_ptr<Publisher> publisher_;
  std::unique_ptr<AccFreeHandler> acc_;
  std::unique_ptr<RpyHandler> rpy_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Orchestrator>());
  rclcpp::shutdown();
  return 0;
}
