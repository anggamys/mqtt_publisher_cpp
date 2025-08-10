#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <nlohmann/json.hpp> // header-only JSON (tambahkan di project atau ganti manual format JSON)
#include <chrono>
#include <ctime>

using json = nlohmann::json;
using namespace std::chrono_literals;

class MqttPublisherNode : public rclcpp::Node {
public:
  MqttPublisherNode() : Node("mqtt_publisher_cpp") {
    // Params
    broker_host_ = this->declare_parameter<std::string>("broker_host", "127.0.0.1");
    broker_port_ = this->declare_parameter<int>("broker_port", 1883);
    username_    = this->declare_parameter<std::string>("username", "factory:mqtt");
    password_    = this->declare_parameter<std::string>("password", "mqttpass");
    topic_       = this->declare_parameter<std::string>("topic", "site/robot1/sensors/imu");
    double rate  = this->declare_parameter<double>("rate_hz", 1.0);

    mosquitto_lib_init();
    client_ = mosquitto_new(("ros2-mqtt-pub-" + std::to_string(std::time(nullptr))).c_str(), true, nullptr);
    if (!client_) {
      RCLCPP_FATAL(get_logger(), "Failed to create mosquitto client");
      throw std::runtime_error("mosquitto_new failed");
    }

    if (mosquitto_username_pw_set(client_, username_.c_str(), password_.c_str()) != MOSQ_ERR_SUCCESS) {
      RCLCPP_FATAL(get_logger(), "Failed to set username/password");
      throw std::runtime_error("mosquitto_username_pw_set failed");
    }

    int rc = mosquitto_connect(client_, broker_host_.c_str(), broker_port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_FATAL(get_logger(), "MQTT connect failed: %s", mosquitto_strerror(rc));
      throw std::runtime_error("mosquitto_connect failed");
    }
    mosquitto_loop_start(client_);
    RCLCPP_INFO(get_logger(), "Connected to MQTT %s:%d as %s", broker_host_.c_str(), broker_port_, username_.c_str());

    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate));
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&MqttPublisherNode::onTimer, this));
  }

  ~MqttPublisherNode() override {
    if (client_) {
      mosquitto_loop_stop(client_, true);
      mosquitto_disconnect(client_);
      mosquitto_destroy(client_);
    }
    mosquitto_lib_cleanup();
  }

private:
  void onTimer() {
    seq_++;
    const auto now = std::time(nullptr);

    json j{
      {"robotId", "robot1"},
      {"sensor",  "imu"},
      {"ts", static_cast<long>(now)},
      {"value", {
        {"orientation", {0.01, 0.02, 0.03}},
        {"angularVel",  {0.1, 0.0, -0.1}},
        {"linearAcc",   {0.0, 9.8, 0.0}}
      }},
      {"seq", seq_}
    };

    auto payload = j.dump();
    int rc = mosquitto_publish(client_, nullptr, topic_.c_str(),
                               static_cast<int>(payload.size()),
                               payload.data(),
                               /*qos*/1, /*retain*/false);
    if (rc != MOSQ_ERR_SUCCESS) {
      RCLCPP_WARN(get_logger(), "Publish failed: %s", mosquitto_strerror(rc));
    } else {
      RCLCPP_INFO(get_logger(), "Published [%s] #%d", topic_.c_str(), seq_);
    }
  }

  std::string broker_host_, username_, password_, topic_;
  int broker_port_{1883};
  int seq_{0};

  mosquitto* client_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MqttPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
