#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mosquitto.h>
#include <string>
#include <mutex>
#include <atomic>

namespace mqtt_publisher_cpp {

class Publisher {
public:
  struct Config {
    std::string host = "127.0.0.1";
    int         port = 1883;
    std::string username;
    std::string password;
    std::string topic_base = "site";
    std::string robot_id   = "robot1";
    int qos = 0;
    bool retain = false;
    int keepalive = 60;
    bool clean_session = true;
    std::string client_id = "ros2-mqtt-pub";
  };

  Publisher(rclcpp::Node* node, const Config& cfg);
  ~Publisher();

  bool ok() const { return connected_; }
  bool send(const std::string& sensor, const std::string& payload);
  const std::string& robot_id() const { return cfg_.robot_id; }

private:
  void connect_locked();
  static void on_connect(struct mosquitto*, void*, int);
  static void on_disconnect(struct mosquitto*, void*, int);
  static void on_log(struct mosquitto*, void*, int, const char*);

  rclcpp::Node* node_;
  Config cfg_;
  struct mosquitto* m_{nullptr};
  std::mutex mtx_;
  std::atomic<bool> connected_{false};
  std::string topic_prefix_;
};

} // namespace mqtt_publisher_cpp
