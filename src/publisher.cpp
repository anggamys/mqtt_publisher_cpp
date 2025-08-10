#include "mqtt_publisher_cpp/publisher.hpp"
#include <chrono>
#include <thread>

using std::placeholders::_1;

namespace mqtt_publisher_cpp {

Publisher::Publisher(rclcpp::Node* node, const Config& cfg)
: node_(node), cfg_(cfg) {
  static std::once_flag once;
  std::call_once(once, [](){ mosquitto_lib_init(); });

  topic_prefix_ = cfg_.topic_base + "/" + cfg_.robot_id + "/sensors";

  // set 'this' as userdata; mosquitto_new's 3rd arg is user data
  m_ = mosquitto_new(cfg_.client_id.c_str(), cfg_.clean_session, this);
  if (!m_) {
    RCLCPP_ERROR(node_->get_logger(), "mosquitto_new failed");
    return;
  }
  // (opsional, menegaskan userdata) mosquitto_user_data_set(m_, this);

  mosquitto_connect_callback_set(m_, &Publisher::on_connect);
  mosquitto_disconnect_callback_set(m_, &Publisher::on_disconnect);
  mosquitto_log_callback_set(m_, &Publisher::on_log);

  if (!cfg_.username.empty()) {
    mosquitto_username_pw_set(
      m_, cfg_.username.c_str(),
      cfg_.password.empty() ? nullptr : cfg_.password.c_str()
    );
  }

  connect_locked();

  // run network loop in background
  int rc = mosquitto_loop_start(m_);
  if (rc != MOSQ_ERR_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "mosquitto_loop_start failed: %s", mosquitto_strerror(rc));
  }
}

Publisher::~Publisher() {
  if (m_) {
    mosquitto_disconnect(m_);
    mosquitto_loop_stop(m_, true);
    mosquitto_destroy(m_);
  }
  // NOTE: intentionally not calling mosquitto_lib_cleanup() (global).
}

void Publisher::connect_locked() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (!m_) return;

  int rc = mosquitto_connect(m_, cfg_.host.c_str(), cfg_.port, cfg_.keepalive);
  if (rc != MOSQ_ERR_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MQTT connect to %s:%d failed: %s",
                 cfg_.host.c_str(), cfg_.port, mosquitto_strerror(rc));
  } else {
    RCLCPP_INFO(node_->get_logger(), "MQTT connecting to %s:%d ...", cfg_.host.c_str(), cfg_.port);
  }
}

void Publisher::on_connect(struct mosquitto* m [[maybe_unused]], void* userdata, int rc) {
  auto* self = static_cast<Publisher*>(userdata);
  if (rc == 0) {
    self->connected_ = true;
    RCLCPP_INFO(self->node_->get_logger(), "MQTT connected.");
  } else {
    self->connected_ = false;
    RCLCPP_ERROR(self->node_->get_logger(), "MQTT connect error: %d", rc);
  }
}

void Publisher::on_disconnect(struct mosquitto* m [[maybe_unused]], void* userdata, int rc) {
  auto* self = static_cast<Publisher*>(userdata);
  self->connected_ = false;
  RCLCPP_WARN(self->node_->get_logger(), "MQTT disconnected (rc=%d). Reconnecting...", rc);
  // simple backoff
  std::thread([self](){
    std::this_thread::sleep_for(std::chrono::seconds(2));
    self->connect_locked();
  }).detach();
}

void Publisher::on_log(struct mosquitto* m [[maybe_unused]], void* userdata, int level, const char* str) {
  auto* self = static_cast<Publisher*>(userdata);
  RCLCPP_DEBUG(self->node_->get_logger(), "[mosq log %d] %s", level, str ? str : "");
}

bool Publisher::send(const std::string& sensor, const std::string& payload) {
  if (!m_) return false;
  if (!connected_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "MQTT not connected yet");
    return false;
  }
  const std::string topic = topic_prefix_ + "/" + sensor;
  int rc = mosquitto_publish(m_, nullptr, topic.c_str(),
                             static_cast<int>(payload.size()),
                             payload.data(),
                             cfg_.qos, cfg_.retain);
  if (rc != MOSQ_ERR_SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Publish failed to %s: %s", topic.c_str(), mosquitto_strerror(rc));
    return false;
  }
  return true;
}

} // namespace mqtt_publisher_cpp
