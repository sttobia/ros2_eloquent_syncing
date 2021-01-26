#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

class SyncTalkerNode : public rclcpp::Node {
 public:
  SyncTalkerNode() : Node("sync_talker") {
    rclcpp::QoS qos(10);

    publisher_temp1_ =
        this->create_publisher<sensor_msgs::msg::Temperature>("temp_1", qos);
    publisher_temp2_ =
        this->create_publisher<sensor_msgs::msg::Temperature>("temp_2", qos);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&SyncTalkerNode::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    rclcpp::Time now = this->get_clock()->now();

    auto msg_tmp1 = sensor_msgs::msg::Temperature();
    msg_tmp1.header.stamp = now;
    msg_tmp1.header.frame_id = "test";
    msg_tmp1.temperature = 1.0;

    auto msg_tmp2 = sensor_msgs::msg::Temperature();
    msg_tmp2.header.stamp = now;
    msg_tmp2.header.frame_id = "test";
    msg_tmp2.temperature = 2.0;

    publisher_temp1_->publish(msg_tmp1);
    publisher_temp2_->publish(msg_tmp2);

    RCLCPP_INFO(this->get_logger(), "Published two temperatures.");
  }
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_temp1_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_temp2_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncTalkerNode>());
  rclcpp::shutdown();
  return 0;
}