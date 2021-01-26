#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

class SyncListenerNode : public rclcpp::Node {
  // typedef message_filters::sync_policies::ExactTime<
  //     sensor_msgs::msg::Temperature, sensor_msgs::msg::Temperature>
  //     ExactPolicy;

  // typedef message_filters::sync_policies::ApproximateTime<
  //     sensor_msgs::msg::Temperature, sensor_msgs::msg::Temperature>
  //     ApproxPolicy;

 public:
  SyncListenerNode() : Node("sync_listener") {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    // subscriber_temp1_.subscribe(this, "temp_1", rmw_qos_profile);
    // subscriber_temp2_.subscribe(this, "temp_2", rmw_qos_profile);

    // message_filters::Synchronizer<ApproxPolicy> approx_sync(
    //     ApproxPolicy(10), subscriber_temp1_, subscriber_temp2_);
    // approx_sync.registerCallback(&SyncListenerNode::TempSyncCallback, this);

    // message_filters::Synchronizer<ExactPolicy> exact_sync(
    //     ExactPolicy(10), subscriber_temp1_, subscriber_temp2_);
    // exact_sync.registerCallback(std::bind(&SyncListenerNode::TempSyncCallback,
    // this,
    //                                       std::placeholders::_1,
    //                                       std::placeholders::_2));

    // // Uncomment this to verify that the messages indeed reach the
    // subscriber_temp1_.registerCallback(std::bind(
    //     &SyncListenerNode::Tmp1Callback, this, std::placeholders::_1));
    // subscriber_temp2_.registerCallback(std::bind(
    //     &SyncListenerNode::Tmp2Callback, this, std::placeholders::_1));

    // message_filters::Subscriber<sensor_msgs::msg::Temperature> sub_1(
    //     this, "temp_1", rmw_qos_profile);
    // message_filters::Subscriber<sensor_msgs::msg::Temperature> sub_2(
    //     this, "temp_2", rmw_qos_profile);

    message_filters::TimeSynchronizer<sensor_msgs::msg::Temperature,
                                      sensor_msgs::msg::Temperature>
        temp_sync(subscriber_temp1_, subscriber_temp2_, 10);
    temp_sync.registerCallback(&SyncListenerNode::TempSyncCallback, this);
  }

 private:
  // For veryfing the single subscriber instances: Uncomment line 26-28.
  void Tmp1Callback(const sensor_msgs::msg::Temperature::ConstSharedPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "Frame '%s', temp %f with ts %u.%u sec ",
                msg->header.frame_id.c_str(), msg->temperature,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  // For veryfing the single subscriber instances: Uncomment line 29-31.
  void Tmp2Callback(const sensor_msgs::msg::Temperature::ConstSharedPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "Frame '%s', temp %f with ts %u.%u sec ",
                msg->header.frame_id.c_str(), msg->temperature,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
  }

  // This callback is never being called.
  void TempSyncCallback(
      const sensor_msgs::msg::Temperature::ConstSharedPtr& msg_1,
      const sensor_msgs::msg::Temperature::ConstSharedPtr& msg_2) {
    RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                msg_1->header.stamp.sec, msg_2->header.stamp.sec);
  }

  message_filters::Subscriber<sensor_msgs::msg::Temperature> subscriber_temp1_;
  message_filters::Subscriber<sensor_msgs::msg::Temperature> subscriber_temp2_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncListenerNode>());
  rclcpp::shutdown();
  return 0;
}