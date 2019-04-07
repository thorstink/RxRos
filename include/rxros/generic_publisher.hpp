#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rxros/helpers.hpp"

template <class T>
class Publisher : public rclcpp::Node
{
 public:
  template <typename T1, typename R1>
  Publisher(std::string node_name, std::string topic_name,
            std::chrono::duration<T1, R1> t,
            rmw_qos_profile_t qos = rmw_qos_profile_default)
      : Node(node_name), counter(0)
  {
    // Node->use_intra_process_comms_ = true;
    publisher_ = create_publisher<T>(topic_name, qos);
    timer_ = create_wall_timer(t, [this]() { this->on_timer(); });
  }

 private:
  void on_timer()
  {
    auto message = T();
    message.data = counter++;
    publisher_->publish(message);
  }
  int counter;
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
