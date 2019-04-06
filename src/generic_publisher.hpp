#pragma once
#include "helpers.hpp"
#include "rclcpp/rclcpp.hpp"

template <class T>
class Publisher : public rclcpp::Node
{
 public:
  Publisher(std::string nodeName, std::string topicName,
            unsigned int samplingFrequency,
            rmw_qos_profile_t qos = rmw_qos_profile_default)
      : Node(nodeName), counter(0)
  {
    publisher_ = create_publisher<T>(topicName, qos);
    timer_ =
        create_wall_timer(std::chrono::milliseconds(1000 / samplingFrequency),
                          [this]() { this->on_timer(); });
  }

 private:
  void on_timer()
  {
    print_pid("message created and send");
    auto message = T();
    message.data = counter++;
    std::cout << "s: " << counter << std::endl;
    publisher_->publish(message);
  }
  int counter;
  typename rclcpp::Publisher<T>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
