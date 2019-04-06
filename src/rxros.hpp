#pragma once
// Reactive Programming library
#include <rxcpp/rx.hpp>
// ROS
#include "rclcpp/rclcpp.hpp"
// Helpers
#include "helpers.hpp"

namespace rxros
{
/**
 * @brief RxRosSubscriber node.
 *
 */
template <class T>
class RxRosSubscriber : public rclcpp::Node
{
 private:
  typename rclcpp::Subscription<T>::ConstSharedPtr subscription_;
  rxcpp::subscriber<std::shared_ptr<T const>> s_;

 public:
  RxRosSubscriber(std::string name)
      : Node(name),
        s_(std::move(rxcpp::make_subscriber<std::shared_ptr<T const>>()))
  {
  }
  ~RxRosSubscriber() { s_.on_completed(); }

  void createSub(const std::string& topic_name,
                 rxcpp::subscriber<std::shared_ptr<T const>>& s)
  {
    print_pid("subscription created");
    s_ = s;
    subscription_ = this->create_subscription<T>(
        topic_name,
        [=](std::shared_ptr<T const> msg) {  //
          if (!s_.is_subscribed()) {  // Stop emitting if nobody is listening
          } else {
            print_pid("callback!");
            // s_.on_next(std::const_pointer_cast<T const>(msg));
            s_.on_next(msg);
          }
        },
        rmw_qos_profile_default);
  }
};

template <class T>
rxcpp::observable<std::shared_ptr<T const>> inline from_event(
    std::shared_ptr<RxRosSubscriber<T>> node, const std::string& topic_name)
{
  print_pid("observable created");
  return rxcpp::observable<>::create<std::shared_ptr<T const>>(
             [=](rxcpp::subscriber<std::shared_ptr<T const>> out) {
               node->createSub(topic_name, out);
             })
      .publish()
      .ref_count()
      .as_dynamic();
}

}  // namespace rxros