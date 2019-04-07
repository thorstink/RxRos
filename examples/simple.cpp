#include "rxros/generic_publisher.hpp"
#include "rxros/rxros.hpp"

#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
using namespace rxros;
using executor = rclcpp::executors::SingleThreadedExecutor;

int main(int argc, char* argv[])
{
  // Normal ROS stuff
  rclcpp::init(argc, argv);
  executor exec;

  print_pid("main thread");

  // First create a normal ROS publisher.. This particular one publishes an
  // Int64 msg every 0.01 seconds on a topic with name "/test"
  auto pub =
      std::make_shared<Publisher<std_msgs::msg::Int64>>("pub", "/test", 10ms);

  // Now creae a special subscriber node which can be used to create an
  // observable for a particular topic. Note that this object does not actually
  // _subscribe_ to a topic upon creation (typical initialization of a
  // ROS-subscriber-node), it just makes a ros-node with a subscribe member
  // variable of type T and no active ROS-subscribtion just yet.
  auto sub = std::make_shared<RxRosSubscriber<std_msgs::msg::Int64>>("sub");

  // Create the actual rxcpp::observable<T> by passing the node (which will have
  // the ros-subscription) and a topic name it should listen to and instantiate
  // the actual subscription to the topic.
  auto s = from_topic<std_msgs::msg::Int64>(sub, "/test");

  // Now use the topic as an ordinairy obervable. This subscription takes
  // ros_msgs while the value is smaller than 5.
  s  //
      .map([](auto ros_msg) {
        // yes->data = 1;  // error! type is shared_ptr<T const> !
        return ros_msg->data;
      })
      .take_while([](int64_t v) { return v < int64_t(5); })
      .tap([](int64_t data) {
        print_pid("subscription 1");
        std::cout << "r: " << data << std::endl;
      })
      .subscribe([](auto) {}, []() { printf("OnCompleted 1\n"); });

  // We can add another "subscriber to the topic" by just subscribing to the
  // observable stream. This stream skipps while the value is smaller than 5 and
  // goes on 'for ever'.
  s  //
      .map([](auto ros_msg) {
        // yes->data = 1;  // error! type is shared_ptr<T const> !
        return ros_msg->data;
      })
      .skip_while([](int64_t data) { return data < int64_t(5); })
      .tap([](int64_t data) {
        print_pid("subscription 2");
        std::cout << "r: " << data << std::endl;
      })
      .subscribe([](auto) {}, []() { printf("OnCompleted 2\n"); });

  // Add the nodes to the ROS-event loop.
  exec.add_node(sub);
  exec.add_node(pub);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}