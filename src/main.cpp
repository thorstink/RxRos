#include "generic_publisher.hpp"
#include "rxros.hpp"

#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
using namespace rxros;
using executor = rclcpp::executors::SingleThreadedExecutor;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  executor exec;

  print_pid("main thread");

  auto sub = std::make_shared<RxRosSubscriber<std_msgs::msg::Int64>>("sub");
  auto pub =
      std::make_shared<Publisher<std_msgs::msg::Int64>>("pub", "/test", 200);
  auto s = from_event<std_msgs::msg::Int64>(sub, "/test");

  s  //
      .map([](auto yes) {
        // yes->data = 1;  // error! yes is shared_ptr<T const> !
        return -yes->data;
      })
      .tap([](int64_t data) {
        print_pid("subscription 1");
        std::cout << "r: " << data << std::endl;
      })
      .subscribe();

  s  //
      .map([](auto yes) { return yes->data; })
      .tap([](int64_t data) {
        print_pid("subscription 2");
        std::cout << "r: " << data << std::endl;
      })
      .subscribe();

  exec.add_node(sub);
  exec.add_node(pub);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}