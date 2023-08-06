#include "roborts_resolve/roborts_resolve.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto resolve = std::make_shared<Resolve::ArmorResolve>();
  while (rclcpp::ok()) {

    rclcpp::spin_some(resolve);
    
  }
  rclcpp::shutdown();
  return 0;
}