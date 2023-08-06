#include <roborts_utils/base_blackboard.h>
#include <roborts_utils/base_class.h>

#include "navigation.h"
#include "roborts_utils/base_blackboard.h"

void on_exit(int sig) {
  TDT_WARNING("Exit by Ctrl+C");
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  signal(SIGINT, on_exit);
  rclcpp::init(argc, argv);
  LoadParam::InitParam("navigation", "config/navigation.jsonc");
  std::string map_name;
  LoadParam::ReadParam("navigation", "map_name", map_name);
  LoadParam::InitParam(
      "navigation_map",
      "install/roborts_navigation/share/roborts_navigation/map_info/" +
          map_name + ".jsonc");
  std::string command_map_name;
  LoadParam::ReadParam("navigation", "command_map_name", command_map_name);
  LoadParam::InitParam(
      "navigation_command_map",
      "install/roborts_navigation/share/roborts_navigation/command_map/info/" +
          command_map_name + ".jsonc");
  tdttoolkit::BaseBlackboard::Init("match_info", sizeof(tdttoolkit::MatchInfo));
  tdttoolkit::BaseBlackboard::Init("nav_status", sizeof(tdttoolkit::NavStatus));
  auto ros_node = std::make_shared<rclcpp::Node>("navigation");
  tdttoolkit::Time::Init(ros_node->now());
  Navigation navigation(ros_node);
  navigation.start();
  rclcpp::shutdown();
  return 0;
}