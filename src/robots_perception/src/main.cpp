#include "robots_perception/robots_perception.h"

std::shared_ptr<robots_perception::RobotsPerception> perception_node;

void on_exit(int sig) {
  TDT_WARNING("Exit by Ctrl+C");
  perception_node->close();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, on_exit);


    perception_node = std::make_shared<robots_perception::RobotsPerception>();
    rclcpp::Rate loop_rate(40);

    while (rclcpp::ok()) {
        rclcpp::spin_some(perception_node);
        
        perception_node->run();

        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}