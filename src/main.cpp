#include "ros2-hoverboard-driver/hoverboard.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hoverboard>());
  rclcpp::shutdown();
  return 0;
}
