//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>

//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"



namespace
{



}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto sick_nav350_publisher = std::make_shared<sick_nav350>();
  signal(SIGINT,ExitHandler);
  int ret = sick_nav350_publisher->work_loop();

  rclcpp::shutdown();
  return ret;
  //return 0;
}