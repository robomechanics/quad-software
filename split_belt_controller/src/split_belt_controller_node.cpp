#include <ros/ros.h>

#include <iostream>

#include "split_belt_controller/split_belt_controller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "split_belt_controller_node");
  ros::NodeHandle nh;

  SplitBeltController split_belt_controller(nh, argc, argv);
  split_belt_controller.spin();
  return 0;
}