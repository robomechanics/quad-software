
#include <ros/ros.h>

#include <iostream>

#include "apply_force/apply_force.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "apply_force_node");
  ros::NodeHandle nh;

  ApplyForce apply_force(nh, argc, argv);
  apply_force.spin();

  return 0;
}
