#include "split_belt_controller/split_belt_controller.h"

#include <iostream>
#include <std_msgs/Float64.h>


SplitBeltController::SplitBeltController(ros::NodeHandle nh, int argc, char** argv) {
  nh_ = nh;
  argc_ = argc;
  argv_ = argv;
  // Initialize the controller
  std::string robot_name, robot_state_topic;
  quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name,
                                  std::string("spirit"));
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "/split_belt_controller/update_rate",
                           update_rate_);

  nh.param<double>("split_belt_controller/left_belt_velocity",
                   left_belt_velocity_, 0.0);
  nh.param<double>("split_belt_controller/right_belt_velocity",
                   right_belt_velocity_, 0.0);

  ros::Publisher left_belt_pub_ =
      nh.advertise<std_msgs::Float64>("left_belt_velocity", left_belt_velocity_);
  ros::Publisher right_belt_pub_ =
      nh.advertise<std_msgs::Float64>("right_belt_velocity", right_belt_velocity_);
}

void SplitBeltController::spin() {
  // Run the controller
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    ros::spinOnce();
    std_msgs::Float64 left_belt_msg;
    std_msgs::Float64 right_belt_msg;
    left_belt_msg.data = left_belt_velocity_;
    right_belt_msg.data = right_belt_velocity_;

    // Publish the velocity commands
    left_belt_pub_.publish(left_belt_msg);
    right_belt_pub_.publish(right_belt_msg);
    r.sleep();
  }
}