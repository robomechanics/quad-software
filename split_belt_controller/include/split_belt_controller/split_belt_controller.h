#ifndef SPLIT_BELT_CONTROLLER_H
#define SPLIT_BELT_CONTROLLER_H

#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

class SplitBeltController {
 public:
  SplitBeltController(ros::NodeHandle nh, int argc, char** argv);
  void spin();
  double left_belt_velocity_;
  double right_belt_velocity_;

 private:
  ros::NodeHandle nh_;
  int argc_;
  char** argv_;
  double update_rate_;
  ros::Publisher left_belt_pub_;
  ros::Publisher right_belt_pub_;
};

#endif  // SPLIT_BELT_CONTROLLER_H