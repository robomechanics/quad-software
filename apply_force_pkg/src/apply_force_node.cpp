#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "apply_force_node");
  ros::NodeHandle nh;

  double force_x, force_y, force_z;
  double torque_x, torque_y, torque_z;

  // Default values if not specified in YAML file
  nh.param<double>("force_x", force_x, 10.0);
  nh.param<double>("force_y", force_y, 0.0);
  nh.param<double>("force_z", force_z, 0.0);
  nh.param<double>("torque_x", torque_x, 0.0);
  nh.param<double>("torque_y", torque_y, 0.0);
  nh.param<double>("torque_z", torque_z, 0.0);

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>(
      "/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench srv;

  // Apply Force to Link of Choice
  srv.request.body_name = "robot_1::body";  // Body is the default
  srv.request.reference_frame = "";
  srv.request.reference_point.x = 0;
  srv.request.reference_point.y = 0;
  srv.request.reference_point.z = 0;

  srv.request.wrench.force.x = force_x;
  srv.request.wrench.force.y = force_y;
  srv.request.wrench.force.z = force_z;
  srv.request.wrench.torque.x = torque_x;
  srv.request.wrench.torque.y = torque_y;
  srv.request.wrench.torque.z = torque_z;

  srv.request.start_time = ros::Time::now();
  srv.request.duration = ros::Duration(0.5);  // Apply for 1 second

  while (ros::ok()) {
    if (client.call(srv)) {
      ROS_INFO(
          "Force applied to the robot body: Force(%.2f, %.2f, %.2f) "
          "Torque(%.2f, %.2f, %.2f)",
          force_x, force_y, force_z, torque_x, torque_y, torque_z);
    } else {
      ROS_ERROR("Failed to call service /gazebo/apply_body_wrench");
    }
  }

  return 0;
}
