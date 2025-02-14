#include "apply_force/apply_force.h"

#include <iostream>

ApplyForce::ApplyForce(ros::NodeHandle nh, int argc, char** argv) {
  nh_ = nh;
  argc_ = argc;
  argv_ = argv;

  std::string robot_name, robot_state_topic;
  quad_utils::loadROSParamDefault(nh_, "robot_type", robot_name,
                                  std::string("spirit"));
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic);
  quad_utils::loadROSParam(nh_, "/apply_force/duration/update_rate",
                           update_rate_);
  // Default values if not specified in YAML file
  nh.param<double>("apply_force/force/x", force_x_, 10.0);
  nh.param<double>("apply_force/force/y", force_y_, 0.0);
  nh.param<double>("apply_force/force/z", force_z_, 0.0);
  nh.param<double>("apply_force/torque/x", torque_x_, 0.0);
  nh.param<double>("apply_force/torque/y", torque_y_, 0.0);
  nh.param<double>("apply_force/torque/z", torque_z_, 0.0);
  nh.param<double>("apply_force/duration/dt", duration_sec_, 0.5);

  robot_state_sub_ =
      nh_.subscribe(robot_state_topic, 1, &ApplyForce::robotStateCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  force_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("force_torque_markers", 100);
  force_client_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>(
      "/gazebo/apply_body_wrench");
}

void ApplyForce::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  last_robot_state_msg_ = *msg;
}

void ApplyForce::updateForce() {
  // Apply Force to Link of Choice
  srv.request.body_name = "robot_1::body";  // Body is the default
  srv.request.reference_frame = "world";
  srv.request.reference_point.x = 0;
  srv.request.reference_point.y = 0;
  srv.request.reference_point.z = 0;

  srv.request.wrench.force.x = force_x_;
  srv.request.wrench.force.y = force_y_;
  srv.request.wrench.force.z = force_z_;
  srv.request.wrench.torque.x = torque_x_;
  srv.request.wrench.torque.y = torque_y_;
  srv.request.wrench.torque.z = torque_z_;

  srv.request.start_time = ros::Time::now();
  srv.request.duration = ros::Duration(duration_sec_);  // Apply for 1 second

  // Create a marker for visualization
  //   visualization_msgs::Marker marker;
  last_robot_marker_msg_.header.frame_id = "robot_1_ground_truth/body";
  last_robot_marker_msg_.header.stamp = ros::Time::now();
  last_robot_marker_msg_.ns = "apply_force";
  last_robot_marker_msg_.id = 0;
  last_robot_marker_msg_.type = visualization_msgs::Marker::ARROW;
  last_robot_marker_msg_.action = visualization_msgs::Marker::ADD;
  last_robot_marker_msg_.pose.position.x = 0.25;
  last_robot_marker_msg_.pose.position.y = -0.4;
  last_robot_marker_msg_.pose.position.z = 0;
  last_robot_marker_msg_.pose.orientation.w = 1.0;

  // Set the scale of the arrow (length corresponds to the force magnitude)
  force_magnitude_ =
      sqrt(force_x_ * force_x_ + force_y_ * force_y_ + force_z_ * force_z_);
  last_robot_marker_msg_.scale.x =
      0.2;  // force_magnitude_*0.1;  // Arrow length
  last_robot_marker_msg_.scale.y = 0.02;  // Arrow width
  last_robot_marker_msg_.scale.z = 0.02;  // Arrow height

  // Set the color of the arrow (e.g., red)
  last_robot_marker_msg_.color.r = 1.0;
  last_robot_marker_msg_.color.g = 0.0;
  last_robot_marker_msg_.color.b = 0.0;
  last_robot_marker_msg_.color.a = 1.0;
  last_robot_marker_msg_.lifetime = ros::Duration(0.1);

  if (force_magnitude_ != 0) {  // Avoid division by zero
    tf2::Vector3 force_vector(force_x_, force_y_, force_z_);
    force_vector.normalize();  // Normalize the force vector

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(
        0, 0,
        atan2(force_y_,
              force_x_));  // Set the orientation based on force direction
    orientation_quat.normalize();  // Normalize quaternion

    // Convert tf2 quaternion to geometry_msgs Quaternion
    last_robot_marker_msg_.pose.orientation = tf2::toMsg(orientation_quat);
  } else {
    // Default orientation if force magnitude is zero
    last_robot_marker_msg_.pose.orientation.x = 0.0;
    last_robot_marker_msg_.pose.orientation.y = 0.0;
    last_robot_marker_msg_.pose.orientation.z = 0.0;
    last_robot_marker_msg_.pose.orientation.w = 1.0;
  }
}

void ApplyForce::publishForce() {
  if (force_client_.call(srv)) {
    force_marker_pub_.publish(last_robot_marker_msg_);
    // Log the applied force and set point positions
    ROS_INFO(
        "Force applied to the robot body: Force(%.2f, %.2f, %.2f) "
        "Torque(%.2f, %.2f, %.2f) at Set Point: (%.2f, %.2f, %.2f)",
        force_x_, force_y_, force_z_, torque_x_, torque_y_, torque_z_,
        set_point(0), set_point(1), set_point(2));
  }
}

double ApplyForce::euclideanDistance(const Eigen::Vector3d& point1,
                                     const Eigen::Vector3d& point2) {
  return (point2 - point1).norm();
}

bool ApplyForce::checkDistance() {
  robot_loc = Eigen::Vector3d::Zero();
  robot_loc << last_robot_state_msg_.body.pose.position.x,
      last_robot_state_msg_.body.pose.position.y,
      last_robot_state_msg_.body.pose.position.z;
  double dist = euclideanDistance(robot_loc, set_point);
  if (dist >= 1.0) {
    set_point(0) += 1;
    return true;
  } else {
    return false;
  }
}

void ApplyForce::spin() {
  // Initialize timing params
  ros::Rate r(update_rate_);

  while (ros::ok()) {
    // Collect new messages on subscriber topics and publish heartbeat
    ros::spinOnce();
    if (checkDistance()) {
      // Get the newest force infomation
      updateForce();
      // Publish the Force and Visualization to the Appropriate Topics
      publishForce();
    }
    // Enforce update rate
    r.sleep();
  }
}
