#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <string>

class ApplyForce {
 public:
  /**
   * @brief Constructor for ApplyForce
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type ApplyForce
   */

  ApplyForce(ros::NodeHandle nh, int argc, char** argv);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle current robot state
   * @param[in] msg input message contining current robot state
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Publishes Service Call and Visualization Marker
   */
  void publishForce();
  /**
   * @brief Builds Service Request, and Visualization Marker
   */
  void updateForce();
  /**
   * @brief Calculates euclidean distance relative to a set point to deploy
   * force
   */

  double euclideanDistance(const Eigen::Vector3d& point1,
                           const Eigen::Vector3d& point2);

  bool checkDistance();

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// ros Publisher for Force Visualization Marker
  ros::Publisher force_marker_pub_;

  /// ros Subscriber for last robot state
  ros::Subscriber robot_state_sub_;

  /// ros ServiceClient for Link Wrench Serivce Call
  ros::ServiceClient force_client_;

  /// Most recent state estimate
  quad_msgs::RobotState last_robot_state_msg_;

  /// Most Recent Visualization of Applied Force
  visualization_msgs::Marker last_robot_marker_msg_;

  /// Most Recent Service Call of Applied Force
  gazebo_msgs::ApplyBodyWrench srv;

  Eigen::Vector3d robot_loc;

  Eigen::Vector3d set_point{0.0, 0.0, 0.27};

  double force_x_;

  double force_y_;

  double force_z_;

  double torque_x_;

  double torque_y_;

  double torque_z_;

  double force_magnitude_;

  double duration_sec_;

  bool single = false;

  double update_rate_;

  /// Required for some hardware interfaces
  int argc_;

  /// Required for some hardware interfaces
  char** argv_;
};

#endif  // APPLY_FORCE_H
