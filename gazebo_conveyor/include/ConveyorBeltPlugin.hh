/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_
#define _GAZEBO_CONVEYOR_BELT_PLUGIN_HH_

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Angle.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin for simulating a conveyor belt.
  /// The plugin accepts the following SDF parameters:

  /// <power>: Sets the initial power of the belt as a percentage [0-100].
  /// <joint>: Joint name used to control the belt.
  /// <belt>: Belt's link name.
  ///
  /// Here's an example of a valid SDF conveyor belt:
  /// <model name="conveyor_belt">
  ///
  ///   <model name="conveyor_belt_1">
  ///     <static>true</static>
  ///     <pose>1.21 -2 0.8126 0 0 -1.57079</pose>
  ///     <link name="belt">
  ///       <pose>-5 0 0 0 0 0</pose>
  ///     </link>
  ///   </model>
  ///
  ///   <model name="conveyor_belt_2">
  ///     <static>false</static>
  ///     <pose>1.21 -2 0.8126 0 0 -1.57079</pose>
  ///     <link name="belt">
  ///       <pose>-5 0 0 0 0 0</pose>
  ///       <inertial>
  ///         <inertia>
  ///           <ixx>3.8185</ixx>
  ///           <ixy>0</ixy>
  ///           <ixz>0</ixz>
  ///           <iyy>1781.5</iyy>
  ///           <iyz>0</iyz>
  ///           <izz>1784.72</izz>
  ///         </inertia>
  ///         <mass>100</mass>
  ///       </inertial>
  ///       <!--Uncomment for debugging -->
  ///       <!--
  ///       <visual name="belt_visual">
  ///         <geometry>
  ///           <box>
  ///             <size>14.62206 0.65461 0.18862</size>
  ///           </box>
  ///         </geometry>
  ///       </visual>
  ///       -->
  ///       <collision name="belt_collision">
  ///         <geometry>
  ///           <box>
  ///             <size>14.62206 0.65461 0.18862</size>
  ///           </box>
  ///         </geometry>
  ///         <surface>
  ///           <friction>
  ///             <ode>
  ///               <mu>1.0</mu>
  ///               <mu2>1.0</mu2>
  ///             </ode>
  ///             <torsional>
  ///               <coefficient>1000.0</coefficient>
  ///               <patch_radius>0.1</patch_radius>
  ///             </torsional>
  ///           </friction>
  ///         </surface>
  ///       </collision>
  ///     </link>
  ///   </model>
  ///
  ///   <joint name="belt_joint" type="prismatic">
  ///     <parent>conveyor_belt_1::belt</parent>
  ///     <child>conveyor_belt_2::belt</child>
  ///     <axis>
  ///       <xyz>1 0 0</xyz>
  ///       <limit>
  ///         <lower>0</lower>
  ///         <upper>1.0</upper>
  ///       </limit>
  ///     </axis>
  ///   </joint>
  ///
  ///   <plugin name="conveyor_belt_plugin" filename="libROSConveyorBeltPlugin.so">
  ///     <robot_namespace>/ariac</robot_namespace>
  ///     <link>conveyor_belt::conveyor_belt_2::belt</link>
  ///     <power>0</power>
  ///   </plugin>
  /// </model>
  class GAZEBO_VISIBLE ConveyorBeltPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ConveyorBeltPlugin() = default;

    /// \brief Destructor.
    public: virtual ~ConveyorBeltPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate();

    /// \brief Get if the belt is enabled.
    /// \return True if enabled.
    protected: bool IsEnabled() const;

    /// \brief Get the power of the conveyor belt.
    /// \return Power of the belt as a percentage (0-100).
    protected: double Power() const;

    /// \brief Set the power of the conveyor belt.
    /// \param[in] _power Power of the belt as a percentage (0-100).
    protected: void SetPower(const double _power);

    /// \brief Overwrite this method for sending periodic updates with the
    /// conveyor state.
    private: virtual void Publish() const;

    /// \brief Call back for enable/disable messaged.
    protected: void OnEnabled(ConstGzStringPtr &_msg);

    /// \brief Belt velocity (m/s).
    protected: double beltVelocity = 0.0;

    /// \brief Belt power expressed as a percentage of the internal maximum
    /// speed.
    protected: double beltPower = 0.0;

    /// \brief If true, power commands are processed, otherwise the belt won't move.
    protected: bool enabled = true;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief The joint that controls the movement of the belt.
    private: physics::JointPtr joint;

    /// \brief The belt's link.
    private: physics::LinkPtr link;

    /// \brief When the joint reaches this point, it will go back to its initial
    /// position.
    private: ignition::math::Angle limit;

    /// \brief Maximum linear velocity of the belt.
    protected: double kMaxBeltLinVel = 0.2;

    /// \brief Gazebo node for communication.
    protected: transport::NodePtr gzNode;

    /// \brief Gazebo publisher for modifying the rate of populating the belt.
    public: transport::PublisherPtr populationRateModifierPub;

    /// \brief Gazebo subscriber for modifying the enabled state of the belt.
    public: transport::SubscriberPtr enabledSub;
  };
}
#endif