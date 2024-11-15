# Gazebo conveyor belt plugin

## How to use

Clone the repository to your project `src` folder and compile the project.

Add this to the end of your URDF/Xacro

```xml
<gazebo>
    <plugin name="conveyor_belt_plugin" filename="libROSConveyorBeltPlugin.so">
        <robot_namespace>/robot_ns</robot_namespace>
        <link>belt_link</link>
        <update_rate>10</update_rate>
        <max_velocity>0.2</max_velocity>
    </plugin>
</gazebo>
```

The belt is active by default, with power of 0%. 

You can change the belts linear velocity using the `<max_velocity>` parameter.

To set the power value, you need to use rosservice.

`$ rosservice call /robot_ns/conveyor/control "power: 15.0"`

This would set the power to 15%.

---

## Origins

Original source is the Open Source Robotics Foundations repository, called "Agile Robotics for Industrial Automation Competition", [ARIAC](https://bitbucket.org/osrf/ariac/src/master/)

Original package created by:

- [Carlos Agüero](https://github.com/caguero)
- [Deanna Hood](https://github.com/dhood)
- [William Woodall](https://github.com/wjwwood)
- [Shane Loretz](https://github.com/sloretz)

## Edits

I made a few edits to the source, that I left behind as comments. I don't know that much about writing plugins for Gazebo or ROS, but I hope this works for some.

> Hannu Henttinen

---

Diff for `src/ConveyorBeltPlugin.cc`

```diff
diff --git a/ariac/osrf_gear/src/ConveyorBeltPlugin.cc b/gazebo-conveyor/src/ConveyorBeltPlugin.cc
index a722f1f..4494bb9 100644
--- a/ariac/osrf_gear/src/ConveyorBeltPlugin.cc
+++ b/gazebo-conveyor/src/ConveyorBeltPlugin.cc
@@ -116,9 +116,10 @@ void ConveyorBeltPlugin::OnUpdate()
     // found an incorrect value in childLinkPose within
     // Joint::SetPositionMaximal(). This workaround makes sure that the right
     // numbers are always used in our scenario.
-    const ignition::math::Pose3d childLinkPose(1.20997, 2.5998, 0.8126, 0, 0, -1.57);
-    const ignition::math::Pose3d newChildLinkPose(1.20997, 2.98, 0.8126, 0, 0, -1.57);
-    this->link->MoveFrame(childLinkPose, newChildLinkPose);
+    //const ignition::math::Pose3d childLinkPose(1.20997, 2.5998, 0.8126, 0, 0, -1.57);
+    //const ignition::math::Pose3d newChildLinkPose(1.20997, 2.98, 0.8126, 0, 0, -1.57);
+    //this->link->MoveFrame(childLinkPose, newChildLinkPose);
+    this->joint->SetPosition(0,0);
   }
 
   this->Publish();
@@ -186,4 +187,4 @@ void ConveyorBeltPlugin::OnEnabled(ConstGzStringPtr &_msg)
 /////////////////////////////////////////////////
 void ConveyorBeltPlugin::Publish() const
 {
-}
+}
\ No newline at end of file
```

Diff for `src/ROSConveyorBeltPlugin.cc`

```diff
diff --git a/ariac/osrf_gear/src/ROSConveyorBeltPlugin.cc b/gazebo-conveyor/src/ROSConveyorBeltPlugin.cc
index a64db82..5735b19 100644
--- a/ariac/osrf_gear/src/ROSConveyorBeltPlugin.cc
+++ b/gazebo-conveyor/src/ROSConveyorBeltPlugin.cc
@@ -15,7 +15,7 @@
  *
 */
 #include "ROSConveyorBeltPlugin.hh"
-#include "osrf_gear/ConveyorBeltState.h"
+#include "gazebo_conveyor/ConveyorBeltState.h"
 
 #include <cstdlib>
 #include <string>
@@ -72,13 +72,13 @@ void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf
 
   // Message used for publishing the state of the conveyor.
   this->statePub = this->rosnode_->advertise<
-    osrf_gear::ConveyorBeltState>(stateTopic, 1000);
+    gazebo_conveyor::ConveyorBeltState>(stateTopic, 1000);
 }
 
 /////////////////////////////////////////////////
 void ROSConveyorBeltPlugin::Publish() const
 {
-  osrf_gear::ConveyorBeltState stateMsg;
+  gazebo_conveyor::ConveyorBeltState stateMsg;
   stateMsg.enabled = this->IsEnabled();
   stateMsg.power = this->Power();
   this->statePub.publish(stateMsg);
@@ -86,10 +86,10 @@ void ROSConveyorBeltPlugin::Publish() const
 
 /////////////////////////////////////////////////
 bool ROSConveyorBeltPlugin::OnControlCommand(ros::ServiceEvent<
-  osrf_gear::ConveyorBeltControl::Request, osrf_gear::ConveyorBeltControl::Response> & event)
+  gazebo_conveyor::ConveyorBeltControl::Request, gazebo_conveyor::ConveyorBeltControl::Response> & event)
 {
-  const osrf_gear::ConveyorBeltControl::Request& req = event.getRequest();
-  osrf_gear::ConveyorBeltControl::Response& res = event.getResponse();
+  const gazebo_conveyor::ConveyorBeltControl::Request& req = event.getRequest();
+  gazebo_conveyor::ConveyorBeltControl::Response& res = event.getResponse();
   gzdbg << "Conveyor control service called with: " << req.power << std::endl;
 
   const std::string& callerName = event.getCallerName();
@@ -122,9 +122,9 @@ bool ROSConveyorBeltPlugin::OnControlCommand(ros::ServiceEvent<
     return true;
   }
 
-  if (!(0 == req.power || (req.power >= 50 && req.power <= 100)))
+  if (!(0 == req.power || req.power <= 100))
   {
-    std::string errStr = "Requested belt power is invalid. Accepted values are 0 or in the range [50, 100].";
+    std::string errStr = "Requested belt power is invalid. Accepted values are in the range [0, 100].";
     gzerr << errStr << std::endl;
     ROS_ERROR_STREAM(errStr);
     res.success = false;
```

Diff for `include/ROSConveyorBeltPlugin.hh`

```diff
diff --git a/ariac/osrf_gear/include/ROSConveyorBeltPlugin.hh b/gazebo-conveyor/include/ROSConveyorBeltPlugin.hh
index 3ade001..0ddf0fa 100644
--- a/ariac/osrf_gear/include/ROSConveyorBeltPlugin.hh
+++ b/gazebo-conveyor/include/ROSConveyorBeltPlugin.hh
@@ -24,7 +24,7 @@
 #include "ConveyorBeltPlugin.hh"
 
 // ROS
-#include <osrf_gear/ConveyorBeltControl.h>
+#include <gazebo_conveyor/ConveyorBeltControl.h>
 #include <ros/ros.h>
 
 namespace gazebo
@@ -47,8 +47,8 @@ namespace gazebo
     /// \param[in] _req The desired state of the conveyor belt.
     /// \param[in] _res If the service succeeded or not.
     public: bool OnControlCommand(ros::ServiceEvent<
-      osrf_gear::ConveyorBeltControl::Request,
-      osrf_gear::ConveyorBeltControl::Response> & event);
+      gazebo_conveyor::ConveyorBeltControl::Request,
+      gazebo_conveyor::ConveyorBeltControl::Response> & event);
 
     // Documentation inherited.
     private: virtual void Publish() const;
```
