#!/usr/bin/env python

# <!-- Students: Samuel Stephen Ben Shiverdaker Zack Boyle -->
# <!-- Lab: FINAL PROJET-->
# <!-- Date: Fall 2023-->
# <!-- Acknowledgements: MoveIt Tutorial Code -->

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#move_group = moveit_commander.MoveGroupCommander(group_name)mmander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##


# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
from tf.transformations import quaternion_from_euler

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
 
    return True

# Helper function to convert an angle to joint goal value
def angleToJoint(angle):
    return (angle/360)*tau

# Helper function to convert degrees to radians
def degToRad(deg):
    return (pi/180)*deg

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
    # Function to set the joint positions manually, utilized for the moves in the demo
    def joint_goal(self, one=0, two=-45, three=0, four=-135, five=0, six=90, seven=45):

        ## Planning to a Joint Goal
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = angleToJoint(one)
        joint_goal[1] = angleToJoint(two)
        joint_goal[2] = angleToJoint(three)
        joint_goal[3] = angleToJoint(four)
        joint_goal[4] = angleToJoint(five)
        joint_goal[5] = angleToJoint(six)
        joint_goal[6] = angleToJoint(seven)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    # Control just the end effector angle on: joint 6.
    def setEefAngle(self, angle=90):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[5] = angleToJoint(angle)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIALEND_SUB_TUTORIAL
        
    # Clear all the objects from the world
    def removeEverythingFromTheWorld(self, timeout=4):
        print("Removing all elements from the world...")
        objects = ['table1', 'table2', 'eefBase', 'eefWiper', 'roller1']
        for obj in objects:
            self.scene.remove_world_object(obj)
        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    
    def buildSolarPanel(self, timeout=4):
        # ==============================
        # Create objects in the world
        # ==============================
        
        print("Building terrain...")
        box1_pose = geometry_msgs.msg.PoseStamped()
        box1_pose.header.frame_id = 'panda_link0'
        box1_pose.pose.position.x = 0.55
        box1_pose.pose.position.y = 0.0
        box1_pose.pose.position.z = 0.5
      
        q = quaternion_from_euler(0, degToRad(70), 0)
        box1_pose.pose.orientation.x = q[0]
        box1_pose.pose.orientation.y = q[1]
        box1_pose.pose.orientation.z = q[2]
        box1_pose.pose.orientation.w = q[3]
        
        self.scene.add_box('table1', box1_pose, size=(0.03, 0.4, 0.4))

        box2_pose = geometry_msgs.msg.PoseStamped()
        box2_pose.header.frame_id = 'panda_link0'
        box2_pose.pose.position.x = 0.5
        box2_pose.pose.position.y = 0.0
        box2_pose.pose.position.z = 0.18

        self.scene.add_box('table2', box2_pose, size=(0.1, 0.1, 0.4))

        box3_pose = geometry_msgs.msg.PoseStamped()
        box3_pose.header.frame_id = 'panda_hand'
        box3_pose.pose.position.x = 0.0
        box3_pose.pose.position.y = 0.0
        box3_pose.pose.position.z = 0.1

        self.scene.add_box('eefBase', box3_pose, size=(0.07, 0.06, 0.05))        

        box4_pose = geometry_msgs.msg.PoseStamped()
        box4_pose.header.frame_id = 'panda_hand'
        box4_pose.pose.position.x = 0.0
        box4_pose.pose.position.y = 0.0
        box4_pose.pose.position.z = 0.12

        self.scene.add_box('eefWiper', box4_pose, size=(0.1, 0.3, 0.01))

        roller1_pose = geometry_msgs.msg.PoseStamped()
        roller1_pose.header.frame_id = 'panda_hand'
        roller1_pose.pose.position.x = 0.0
        roller1_pose.pose.position.y = 0.0
        roller1_pose.pose.position.z = 0.135     

        self.scene.add_box('roller1', roller1_pose, size=(0.03, 0.3, 0.02))
    
    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        self.scene.attach_box(eef_link, 'eefBase', touch_links=touch_links)
        self.scene.attach_box(eef_link, 'eefWiper', touch_links=touch_links)
        self.scene.attach_box(eef_link, 'roller1', touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = 'obj'
        
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )


def main():
    try:
        viz = MoveGroupPythonInterfaceTutorial()
        print("Running sim!")

        #Clear world
        viz.removeEverythingFromTheWorld()
        
        #Put back objects
        viz.buildSolarPanel()

        viz.attach_box()

        # Go to start position and wait for further instruction    
        viz.joint_goal()
        rospy.sleep(1)

        # Move arm all the way up
        viz.joint_goal(0, -45, 0, -121, -5, 90, 45)
        
        # Move to the top of the solar panel
        viz.joint_goal(0, -2, 0, -95, 0, 132, 45)

        # Make sure we stay as close to panel as possible
        viz.joint_goal(0, -22, 0, -120, 0, 135, 45)
        viz.joint_goal(0, -34, 0, -129, 0, 120, 45)

        # Finish move at the bottom of the panel
        viz.joint_goal(0, -51, 0, -144, 0, 112, 45)

        # Reset position
        viz.joint_goal()

        print("DONE")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:TUTORIAL

if __name__ == "__main__":
    main()
