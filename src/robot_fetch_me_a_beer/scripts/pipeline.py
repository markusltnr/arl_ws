#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from manipulate import Gripper, DmpRos
from detect import ObjectDetector
from navigate import GoalPublisher

dmp_folder = '/home/user/exchange/arl_ws/src/robot_fetch_me_a_beer/'

def grasp_object(dmp_ros : DmpRos, gripper : Gripper, can_position):
    print("Grasping Object")
    action = 'grasp'
    dmp_parameter_path = dmp_folder + str(action) + '_dmp.npz'
    rospy.set_param('motion', action)

    # Load the DMP parameters from the specified file path
    dmp_ros.load_dmp_parameters(dmp_parameter_path)

    print("Opening gripper")
    gripper.open()

    # Execute the DMP to perform the desired motion
    dmp_ros.run_dmp(can_position)

    print("Closing gripper")
    gripper.close()

    print("Grasping finished")

def retrieve_object(dmp_ros : DmpRos, gripper : Gripper, target_position):
    print("Retrieving Object")
    action = 'retrieve'
    dmp_parameter_path = dmp_folder + str(action) + '_dmp.npz'
    rospy.set_param('motion', action)

    # Load the DMP parameters from the specified file path
    dmp_ros.load_dmp_parameters(dmp_parameter_path)

    # Execute the DMP to perform the desired motion
    dmp_ros.run_dmp(target_position)

    print("Retrieving finished")

def place_object(dmp_ros : DmpRos, gripper : Gripper, target_position):
    "Placing Object"
    action = 'place'
    dmp_parameter_path = dmp_folder + str(action) + '_dmp.npz'
    rospy.set_param('motion', action)

    # Load the DMP parameters from the specified file path
    dmp_ros.load_dmp_parameters(dmp_parameter_path)

    # Execute the DMP to perform the desired motion
    dmp_ros.run_dmp(target_position)

    print("Opening gripper")
    gripper.open()

    print("Placing finished")

def move_to_table(goalPublisher : GoalPublisher, goal_pose : MoveBaseGoal) : 
    result = goalPublisher.publish_goal(goal_pose)
    return result

def move_head(objectDetector : ObjectDetector):
    """
        move_head function use a while loop to move head joints until object is found by TIAGo
    """

    # publsih to head controller the join trajectory 
    pub_head_controller = rospy.Publisher(
        '/head_controller/command', JointTrajectory, queue_size=1)
    head_2_movement = 0

    print("Moving head")
    # loop function until object was found
    while objectDetector.can_position is None:
        # trajectory_msgs --> JointTrajectory
        trajectory = JointTrajectory()
        # Define joint in use in order to move head 
        trajectory.joint_names = ['head_1_joint', 'head_2_joint']

        # ***The action can be used to give a trajectory to the head expressed in several waypoints***. 
        trajectory_points = JointTrajectoryPoint()
        # change coordinate just along z axis
        trajectory_points.positions = [0.0, head_2_movement]
        # Define time action 
        trajectory_points.time_from_start = rospy.Duration(1.0)

        trajectory.points.append(trajectory_points)
        
        pub_head_controller.publish(trajectory)
        # interval to start next movement
        rospy.sleep(0.8)
        # Define head movment in a lowering cycle with -0.1 step to a max -1, until object is detected
        head_2_movement = max(-1, head_2_movement-0.1)

def generate_goal(x, y, z, q_x, q_y, q_z, q_w):
    # generate a MoveBaseGoal that can be published from pose
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position = Point(x=x, y=y, z=z)
    
	goal.target_pose.pose.orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
	return goal

if __name__ == '__main__':
    # TODO: call rospy init here instead of in the separate files

    # TODO: execute ObjectDetector seperately? -> Sergej
    # one node, which is always running and makes object detector service available
    # subscribe to this topic, which starts the object detection (topic or better service)
    # call service to get the detection output
    # stop calling the service once the can is grasped

    # Initialize ObjectDetector
    # TODO: synchronize depth and rgb images in object detector class
    objectDetector = ObjectDetector()

    # Initialize GoalPublisher
    goalPublisher = GoalPublisher()

    # TODO: take can position from marker? currently it's being returned from the ObjectDetector class
    # marker subscriber
    # marker_sub = rospy.Publisher("/visualization_marker", Marker, callback_image)

    # Initialize the gripper for the right arm.
    gripper = Gripper("right")

    # Initialize the DMP (Dynamic Motion Primitive) handling in ROS.
    dmp_ros = DmpRos()

    print ("Initialized objectDetector, gripper and dmpRos")

    print("Moving to table")
    # goal position is from .yaml file
    goal_pose = generate_goal(x=2.0, y=-2.2, z=0.0, q_x=0.0, q_y=0.0, q_z=-0.7071067811865476, q_w=0.7071067811865476)

    # for now we use fixed position, later if there is time we can search for the table
    move_to_table(goalPublisher, goal_pose)

    # TODO: not yet working, poses instead of trajectory has to be published -> Markus
    # "/whole_body_kinematic_controller/gaze_objective_xtion_optical_frame_goal" topic
    # When you publish PoseStamped messages to this topic the robot will move its head, so it looks at the point you have sent
    move_head(objectDetector)

    # get can position
    # TODO: get from published topic from object detector (or marker)
    can_position = objectDetector.can_position
    
    grasp_object(dmp_ros, gripper, can_position=can_position)

    # TODO: stop yolo 

    retrieve_object(dmp_ros, gripper, target_position=np.array([0.27094205, -0.4120216, 1.05597785]))

    # goal position is from .yaml file
    goal_pose = generate_goal(x=0.5, y=1.4, z=0.0, q_x=0.0, q_y=0.0, q_z=0.7071067811865476, q_w=0.7071067811865476)
    move_to_table(goalPublisher, goal_pose)

    # goal position should be in robot base frame (correct?)
    place_object(dmp_ros, gripper, target_position=np.array([0.55, -0.2, 0.63]))

    # TODO: Report 
    # Anish will start with the basic sections

    # TODO: Provide shell scripts for seperate actions -> Moritz


    # Extension:
    # Implement on a real TIAGO++ robot
    # Handover beer can to person instead of placing on table
    #   same DMP with different goal pose can be used (?)
    #   Human Detection (YOLO), approximate hand, give goal

    # Detect table? too much work, data set collection etc.
    
