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
from robot_fetch_me_a_beer.srv import DetectionControl

dmp_folder = '/home/user/exchange/arl_ws/src/robot_fetch_me_a_beer/dmp/'

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
    rospy.sleep(3)

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

def move_head(x = 0.5, y = -0.2, z = 0.8):
    """
        Move the head to a specific position. x, y and z in the robot base frame
    """

    # publsih to head controller the join trajectory 
    pub_head_controller = rospy.Publisher(
        '/whole_body_kinematic_controller/gaze_objective_xtion_optical_frame_goal', PoseStamped, queue_size=1, latch=True)

    print("Moving head")
    # loop function until object was found
   
    # trajectory_msgs --> JointTrajectory
    gaze_pose = PoseStamped()
    # Define joint in use in order to move head 
    gaze_pose.pose.position.x = x
    gaze_pose.pose.position.y = y
    gaze_pose.pose.position.z = z
    gaze_pose.pose.orientation.x = 0.0
    gaze_pose.pose.orientation.y = 0.0
    gaze_pose.pose.orientation.z = 0.0
    gaze_pose.pose.orientation.w = 0.0
    gaze_pose.header.stamp.secs = 0
    gaze_pose.header.stamp.nsecs = 0
    gaze_pose.header.seq = 1
    gaze_pose.header.frame_id = 'base_footprint'

    pub_head_controller.publish(gaze_pose)

def generate_goal(x, y, z, q_x, q_y, q_z, q_w):
    # generate a MoveBaseGoal that can be published from pose
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position = Point(x=x, y=y, z=z)
    
	goal.target_pose.pose.orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
	return goal

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("pipeline")

    # TODO: execute ObjectDetector seperately? -> Sergej
    # one node, which is always running and makes object detector service available
    # subscribe to this topic, which starts the object detection (topic or better service)
    # call service to get the detection output
    # stop calling the service once the can is grasped
    
    # Move the head to a specific position
    move_head()

    # Initialize ObjectDetector
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
    # goal_pose = generate_goal(x=1.1, y=-3.0, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0)

    # for now we use fixed position, later if there is time we can search for the table
    move_to_table(goalPublisher, goal_pose)

    # TODO: not yet working, poses instead of trajectory has to be published -> Markus
    # "/whole_body_kinematic_controller/gaze_objective_xtion_optical_frame_goal" topic
    # When you publish PoseStamped messages to this topic the robot will move its head, so it looks at the point you have sent
    move_head()

    # get can position
    # TODO: get from published topic from object detector (or marker)
    rospy.wait_for_service('control_detection')
    try:
        
        control_detection = rospy.ServiceProxy('control_detection', DetectionControl)
        # Enable detection
        response = control_detection(True)
        rospy.loginfo(response.message)
        # while(objectDetector.can_position is None):
        #     if rospy.is_shutdown():
        #         rospy.logerr("Terminated.")
        #         rospy.sleep(5)
        #         break
        #     rospy.loginfo("Looking for a can...")
        #     rospy.sleep(1)
        rospy.loginfo("Looking for a can...")
        while True:
            try:
                can_position = rospy.wait_for_message(topic="/detected_can_pose", topic_type=PoseStamped, timeout=15)   # maybe some recovery behaviour?
                can_position = np.array([
                    can_position.pose.position.x,
                    can_position.pose.position.y,
                    can_position.pose.position.z
                    ])
                if can_position is not None:
                    break
            except rospy.ROSException as e:
                rospy.logwarn = "No can detected - moving the head to search for it (soon :D)"
                break
                # since we theoretically know the bounds in which we can move the head, we can hard code the search pattern for the head i think
        
        # can_position = objectDetector.can_position
        grasp_object(dmp_ros, gripper, can_position=can_position)
        rospy.sleep(6)

        # object detection still active here, perhaps if we wanna have it dynamically change in the future?
        # otherwise running it once to get the pose and deactivating it right after seems the way to go

        # Disable detection
        response = control_detection(False)
        rospy.loginfo(response.message)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}") 

    # TODO: stop yolo (done in 'Disable detection' above)

    retrieve_object(dmp_ros, gripper, target_position=np.array([0.27094205, -0.4120216, 1.05597785]))

    # goal position is from .yaml file
    goal_pose = generate_goal(x=0.5, y=1.4, z=0.0, q_x=0.0, q_y=0.0, q_z=0.7071067811865476, q_w=0.7071067811865476)
    move_to_table(goalPublisher, goal_pose)

    # goal position should be in robot base frame (correct?)
    place_object(dmp_ros, gripper, target_position=np.array([0.8, 0, 0.94]))
    
    # rospy.sleep(5) NOTE optionally retrieve robot arm after placing the can
    # retrieve_object(dmp_ros, gripper, target_position=np.array([0.3, -0.4, 1.10]))

    # TODO: Report 
    # Anish will start with the basic sections: 
    # Update - completed initial draft of abstract, introduction, Methodology and Results for Manipulation and Detection. 
    # TODO Remaining: Navigation, Integration and Extension sections

    # TODO: Provide shell scripts for seperate actions -> Moritz


    # Extension:
    # Implement on a real TIAGO++ robot
    # Handover beer can to person instead of placing on table
    #   same DMP with different goal pose can be used (?)
    #   Human Detection (YOLO), approximate hand, give goal

    # Detect table? too much work, data set collection etc.
    



    # TODO: retrieve/place position/orientation right now, he is dropping the can because of the high table?? - adjusted the place goal postion to approach above the table
    # TODO: fix nans correlated to depth image calculation (2d point to 3d point?) - seems like this is resolved with our recent codde adaptations
    # TODO: what world should we use for simulation? original or navigation?