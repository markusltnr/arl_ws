#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image, CameraInfo
from movement_primitives.dmp import CartesianDMP
import pytransform3d.transformations as pt
import rospkg
from bagpy import bagreader
import matplotlib.pyplot as plt
import sys
from bagpy import bagreader
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from visualization_msgs.msg import Marker
import tf
import message_filters
from robot_fetch_me_a_beer.srv import DetectionControl
    
class ObjectDetector:
    def __init__(self):

        # depth image, will be updated by subscriber
        self.depth_img = np.zeros((480, 640))
        
        self.can_position = None

        # get intrinsic matrix 
        # print("Getting camera info ...")
        camera_info = rospy.wait_for_message('/xtion/depth_registered/camera_info', CameraInfo, timeout=5)
        self.K = np.array(camera_info.K).reshape((3, 3))
        # print("Successfully retrieved intrinsic matrix:")
        # print(self.K)

        # load YOLO model
        self.model = YOLO("/home/user/exchange/arl_ws/src/robot_fetch_me_a_beer/yolo/best.pt")
        
        self.listener = tf.TransformListener()

        # initialize ROS publishers
        self.image_pub = rospy.Publisher("/xtion/rgb/image_raw2", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        self.can_pose_pub = rospy.Publisher("/detected_can_pose", PoseStamped, queue_size=1)
        self.bridge = CvBridge()

        # initialize ROS subscribers
        self.image_sub = message_filters.Subscriber("/xtion/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/xtion/depth_registered/image_raw", Image)

        # Time synchronizer for RGB and Depth images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=2)    # chang params?
        self.ts.registerCallback(self.image_callback)
        
        self._xtion_frame = "xtion_rgb_optical_frame"   # frame of depth camera
        self._map_frame = "map"                         # world frame
        self._base_frame = "base_footprint"             # robot base frame

        self.active = False                             # Is the detector activated or not

        rospy.Service('control_detection', DetectionControl, self.control_detection)

        rospy.loginfo("Successfully initialized Publisher/Subscriber and YOLO model!")

    def control_detection(self, request):
        self.active = request.activate
        return True, f"Detection {'enabled' if self.active else 'disabled'}"


    def image_callback(self, rgb_msg, depth_msg):
        if not self.active:
            return
        
        time = rgb_msg.header.stamp
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")     # or passthrough ?
            self.depth_img = depth_image

            # Detect objects using YOLOv8
            results = self.model.track(source=cv_image, persist=True, conf=0.4, iou=0.4, verbose=False, device='cuda:0')
            
            # Draw bounding boxes and compute 3D positions
            cv_image = self.annotate_image(results[0], time)
            
            # Convert OpenCV image back to ROS Image message and publish
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
          
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def convert_2D_to_3D_point(self, x, y):
        K_inv = np.linalg.inv(self.K)
        depth = self.depth_img[y, x]
        
        point_2d = np.array([x, y , 1])
        
        point_3d = (K_inv @ point_2d) * depth
        point = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        # print('3D point: ' + str(point))
        return point
    
    def base_T_xtion(self, point, time):
        self.listener.waitForTransform(self._xtion_frame, self._base_frame, time, rospy.Duration(5))
        trans, rot = self.listener.lookupTransform(self._xtion_frame, self._base_frame, time)

        pose_xtion = np.array([trans[0], trans[1], trans[2], rot[3], rot[0], rot[1], rot[2]]) 
        base_T_xtion = pt.transform_from_pq(pose_xtion)

        point_map = np.linalg.inv(base_T_xtion) @ point

        return point_map

    def annotate_image(self, result, time):
        for idx in range(len(result.boxes)):
            boxes = result.boxes[idx]
            cls = result.names[int(boxes.cls.item())]

            if cls != 'can':
                print("Detected something else than can")
                continue
            

            x1, y1, x2, y2 = map(int, result.boxes[idx].xyxy[0])

            offset_x = int(np.abs(x1 - x2) / 2)
            offset_y = int(np.abs(y1 - y2) / 2)
            point = self.convert_2D_to_3D_point(x1 + offset_x, y1 + offset_y)
            point = self.base_T_xtion(point, time)
            self.publish_marker(point)
            self.publish_can_pose(point)

            conf = boxes.conf.item()

            cv2.rectangle(result.orig_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(result.orig_img, cls, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 0 ,255), thickness=2)
            cv2.putText(result.orig_img, str(round(conf, 2)), (x1, y2), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 0 ,255), thickness=2)

        return result.orig_img
    
    def publish_marker(self, point):
        marker = Marker()

        marker.header.frame_id = self._base_frame
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.marker_pub.publish(marker)
        self.can_position = np.array([point[0], point[1], point[2]]) # should be removed (?)

    
    def publish_can_pose(self, can_pose):
        can = PoseStamped()
        can.header.frame_id = self._base_frame
        can.header.stamp = rospy.Time.now()

        can.pose.position.x = can_pose[0]
        can.pose.position.y = can_pose[1]
        can.pose.position.z = can_pose[2]
        can.pose.orientation.x = 0.0
        can.pose.orientation.y = 0.0
        can.pose.orientation.z = 0.0
        can.pose.orientation.w = 1.0

        self.can_pose_pub.publish(can)


# TODO NOTE ideally we can make this a child class of dector.ObjectDetector. BUT can we actually use the same service again, or should we have a separate service for this?
class PersonDetector(ObjectDetector):
    def __init__(self):
        ObjectDetector.__init__()
        self.model = YOLO("yolov8s.pt")  # loading pretrained yolomodel from ultralytics trained on coco dataset with 'person' included as a object category
        self.person_position = None


    

if __name__ == '__main__':
    rospy.init_node("detect")

    image_converter = ObjectDetector()

    try: 
        ObjectDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
