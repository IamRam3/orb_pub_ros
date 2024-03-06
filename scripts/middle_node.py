#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Path

def callback(data, pub):
    pub.publish(data)

def middle_node():
    rospy.init_node('middle_subscriber', anonymous=True)
    
    # Define publishers with different names but same message types
    pub1 = rospy.Publisher("/clicked_point_republished", PointStamped, queue_size=10)
    pub2 = rospy.Publisher("/initialpose_republished", PoseWithCovarianceStamped, queue_size=10)
    pub3 = rospy.Publisher("/move_base_simple/goal_republished", PoseStamped, queue_size=10)
    pub4 = rospy.Publisher("/orb_slam3/all_points_republished", PointCloud2, queue_size=10)
    pub5 = rospy.Publisher("/orb_slam3/camera_pose_republished", PoseStamped, queue_size=10)
    pub6 = rospy.Publisher("/orb_slam3/kf_markers_republished", Marker, queue_size=10)
    pub7 = rospy.Publisher("/orb_slam3/kf_markers_array_republished", MarkerArray, queue_size=10)
    pub8 = rospy.Publisher("/orb_slam3/tracked_key_points_republished", PointCloud2, queue_size=10)
    pub9 = rospy.Publisher("/orb_slam3/tracked_points_republished", PointCloud2, queue_size=10)
    pub10 = rospy.Publisher("/orb_slam3/tracking_image_republished", Image, queue_size=10)
    pub11 = rospy.Publisher("/orb_slam3_ros/syscommand_republished", String, queue_size=10)
    pub12 = rospy.Publisher("/orb_slam3_ros/trajectory_republished", Path, queue_size=10)

    # Define subscribers with the original topic names and callback function
    sub1 = rospy.Subscriber("/clicked_point", PointStamped, callback, pub1)
    sub2 = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callback, pub2)
    sub3 = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback, pub3)
    sub4 = rospy.Subscriber("/orb_slam3/all_points", PointCloud2, callback, pub4)
    sub5 = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, callback, pub5)
    sub6 = rospy.Subscriber("/orb_slam3/kf_markers", Marker, callback, pub6)
    sub7 = rospy.Subscriber("/orb_slam3/kf_markers_array", MarkerArray, callback, pub7)
    sub8 = rospy.Subscriber("/orb_slam3/tracked_key_points", PointCloud2, callback, pub8)
    sub9 = rospy.Subscriber("/orb_slam3/tracked_points", PointCloud2, callback, pub9)
    sub10 = rospy.Subscriber("/orb_slam3/tracking_image", Image, callback, pub10)
    sub11 = rospy.Subscriber("/orb_slam3_ros/syscommand", String, callback, pub11)
    sub12 = rospy.Subscriber("/orb_slam3_ros/trajectory", Path, callback, pub12)

    rospy.spin()

if __name__ == '__main__':
    middle_node()