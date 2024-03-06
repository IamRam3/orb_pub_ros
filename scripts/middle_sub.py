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


if __name__=='__main__':
    rospy.init_node('middle_subscriber')
    #sub = rospy.Subscriber("/clicked_point", PointStamped)
    #sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped)
    #sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped)
    sub1 = rospy.Subscriber("/orb_slam3/all_points", PointCloud2)
    sub2 = rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped)
    sub3 = rospy.Subscriber("/orb_slam3/kf_markers", Marker)
    sub4 = rospy.Subscriber("/orb_slam3/kf_markers_array", MarkerArray)
    sub5 = rospy.Subscriber("/orb_slam3/tracked_key_points", PointCloud2)
    sub6 = rospy.Subscriber("/orb_slam3/tracked_points", PointCloud2)
    sub7 = rospy.Subscriber("/orb_slam3/tracking_image", Image)
    sub8 = rospy.Subscriber("/orb_slam3_ros/syscommand", String)
    sub9 = rospy.Subscriber("/orb_slam3_ros/trajectory", Path)

    rospy.spin()