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
    rospy.init_node('middle_Publisher')
    #sub = rospy.Publisher("/clicked_point", PointStamped)
    #sub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped)
    #sub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    publ1 = rospy.Publisher("/orb_slam3/all_points", PointCloud2)
    publ2 = rospy.Publisher("/orb_slam3/camera_pose", PoseStamped)
    publ3 = rospy.Publisher("/orb_slam3/kf_markers", Marker)
    publ4 = rospy.Publisher("/orb_slam3/kf_markers_array", MarkerArray)
    publ5 = rospy.Publisher("/orb_slam3/tracked_key_points", PointCloud2)
    publ6 = rospy.Publisher("/orb_slam3/tracked_points", PointCloud2)
    publ7 = rospy.Publisher("/orb_slam3/tracking_image", Image)
    publ8 = rospy.Publisher("/orb_slam3_ros/syscommand", String)
    publ9 = rospy.Publisher("/orb_slam3_ros/trajectory", Path)

    rospy.spin()