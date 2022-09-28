#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import math

import tf_conversions
import tf2_ros

def callback(data):
    br1 = tf2_ros.TransformBroadcaster()
    br2 = tf2_ros.TransformBroadcaster()
    
    #roll_x, pitch_y, yaw_z = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
    t1 = geometry_msgs.msg.TransformStamped()
    
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_link"
    t1.child_frame_id = "camera_frame"
    t1.transform.translation.x = data.pose.position.x
    t1.transform.translation.y = data.pose.position.y
    t1.transform.translation.z = data.pose.position.z
    ##q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t1.transform.rotation.x = data.pose.orientation.x
    t1.transform.rotation.y = data.pose.orientation.y
    t1.transform.rotation.z = data.pose.orientation.z
    t1.transform.rotation.w = data.pose.orientation.w
    
    br1.sendTransform(t1)
    
    t2 = geometry_msgs.msg.TransformStamped()
    
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_link"
    t2.child_frame_id = "camera_frame"
    t2.transform.translation.x = data.pose.position.x
    t2.transform.translation.y = data.pose.position.y
    t2.transform.translation.z = data.pose.position.z
    ##q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t2.transform.rotation.x = data.pose.orientation.x
    t2.transform.rotation.y = data.pose.orientation.y
    t2.transform.rotation.z = data.pose.orientation.z
    t2.transform.rotation.w = data.pose.orientation.w
    
    br1.sendTransform(t2)
    
def track_marker():
    rospy.init_node('tracking_data', anonymous=True)
    rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
    rospy.spin()
    
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
        
if __name__ == '__main__':
    print('single fiducial tf starts .......')
    track_marker()

