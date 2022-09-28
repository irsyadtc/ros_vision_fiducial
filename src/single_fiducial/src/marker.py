#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import math

import tf_conversions
import tf2_ros

#Publish marker 

def callback(data):
    br = tf2_ros.TransformBroadcaster()
    
    roll_x, pitch_y, yaw_z = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    
    t = geometry_msgs.msg.TransformStamped()
    
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "camera_frame"
    t.transform.translation.x = data.pose.position.x
    t.transform.translation.y = data.pose.position.y
    t.transform.translation.z = data.pose.position.z
    ##q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = data.pose.orientation.x
    t.transform.rotation.y = data.pose.orientation.y
    t.transform.rotation.z = data.pose.orientation.z
    t.transform.rotation.w = data.pose.orientation.w
    
    br.sendTransform(t)
    
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

