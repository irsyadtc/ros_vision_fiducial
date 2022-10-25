#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>


  /************************
  	QUARTERNION TO EULER
  *************************/
geometry_msgs::Vector3 euler_from_quaternion(geometry_msgs::Quaternion q)
{
	//Convert a quaternion into euler angles (roll, pitch, yaw)
	//roll is rotation around x in radians (counterclockwise)
	//pitch is rotation around y in radians (counterclockwise)
	//yaw is rotation around z in radians (counterclockwise)
	geometry_msgs::Vector3 v;
	double t0 = +2.0 * (q.w * q.x + q.y * q.z);
	double t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	v.x = atan2(t0, t1); //roll

	double t2 = +2.0 * (q.w * q.y - q.z * q.x);
		
	if(t2 > 1.0)
		{ t2 = 1.0;	}

	if(t2 < -1.0)
		{ t2 = -1.0; }
		
	v.y = asin(t2); //pitch
	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
	double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	v.z = atan2(t3, t4); //yaw
     
	return v; // in radians
}
	
  /************************
  	MAIN
  *************************/	
int main(int argc, char** argv){
  ros::init(argc, argv, "pose");

  std::string camera_frame;
  std::string reference_frame;
  
  std_msgs::UInt32 durationMsg;
  
  geometry_msgs::PoseStamped pose_prev;
  geometry_msgs::Vector3 rpy_prev;
  geometry_msgs::TwistStamped twist_prev;
  
  ros::NodeHandle nh;


	 nh.param<std::string>("reference_frame", reference_frame, "");
   nh.param<std::string>("camera_frame", camera_frame, "");

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam",3);
  ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3>("rpy", 5);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cam", 5);
  ros::Publisher accel_pub = nh.advertise<geometry_msgs::AccelStamped>("accel_cam", 5);

  //ros::Publisher duration_pub = nh.advertise<std_msgs::UInt32>("duration", 3);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
	
	ROS_INFO("camera_frame: %s | reference_frame: %s", camera_frame.c_str(), reference_frame.c_str());
  ros::Rate rate(10.0);
  
  while (nh.ok()){
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped poseMsg;
		geometry_msgs::Vector3 rpyMsg;
    geometry_msgs::TwistStamped twistMsg;
    geometry_msgs::AccelStamped accelMsg;

    
    try{
      transformStamped = tfBuffer.lookupTransform(reference_frame, camera_frame,
                               ros::Time::now(),ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    //pose
		//geometry_msgs::PoseStamped posestamped;
		poseMsg.header.frame_id = reference_frame;
		poseMsg.header.stamp = ros::Time::now();
		poseMsg.pose.position.x = transformStamped.transform.translation.x;
		poseMsg.pose.position.y = transformStamped.transform.translation.y;
    poseMsg.pose.position.z = transformStamped.transform.translation.z;
    poseMsg.pose.orientation.x = transformStamped.transform.rotation.x;
    poseMsg.pose.orientation.y = transformStamped.transform.rotation.y;
    poseMsg.pose.orientation.z = transformStamped.transform.rotation.z;
    poseMsg.pose.orientation.w = transformStamped.transform.rotation.w;
		
		pose_pub.publish(poseMsg);
		
		//rpy
		rpyMsg = euler_from_quaternion(transformStamped.transform.rotation);
		
		rpy_pub.publish(rpyMsg);
		
		//twist
		twistMsg.header.frame_id = reference_frame;
		twistMsg.header.stamp = poseMsg.header.stamp;
		uint32_t dura = poseMsg.header.stamp.nsec - pose_prev.header.stamp.nsec;
		twistMsg.twist.linear.x = (poseMsg.pose.position.x - pose_prev.pose.position.x)*pow(10,9)/dura;
		twistMsg.twist.linear.y = (poseMsg.pose.position.y - pose_prev.pose.position.y)*pow(10,9)/dura;
		twistMsg.twist.linear.z = (poseMsg.pose.position.z - pose_prev.pose.position.z)*pow(10,9)/dura; 
    //durationMsg.data = dura;
    twistMsg.twist.angular.x = (rpyMsg.x - rpy_prev.x)*pow(10,9)/dura;
    twistMsg.twist.angular.y = (rpyMsg.y - rpy_prev.y)*pow(10,9)/dura;
    twistMsg.twist.angular.z = (rpyMsg.z - rpy_prev.z)*pow(10,9)/dura;
    
    twist_pub.publish(twistMsg);
    
    //duration_pub.publish(durationMsg);
    
    //acceleration
    accelMsg.header.frame_id = reference_frame;
    accelMsg.header.stamp = poseMsg.header.stamp;
    accelMsg.accel.linear.x = (twistMsg.twist.linear.x - twist_prev.twist.linear.x)*pow(10,9)/dura;
    accelMsg.accel.linear.y = (twistMsg.twist.linear.y - twist_prev.twist.linear.y)*pow(10,9)/dura;
    accelMsg.accel.linear.z = (twistMsg.twist.linear.z - twist_prev.twist.linear.z)*pow(10,9)/dura;
    accelMsg.accel.angular.x = (twistMsg.twist.angular.x - twist_prev.twist.angular.x)*pow(10,9)/dura;
    accelMsg.accel.angular.y = (twistMsg.twist.angular.y - twist_prev.twist.angular.y)*pow(10,9)/dura;
    accelMsg.accel.angular.z = (twistMsg.twist.angular.z - twist_prev.twist.angular.z)*pow(10,9)/dura;
    
    accel_pub.publish(accelMsg);
    
    //record previous values
    pose_prev = poseMsg;
    rpy_prev = rpyMsg;
    twist_prev = twistMsg;
       
    rate.sleep();
  }
  return 0;
};
