#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>



int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub3 = n.advertise<visualization_msgs::Marker>("marker_3", 3);
  ros::Publisher marker_pub4 = n.advertise<visualization_msgs::Marker>("marker_4", 3);

  // Set our shape as arrow
  uint32_t shape = visualization_msgs::Marker::ARROW;
  
  //tf2 static
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  

  while (ros::ok())
  {
    visualization_msgs::Marker marker3;
    visualization_msgs::Marker marker4;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker3.header.frame_id = "map";
    marker3.header.stamp = ros::Time::now();
    marker4.header.frame_id = "map";
    marker4.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker3.ns = "marker";
    marker3.id = 3;
    marker4.ns = "marker";
    marker4.id = 4;


    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker3.type = shape;
    marker4.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker3.action = visualization_msgs::Marker::ADD;
    marker4.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker3.pose.position.x = 16;
    marker3.pose.position.y = 8.84;
    marker3.pose.position.z = 1;
    //convert euler to quarternion
    tf2::Quaternion quaternion3;
    quaternion3.setRPY(0,0,M_PI);
    
    marker3.pose.orientation.x = quaternion3.getX();
    marker3.pose.orientation.y = quaternion3.getY();
    marker3.pose.orientation.z = quaternion3.getZ();
    marker3.pose.orientation.w = quaternion3.getW();
    
    marker4.pose.position.x = 16;
    marker4.pose.position.y = 7.24;
    marker4.pose.position.z = 1;
    //convert euler to quarternion
    tf2::Quaternion quaternion4;
    quaternion4.setRPY(0,0,M_PI);
    
    marker4.pose.orientation.x = quaternion4.getX();
    marker4.pose.orientation.y = quaternion4.getY();
    marker4.pose.orientation.z = quaternion4.getZ();
    marker4.pose.orientation.w = quaternion4.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker3.scale.x = 0.3;
    marker3.scale.y = 0.1;
    marker3.scale.z = 0.1;
    
    marker4.scale.x = 0.3;
    marker4.scale.y = 0.1;
    marker4.scale.z = 0.1;


    // Set the color -- be sure to set alpha to something non-zero!
    marker3.color.r = 0.0f;
    marker3.color.g = 1.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 1.0;
    
    marker4.color.r = 0.0f;
    marker4.color.g = 1.0f;
    marker4.color.b = 0.0f;
    marker4.color.a = 1.0;

    marker3.lifetime = ros::Duration();
    marker4.lifetime = ros::Duration();

    // Publish the marker
    marker_pub3.publish(marker3);
    marker_pub4.publish(marker4);
    
    
    static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "marker_3";
  static_transformStamped.transform.translation.x = 16;
  static_transformStamped.transform.translation.y = 8.84;
  static_transformStamped.transform.translation.z = 1;
  tf2::Quaternion quat;
  quat.setRPY(-1*M_PI_2,0,M_PI_2);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
  static_broadcaster.sendTransform(static_transformStamped);

    r.sleep();
  }
}
