#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_msgs/TFMessage.h>//__JS

#include <iostream>
#include <kinect2_tracker/user_IDs.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265
#define RAD2DEG(x) x*180/PI

geometry_msgs::TransformStamped tf_torso,tf_left_shoulder,tf_left_elbow,tf_left_hand,tf_left_hip,tf_left_knee,tf_left_foot,tf_right_shoulder,tf_right_elbow,tf_right_hand,tf_right_hip,tf_right_knee,tf_right_foot, tf_neck, tf_head;

geometry_msgs::TransformStamped tf_shoulderR, tf_shoulderL, tf_hipR, tf_hipL;

void publishToVrep(tf2_ros::Buffer &tfBuffer, geometry_msgs::Quaternion &shoulderR, geometry_msgs::Quaternion &shoulderL, geometry_msgs::Quaternion &elbowR, geometry_msgs::Quaternion &elbowL)
{
  /*
  geometry_msgs::TransformStamped tf_shoulderR, tf_shoulderL, tf_elbowR, tf_elbowL;
  tf_shoulderR = tfBuffer.lookupTransform("rightShoulder", "right_shoulder", ros::Time(0));
  tf_shoulderL = tfBuffer.lookupTransform("leftShoulder", "left_shoulder", ros::Time(0));
  tfBuffer.waitForTransform("/turtle2", "/turtle1",ros::Time(0), ros::Duration(3.0));
  tfBuffer.waitForTransform("/turtle2", "/turtle1",ros::Time(0), ros::Duration(3.0));

  tf_elbowR = tfBuffer.lookupTransform("rightElbow", "rightShoulder", ros::Time(0));
  tf_elbowL = tfBuffer.lookupTransform("leftElbow", "leftShoulder", ros::Time(0));

  shoulderR = tf_shoulderR.transform.rotation;
  shoulderL = tf_shoulderL.transform.rotation;
  */
}

double calcAngle( geometry_msgs::TransformStamped pt1, geometry_msgs::TransformStamped pt2, geometry_msgs::TransformStamped pt3)
{
  tf2::Vector3 vec1,vec2;
  vec1 = tf2::Vector3(pt1.transform.translation.x-pt2.transform.translation.x,
                    pt1.transform.translation.y-pt2.transform.translation.y,
                    pt1.transform.translation.z-pt2.transform.translation.z);
  vec2 = tf2::Vector3(pt3.transform.translation.x-pt2.transform.translation.x,
                    pt3.transform.translation.y-pt2.transform.translation.y,
                    pt3.transform.translation.z-pt2.transform.translation.z);
  double angle = vec1.angle(vec2);
  return angle;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "vrep_publisher");
  ros::NodeHandle nh;

  ros::Publisher elbow_pub,knee_pub;
  ros::Publisher shoulderR_pub,shoulderL_pub,hipR_pub,hipL_pub;
  ros::Publisher intentionPub_;

  elbow_pub = nh.advertise<geometry_msgs::Vector3>("joint/elbow", 100);
  knee_pub = nh.advertise<geometry_msgs::Vector3>("joint/knee", 100);
  shoulderR_pub = nh.advertise<geometry_msgs::Quaternion>("joint/shoulderR", 100);
  shoulderL_pub = nh.advertise<geometry_msgs::Quaternion>("joint/shoulderL", 100);
  hipR_pub = nh.advertise<geometry_msgs::Quaternion>("joint/hipR", 100);
  hipL_pub = nh.advertise<geometry_msgs::Quaternion>("joint/hipL", 100);
//Intention Publisher __JS
  intentionPub_ = nh.advertise<tf2_msgs::TFMessage>("/intention_pose", 100);


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  tf::TransformBroadcaster tfBroadcast;
  ros::Rate rate(100.0);


  while (nh.ok())
  {
    ros::Time now = ros::Time::now();
    try
    {
      // Torso
      tf_torso = tfBuffer.lookupTransform("camera_frame", "torso",
                               ros::Time(0));
      tf_neck = tfBuffer.lookupTransform("camera_frame", "neck",
                               ros::Time(0));
      tf_head = tfBuffer.lookupTransform("camera_frame", "head",
                               ros::Time(0));
      // Left
      tf_left_shoulder = tfBuffer.lookupTransform("camera_frame", "left_shoulder",
                               ros::Time(0));
      tf_left_elbow = tfBuffer.lookupTransform("camera_frame", "left_elbow",
                               ros::Time(0));
      tf_left_hand = tfBuffer.lookupTransform("camera_frame", "left_hand",
                               ros::Time(0));
      tf_left_hip = tfBuffer.lookupTransform("camera_frame", "left_hip",
                               ros::Time(0));
      tf_left_knee = tfBuffer.lookupTransform("camera_frame", "left_knee",
                               ros::Time(0));
      tf_left_foot = tfBuffer.lookupTransform("camera_frame", "left_foot",
                               ros::Time(0));
      // Right
      tf_right_shoulder = tfBuffer.lookupTransform("camera_frame", "right_shoulder",
                               ros::Time(0));
      tf_right_elbow = tfBuffer.lookupTransform("camera_frame", "right_elbow",
                               ros::Time(0));
      tf_right_hand = tfBuffer.lookupTransform("camera_frame", "right_hand",
                               ros::Time(0));
      tf_right_hip = tfBuffer.lookupTransform("camera_frame", "right_hip",
                               ros::Time(0));
      tf_right_knee = tfBuffer.lookupTransform("camera_frame", "right_knee",
                               ros::Time(0));
      tf_right_foot = tfBuffer.lookupTransform("camera_frame", "right_foot",
                               ros::Time(0));
      // Shoulder
      tf_shoulderR = tfBuffer.lookupTransform("reference_frame", "rightShoulder", ros::Time(0),
                              ros::Duration(1.0));
      tf_shoulderL = tfBuffer.lookupTransform("reference_frame", "leftShoulder", ros::Time(0),
                              ros::Duration(1.0));

      // Hip
      tf_hipR = tfBuffer.lookupTransform("reference_frame", "rightHip", ros::Time(0),
                              ros::Duration(1.0));
      tf_hipL = tfBuffer.lookupTransform("reference_frame", "leftHip", ros::Time(0),
                              ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // publish elbow & knee angle
    geometry_msgs::Vector3 vec_elbow,vec_knee;
    double knee_angle[2], elbow_angle[2];

    elbow_angle[0]=calcAngle(tf_left_shoulder,tf_left_elbow,tf_left_hand);
    elbow_angle[1]=calcAngle(tf_right_shoulder,tf_right_elbow,tf_right_hand);
    knee_angle[0]=calcAngle(tf_left_hip,tf_left_knee,tf_left_foot);
    knee_angle[1]=calcAngle(tf_right_shoulder,tf_right_elbow,tf_right_hand);

    vec_elbow.x = elbow_angle[0];
    vec_elbow.y = elbow_angle[1];
    vec_knee.x = knee_angle[0];
    vec_knee.y = knee_angle[1];

//______JS______
    tf2_msgs::TFMessage intention_msg;
    intention_msg.transforms.push_back(tf_left_shoulder);
    intention_msg.transforms.push_back(tf_left_elbow);
    intention_msg.transforms.push_back(tf_right_shoulder);
    intention_msg.transforms.push_back(tf_right_elbow);

    intentionPub_.publish(intention_msg);

    elbow_pub.publish(vec_elbow);
    knee_pub.publish(vec_knee);

    // publish shoulder quaternion
    geometry_msgs::Quaternion quatShoulderR, quatShoulderL, quatHipR, quatHipL;
    quatShoulderR = tf_shoulderR.transform.rotation;
    quatShoulderL = tf_shoulderL.transform.rotation;
    quatHipR = tf_hipR.transform.rotation;
    quatHipL = tf_hipL.transform.rotation;

    shoulderR_pub.publish(quatShoulderR);
    shoulderL_pub.publish(quatShoulderL);
    hipR_pub.publish(quatHipR);
    hipL_pub.publish(quatHipL);

    rate.sleep();
  }
  return 0;
};
