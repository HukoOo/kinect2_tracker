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

#include <iostream>
#include <user_IDs.h>
#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265
#define RAD2DEG(x) x*180/PI

geometry_msgs::TransformStamped tf_torso,tf_left_shoulder,tf_left_elbow,tf_left_hand,tf_left_hip,tf_left_knee,tf_left_foot,tf_right_shoulder,tf_right_elbow,tf_right_hand,tf_right_hip,tf_right_knee,tf_right_foot, tf_neck, tf_head;



Eigen::Quaterniond QuaternionRot(Eigen::Vector3d x1, Eigen::Vector3d y1, Eigen::Vector3d z1,Eigen::Vector3d x2, Eigen::Vector3d y2, Eigen::Vector3d z2)
{
    Eigen::Matrix3d M = x1*x2.transpose() + y1*y2.transpose() + z1*z2.transpose();

    Eigen::Matrix4d N;
    N << M(0,0)+M(1,1)+M(2,2)   ,M(1,2)-M(2,1)          , M(2,0)-M(0,2)         , M(0,1)-M(1,0),
         M(1,2)-M(2,1)          ,M(0,0)-M(1,1)-M(2,2)   , M(0,1)+M(1,0)         , M(2,0)+M(0,2),
         M(2,0)-M(0,2)          ,M(0,1)+M(1,0)          ,-M(0,0)+M(1,1)-M(2,2)  , M(1,2)+M(2,1),
         M(0,1)-M(1,0)          ,M(2,0)+M(0,2)          , M(1,2)+M(2,1)         ,-M(0,0)-M(1,1)+M(2,2);

    Eigen::EigenSolver<Eigen::Matrix4d> N_es(N);
    Eigen::Vector4d::Index maxIndex;
    N_es.eigenvalues().real().maxCoeff(&maxIndex);

    Eigen::Vector4d ev_max = N_es.eigenvectors().col(maxIndex).real();

    Eigen::Quaterniond quat(ev_max(0), ev_max(1), ev_max(2), ev_max(3));
    quat.normalize();

    return quat;
}


void addArrow(visualization_msgs::MarkerArray &markers, geometry_msgs::TransformStamped _from, geometry_msgs::TransformStamped _to, int &id)
{
  visualization_msgs::Marker marker_arrow;
            marker_arrow.ns = "skeleton";
            marker_arrow.id = id;
            id++;
            marker_arrow.header.frame_id = "camera_frame";
            marker_arrow.header.stamp = ros::Time::now();
            marker_arrow.type = visualization_msgs::Marker::ARROW;
            marker_arrow.action = visualization_msgs::Marker::ADD;
            marker_arrow.scale.x = 0.01;
            marker_arrow.scale.y = 0.01;
            marker_arrow.scale.z = 0.01;
            
            marker_arrow.color.r = 1.0f;
            marker_arrow.color.g = 0.0f;
            marker_arrow.color.b = 0.0f;
            marker_arrow.color.a = 1.0f;
            marker_arrow.lifetime = ros::Duration();

            geometry_msgs::Point pt1, pt2;
            pt1.x = _from.transform.translation.x;
            pt1.y = _from.transform.translation.y;
            pt1.z = _from.transform.translation.z;

            pt2.x = _to.transform.translation.x;
            pt2.y = _to.transform.translation.y;
            pt2.z = _to.transform.translation.z;

            marker_arrow.points.push_back(pt1);
            marker_arrow.points.push_back(pt2);

            markers.markers.push_back(marker_arrow);
}

void addText(visualization_msgs::MarkerArray &markers, geometry_msgs::TransformStamped _point, double angle, int &id)
{
  visualization_msgs::Marker marker_text;
            marker_text.ns = "text";
            marker_text.id = id;
            id++;
            marker_text.header.frame_id = "camera_frame";
            marker_text.header.stamp = ros::Time::now();
            marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_text.action = visualization_msgs::Marker::ADD;
            marker_text.scale.z = 0.05;
            
            marker_text.color.r = 1.0f;
            marker_text.color.g = 1.0f;
            marker_text.color.b = 1.0f;
            marker_text.color.a = 1.0f;
            marker_text.lifetime = ros::Duration();
            marker_text.pose.position.x=_point.transform.translation.x;
            marker_text.pose.position.y=_point.transform.translation.y;
            marker_text.pose.position.z=_point.transform.translation.z;
            marker_text.text = std::to_string(RAD2DEG(angle));

            markers.markers.push_back(marker_text);

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
tf::Quaternion calcTransformBetweenVectors(tf::Vector3 fromVec, tf::Vector3 toVec)
{
  Eigen::Vector3d eigenFromVec3;
  Eigen::Vector3d eigenToVec3;
  tf::vectorTFToEigen(fromVec, eigenFromVec3); 
  tf::vectorTFToEigen(toVec, eigenToVec3);  

  Eigen::Quaterniond eigen_Quaternion;
  eigen_Quaternion.setFromTwoVectors(eigenFromVec3, eigenToVec3);
  tf::Quaternion tf_Quaternion;
  tf::quaternionEigenToTF(eigen_Quaternion, tf_Quaternion);

  return tf_Quaternion;
}
void publishBodyTF(tf::TransformBroadcaster tfBroadcast)
{
  tf::Transform upperTF,lowerTF,refTF;
  tf::Transform torsoTF;
 
  tf::Vector3 torsoVec3 = tf::Vector3(tf_torso.transform.translation.x,
                                      tf_torso.transform.translation.y,
                                      tf_torso.transform.translation.z );
 
  tf::Vector3 RshoulderVec3 = tf::Vector3(tf_right_shoulder.transform.translation.x,
                                          tf_right_shoulder.transform.translation.y,
                                          tf_right_shoulder.transform.translation.z ); 
  RshoulderVec3 = (RshoulderVec3 - torsoVec3); 
  
  tf::Vector3 LshoulderVec3 = tf::Vector3(tf_left_shoulder.transform.translation.x,
                                          tf_left_shoulder.transform.translation.y,
                                          tf_left_shoulder.transform.translation.z );
  LshoulderVec3 = (LshoulderVec3 - torsoVec3);
  tf::Vector3 UpperVec3 = LshoulderVec3.cross(RshoulderVec3);

  tf::Vector3 RhipVec3 = tf::Vector3(tf_right_hip.transform.translation.x,
                                     tf_right_hip.transform.translation.y,
                                     tf_right_hip.transform.translation.z ); 
  RhipVec3 = (RhipVec3 - torsoVec3); 
  tf::Vector3 LhipVec3 = tf::Vector3(tf_left_hip.transform.translation.x,tf_left_hip.transform.translation.y,tf_left_hip.transform.translation.z ); 
  LhipVec3 = (LhipVec3 - torsoVec3);
  tf::Vector3 LowerVec3 = RhipVec3.cross(LhipVec3);
  
  // set upper body pose
  upperTF.setOrigin(tf::Vector3(0,0,0));
  
  tf::Vector3 vec_torso2neck= tf::Vector3(tf_neck.transform.translation.x-tf_torso.transform.translation.x,tf_neck.transform.translation.y-tf_torso.transform.translation.y,tf_neck.transform.translation.z-tf_torso.transform.translation.z);
  tf::Vector3 vec_cross= vec_torso2neck.cross(UpperVec3);
  
  // calculate transform matrix between two vectors
  Eigen::Vector3d x1,y1,z1;
  Eigen::Vector3d x2,y2,z2;
  x1 = Eigen::Vector3d(1,0,0).normalized();
  y1 = Eigen::Vector3d(0,1,0).normalized();
  z1 = Eigen::Vector3d(0,0,1).normalized();
  tf::vectorTFToEigen(UpperVec3,x2);
  tf::vectorTFToEigen(vec_cross,y2);
  tf::vectorTFToEigen(vec_torso2neck,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  Eigen::Quaterniond quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);

  tf::Quaternion test_tf;
  tf::quaternionEigenToTF(quat, test_tf);

  upperTF.setRotation(test_tf);	   
  std::stringstream upper_frame_id_stream; 
  std::string upper_frame_id; 
  upper_frame_id_stream <<"/upper_body";
  upper_frame_id = upper_frame_id_stream.str();

  std::stringstream r_frame_id_stream; 
  std::string r_frame_id; 
  r_frame_id_stream << "/torso";
  r_frame_id = r_frame_id_stream.str();
            
  tfBroadcast.sendTransform(tf::StampedTransform(upperTF, ros::Time::now(), r_frame_id, upper_frame_id));

  // publish reference frame of shoulder & hip
  refTF.setOrigin(tf::Vector3(0,0,0));
  tf::Vector3 vec_neck2torso= tf::Vector3(tf_torso.transform.translation.x-tf_neck.transform.translation.x,tf_torso.transform.translation.y-tf_neck.transform.translation.y,tf_torso.transform.translation.z-tf_neck.transform.translation.z);
  tf::Vector3 vec_y= vec_neck2torso.cross(UpperVec3);

  tf::vectorTFToEigen(vec_neck2torso,x2);
  tf::vectorTFToEigen(UpperVec3,y2);
  tf::vectorTFToEigen(vec_y,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);
  tf::quaternionEigenToTF(quat, test_tf);

  refTF.setRotation(test_tf);	   

  std::stringstream ref_frame_id_stream; 
  std::string ref_frame_id; 
  ref_frame_id_stream << "/reference_frame";
  ref_frame_id = ref_frame_id_stream.str();
            
  tfBroadcast.sendTransform(tf::StampedTransform(refTF, ros::Time::now(), r_frame_id, ref_frame_id));
}

void publishLimbTF(tf::TransformBroadcaster tfBroadcast, geometry_msgs::TransformStamped shoulder,geometry_msgs::TransformStamped elbow, geometry_msgs::TransformStamped hand,geometry_msgs::TransformStamped hip, geometry_msgs::TransformStamped knee,geometry_msgs::TransformStamped foot, std::string sideName)
{
  tf::Transform shoulderTF,elbowTF,handTF;
  tf::Transform hipTF,kneeTF,footTF;
  shoulderTF.setOrigin(tf::Vector3(0,0,0));
  elbowTF.setOrigin(tf::Vector3(0,0,0));
  hipTF.setOrigin(tf::Vector3(0,0,0));
  kneeTF.setOrigin(tf::Vector3(0,0,0));

  tf::Vector3 torso2shoulderVec3 = tf::Vector3(shoulder.transform.translation.x-tf_torso.transform.translation.x,shoulder.transform.translation.y-tf_torso.transform.translation.y,shoulder.transform.translation.z-tf_torso.transform.translation.z ); 

  tf::Vector3 torso2hipVec3 = tf::Vector3(hip.transform.translation.x-tf_torso.transform.translation.x,hip.transform.translation.y-tf_torso.transform.translation.y,hip.transform.translation.z-tf_torso.transform.translation.z ); 
  
  //
  tf::Vector3 shoulderVec3 = tf::Vector3(shoulder.transform.translation.x,shoulder.transform.translation.y,shoulder.transform.translation.z ); 
  tf::Vector3 elbowVec3 = tf::Vector3(elbow.transform.translation.x,elbow.transform.translation.y,elbow.transform.translation.z ); 
  tf::Vector3 handVec3 = tf::Vector3(hand.transform.translation.x,hand.transform.translation.y,hand.transform.translation.z ); 

  tf::Vector3 hipVec3 = tf::Vector3(hip.transform.translation.x,hip.transform.translation.y,hip.transform.translation.z ); 
  tf::Vector3 kneeVec3 = tf::Vector3(knee.transform.translation.x,knee.transform.translation.y,knee.transform.translation.z ); 
  tf::Vector3 footVec3 = tf::Vector3(foot.transform.translation.x,foot.transform.translation.y,foot.transform.translation.z ); 


  /////////////////////////////////////////////////////////////
  // Arm

  // set arm axis
  tf::Vector3 elbow_x = (handVec3 - elbowVec3);
  tf::Vector3 shoulder_x = (elbowVec3 - shoulderVec3);
  tf::Vector3 elbow_z = (shoulder_x*-1).cross(elbow_x);
  tf::Vector3 shoulder_z = elbow_z;
  tf::Vector3 elbow_y = elbow_z.cross(elbow_x);
  tf::Vector3 shoulder_y = shoulder_z.cross(shoulder_x);
  
  // calculate transform matrix between two vectors
  Eigen::Vector3d x1,y1,z1;
  Eigen::Vector3d x2,y2,z2;
  x1 = Eigen::Vector3d(1,0,0).normalized();
  y1 = Eigen::Vector3d(0,1,0).normalized();
  z1 = Eigen::Vector3d(0,0,1).normalized();

  // calculate shoulder transform
  tf::vectorTFToEigen(shoulder_x,x2);
  tf::vectorTFToEigen(shoulder_y,y2);
  tf::vectorTFToEigen(shoulder_z,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  Eigen::Quaterniond quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);

  tf::Quaternion quat_tf;
  tf::quaternionEigenToTF(quat, quat_tf);

  shoulderTF.setRotation(quat_tf);	   

  std::string frame_id, ref_frame_id; 
  frame_id = sideName + "Shoulder";
  ref_frame_id = sideName + "_shoulder";
            
  tfBroadcast.sendTransform(tf::StampedTransform(shoulderTF, ros::Time::now(), ref_frame_id, frame_id));


  // calculate elbow transform
  tf::vectorTFToEigen(elbow_x,x2);
  tf::vectorTFToEigen(elbow_y,y2);
  tf::vectorTFToEigen(elbow_z,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);
  tf::quaternionEigenToTF(quat, quat_tf);

  elbowTF.setRotation(quat_tf);	   
  
  frame_id = sideName + "Elbow";
  ref_frame_id = sideName + "_elbow";
            
  tfBroadcast.sendTransform(tf::StampedTransform(elbowTF, ros::Time::now(), ref_frame_id, frame_id));


  /////////////////////////////////////////////////////////////
  // Leg
  
  // set arm axis
  tf::Vector3 knee_x = (footVec3 - kneeVec3);
  tf::Vector3 hip_x = (kneeVec3 - hipVec3);
  tf::Vector3 knee_z = (hip_x*-1).cross(knee_x);
  tf::Vector3 hip_z = knee_z;
  tf::Vector3 knee_y = knee_z.cross(knee_x);
  tf::Vector3 hip_y = hip_z.cross(hip_x);
  
  // calculate transform matrix between two vectors
  x1 = Eigen::Vector3d(1,0,0).normalized();
  y1 = Eigen::Vector3d(0,1,0).normalized();
  z1 = Eigen::Vector3d(0,0,1).normalized();

  // calculate hip transform
  tf::vectorTFToEigen(hip_x,x2);
  tf::vectorTFToEigen(hip_y,y2);
  tf::vectorTFToEigen(hip_z,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);
  tf::quaternionEigenToTF(quat, quat_tf);

  hipTF.setRotation(quat_tf);	   
  
  frame_id = sideName + "Hip";
  ref_frame_id = sideName + "_hip";
            
  tfBroadcast.sendTransform(tf::StampedTransform(hipTF, ros::Time::now(), ref_frame_id, frame_id));


  // calculate knee transform
  tf::vectorTFToEigen(knee_x,x2);
  tf::vectorTFToEigen(knee_y,y2);
  tf::vectorTFToEigen(knee_z,z2);
  x2 = x2.normalized();
  y2 = y2.normalized();
  z2 = z2.normalized();
  quat =  QuaternionRot(x1,y1,z1,x2,y2,z2);
  tf::quaternionEigenToTF(quat, quat_tf);

  kneeTF.setRotation(quat_tf);	  
  
  frame_id = sideName + "Knee";
  ref_frame_id = sideName + "_knee";
            
  tfBroadcast.sendTransform(tf::StampedTransform(kneeTF, ros::Time::now(), ref_frame_id, frame_id));


}


geometry_msgs::Quaternion calcLinkTransform(geometry_msgs::TransformStamped _parent, geometry_msgs::TransformStamped _own, geometry_msgs::TransformStamped _child)
{
  tf::Vector3 parentVec3 = tf::Vector3(_parent.transform.translation.x,_parent.transform.translation.y,_parent.transform.translation.z );
  tf::Vector3 ownVec3 = tf::Vector3(_own.transform.translation.x,_own.transform.translation.y,_own.transform.translation.z );
  tf::Vector3 childVec3 = tf::Vector3(_child.transform.translation.x,_child.transform.translation.y,_child.transform.translation.z );

  tf::Vector3 Vec1 = ownVec3 - parentVec3;
  tf::Vector3 Vec2 = childVec3 - ownVec3;
  Vec1 = Vec1 * 0.1;
  Vec2 = Vec2 * 0.1;


  //conversion of tf:Vec3 to eigen
  Eigen::Vector3d eigenVec1;
  tf::vectorTFToEigen(Vec1, eigenVec1); 
  Eigen::Vector3d eigenVec2;
  tf::vectorTFToEigen(Vec2, eigenVec2);     

  //conversion of tf:vec3 to eigen         
  Eigen::Quaterniond eigenQuaternion;
  eigenQuaternion.setFromTwoVectors(eigenVec1, eigenVec2);
  tf::Quaternion tf_Quaternion;
  tf::quaternionEigenToTF(eigenQuaternion, tf_Quaternion);
  geometry_msgs::Quaternion msg_Quaternion;
  tf::quaternionTFToMsg(tf_Quaternion.normalized(), msg_Quaternion);


  return msg_Quaternion;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "frame_creator");
  ros::NodeHandle nh;
  ros::Publisher marker_pub,elbow_pub,knee_pub,hip_pub,shoulder_pub;
  ros::Publisher shoulderR_pub,shoulderL_pub,elbowR_pub,elbowL_pub;
  
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("skeleton/markers", 100);

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
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


    
    // Calculate torso pose
    publishBodyTF(tfBroadcast);
    publishLimbTF(tfBroadcast, tf_left_shoulder, tf_left_elbow, tf_left_hand, tf_left_hip, tf_left_knee, tf_left_foot, "left");
    publishLimbTF(tfBroadcast, tf_right_shoulder, tf_right_elbow, tf_right_hand, tf_right_hip, tf_right_knee, tf_right_foot, "right"); 

    // Markers for visualization in Rviz
    int id=0;    
    visualization_msgs::MarkerArray markers;
    addArrow(markers, tf_torso, tf_left_shoulder,id);
    addArrow(markers, tf_torso, tf_right_shoulder,id);
    addArrow(markers, tf_torso, tf_left_hip,id);
    addArrow(markers, tf_torso, tf_right_hip,id);
    addArrow(markers, tf_torso, tf_neck,id);
    addArrow(markers, tf_neck, tf_head,id);
    addArrow(markers, tf_left_shoulder, tf_left_elbow,id);
    addArrow(markers, tf_right_shoulder, tf_right_elbow,id);
    addArrow(markers, tf_left_elbow, tf_left_hand,id);
    addArrow(markers, tf_right_elbow, tf_right_hand,id);
    addArrow(markers, tf_left_hip, tf_left_knee,id);
    addArrow(markers, tf_right_hip, tf_right_knee,id);
    addArrow(markers, tf_left_knee, tf_left_foot,id);
    addArrow(markers, tf_right_knee, tf_right_foot,id);
    
    marker_pub.publish(markers);

    rate.sleep();
  }
  return 0;
};
