#include <iostream>
#include <sstream>
#include <math.h>

#include "ros/ros.h"

//#include <tf/transform.h>

#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

// calculation relative coordinate of point from current_pose frame
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  ROS_INFO("[%f, %f, %f]\n", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  ROS_INFO("[%f, %f, %f]\n", point_msg.x, point_msg.y, point_msg.z);
  tf::Transform inverse;
  //ROS_INFO("inverse [%f, %f, %f]\n", inverse.x, inverse.y, inverse.z);
  tf::poseMsgToTF(current_pose, inverse);
  //ROS_INFO("inverse [%f, %f, %f]\n", inverse.x, inverse.y, inverse.z);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  ROS_INFO("[%f, %f, %f]\n", tf_point_msg.x, tf_point_msg.y, tf_point_msg.z);
  return tf_point_msg;
}

tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

// distance between target 1 and target2 in 2-D
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

double calcCurvature(geometry_msgs::Point target, geometry_msgs::Pose current_pose_)
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

  if (denominator != 0)
    kappa = numerator / denominator;
  else
  {
    if (numerator > 0)
      kappa = 0.0001;
    else
      kappa = -0.0001;
  }
  ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

double deg2rad(double deg)
{
  return deg * M_PI / 180;
}  // convert degree to radian


double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
  return atan(wheel_base * kappa);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_tf");
	double th = deg2rad(90.0);
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	ROS_INFO("Quanternion: [%f, %f, %f, %f]\n", odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);

	geometry_msgs::Pose current_pose;
	current_pose.position.x = 0.0;
	current_pose.position.y = 0.0;
	current_pose.position.z = 0.0;
	current_pose.orientation = odom_quat;
	// current_pose.orientation.x = 1.0;
	// current_pose.orientation.y = 1.0;
	// current_pose.orientation.z = 1.0;
	// current_pose.orientation.w = 1.0;

	geometry_msgs::Point point_msg;
	// point_msg.x = 1.8228;
	// point_msg.y = 0.8228;
	// point_msg.z = 0.0;

	point_msg.y = 1.8228;
	point_msg.x = 0.8228;
	point_msg.z = 0.0;

	double mykappa =  calcCurvature(point_msg, current_pose);
    ROS_INFO("kappa = [%f]\n", mykappa);

	//with kappa and v calculate the w 
	//v = w*R ------> kapp = 1/R = w / v --------> w = kappa * v
	double twist_linear_x = 1.0;
	double twist_angular_z =  mykappa * twist_linear_x;

	double wheel_base_ = 2.7;
	//with kappa and  wheel_base calculate car-like model steer angle
    double steering_angle = convertCurvatureToSteeringAngle(wheel_base_, mykappa);


	ROS_INFO("w , steer : [%f, %f]\n", twist_angular_z, steering_angle * 180.0 / M_PI);
	//geometry_msgs::Point res = calcRelativeCoordinate(point_msg, current_pose);

	//ROS_INFO("[%f, %f, %f]\n", res.x, res.y, res.z);

	
	return 0;
}
