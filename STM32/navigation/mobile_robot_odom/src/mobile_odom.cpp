#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Char.h"
#include "std_msgs/UInt16.h"
#include "angles/angles.h"
#include "iostream"

ros::Publisher pub_velocity;
ros::Timer tim_50hz;

//odometry & gyro
double pos_x,pos_y;
double theta;
double buffer_pos_x, buffer_pos_y, buffer_theta;
double offset_pos_x = 0, offset_pos_y = 0, offset_theta = 0;

//velocity
double vx = 0,vy = 0,vw = 0;
double buffer_vx = 0, buffer_vy = 0, buffer_vw = 0;

void manual_move(int _vx, int _vy, int _vw)
{
    geometry_msgs::Twist msg_velocity;
    msg_velocity.linear.x = vx = _vx;
    msg_velocity.linear.y = vy = _vy;
    msg_velocity.angular.z = vw = _vw;
    pub_velocity.publish(msg_velocity);  
}

void callback_velocity (const geometry_msgs::TwistPtr &msg)
{
    buffer_vx = msg->linear.x;
    buffer_vx = buffer_vx *40;
    buffer_vy = msg->linear.y;
    buffer_vy = buffer_vy *40;
    buffer_vw = msg->angular.z;
    buffer_vw = buffer_vw *40;
}

void callback_odometry (const geometry_msgs::Pose2DConstPtr &msg)
{
    buffer_pos_x = msg->x;
    buffer_pos_y = msg->y;
    buffer_theta = msg->theta;

    pos_x = buffer_pos_x - offset_pos_x;
    pos_y = buffer_pos_y - offset_pos_y;
    theta = buffer_theta - offset_theta;
    theta = 90.0 - theta;
    while (theta < -180)
    theta += 360;
    while (theta > 180)
    theta -= 360;
}

void callback_50hz(const ros::TimerEvent &time)
{
    manual_move(buffer_vx,buffer_vy,buffer_vw);
    ROS_INFO("%f", theta);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  pub_velocity = n.advertise<geometry_msgs::Twist>("pc2stm_velocity",8);

  tf::TransformBroadcaster odom_broadcaster;

  ros::Subscriber sub_odometry_stm = n.subscribe("stm2pc_odometry_buffer",50,callback_odometry);
  ros::Subscriber sub_velocity = n.subscribe("cmd_vel", 8, callback_velocity);

  tim_50hz = n.createTimer(ros::Duration(0.02), callback_50hz);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    /*double dt = (current_time - last_time).toSec();
	
    double delta_x = (buffer_vx * cos(120 * 0.0174533)) + (buffer_vy * sin(120 * 0.0174533)) + buffer_vw;
    double delta_y = (buffer_vx * cos(240 * 0.0174533)) + (buffer_vy * sin(240 * 0.0174533)) + buffer_vw;
    double delta_th = (buffer_vx * cos(0 * 0.0174533)) + (buffer_vy * sin(0 * 0.0174533)) + buffer_vw;
	
    pos_x += delta_x;
    pos_y += delta_y;
    theta += delta_th;*/

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = buffer_vx;
    odom.twist.twist.linear.y = buffer_vy;
    odom.twist.twist.angular.z = buffer_vw;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
