#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <asctec/PDCmd.h>
#include <ipc_bridge/msgs/asctec_PDCmd.h>

#define NAMESPACE asctec
#define NAME PDCmd

ros::Publisher pub;
NAMESPACE::NAME out_msg;

void callback(const ipc_bridge::NAMESPACE::NAME &msg)
{ 
  out_msg.thrust = msg.thrust;
  out_msg.roll = msg.roll;
  out_msg.pitch = msg.pitch;
  out_msg.yaw_delta = msg.yaw_delta;

  out_msg.kp_roll = msg.kp_roll;
  out_msg.kd_roll = msg.kd_roll;
  out_msg.kp_pitch = msg.kp_pitch;
  out_msg.kd_pitch = msg.kd_pitch;
  out_msg.kd_yaw = msg.kd_yaw;
  out_msg.p_des = msg.p_des;
  out_msg.q_des = msg.q_des;
  out_msg.r_des = msg.r_des;
  out_msg.roll_delta = msg.roll_delta;
  out_msg.pitch_delta = msg.pitch_delta;
  out_msg.vicon_roll = msg.vicon_roll;
  out_msg.vicon_pitch = msg.vicon_pitch;
  out_msg.vicon_bil = msg.vicon_bil;
  

  pub.publish(out_msg);
}

#include "subscriber.h"
