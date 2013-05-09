#include <ros/ros.h>
#include <ipc_bridge/ipc_bridge.h>

#include <asctec/Voltage.h>
#include <ipc_bridge/msgs/asctec_Voltage.h>

#define NAMESPACE asctec
#define NAME Voltage

ipc_bridge::Publisher<ipc_bridge::NAMESPACE::NAME> *p;
ipc_bridge::NAMESPACE::NAME out_msg;

unsigned int frame_id_prior_length = 0;


void callback(const NAMESPACE::NAME::ConstPtr &msg)
{
  ROS_INFO("recieved voltage %2.4f",msg->battery_voltage);
  out_msg.header.seq = msg->header.seq;
  out_msg.header.stamp = msg->header.stamp.toSec();
  out_msg.header.frame_id =  new char[5];
  strcpy(out_msg.header.frame_id, "happy");
  
/*
  if (strlen(msg->header.frame_id.c_str()) != frame_id_prior_length)
    {
      delete[] out_msg.header.frame_id;
      out_msg.header.frame_id = 
        new char[strlen(msg->header.frame_id.c_str()) + 1];
      frame_id_prior_length = strlen(msg->header.frame_id.c_str());
    }
  strcpy(out_msg.header.frame_id, msg->header.frame_id.c_str());
  */

  out_msg.battery_voltage = msg->battery_voltage;

  
   p->Publish(out_msg);   
}

#include "publisher.h"

