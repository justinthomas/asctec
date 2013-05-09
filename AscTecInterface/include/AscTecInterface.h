#ifndef __ASCTECINTERFACE__
#define __ASCTECINTERFACE__

// Class object to interface with AscTec Robots

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cmath>

#include "SerialDevice.hh"
#include "AscTecPacket.h"
#include "AscTecDataTypes.h"

class AscTec
{
 public:
  AscTec();
  ~AscTec();

  // Connect to the serial port and set everything up
  int Connect(const char* serial_port, unsigned int serial_speed);
  // Close the serial port
  void Disconnect();
  
  // Cycle once to process serial buffer and callbacks for data
  void Update(double timeout_ms = 1);
  
  // Set the command
  void SetHWCommand(float thrust, float roll, float pitch, float yaw,
                    bool cmd_thrust, bool cmd_roll, 
                    bool cmd_pitch, bool cmd_yaw);

  void SetDMCommand(float u1, float u2, float u3, float u4);
  void SetPDCommand(float thrust, float roll, float pitch, float yaw_delta,
                    float kp_roll, float kd_roll,
                    float kp_pitch, float kd_pitch,
                    float kd_yaw, float p_des, float q_des, float r_des,
		    float roll_delta, float pitch_delta,
		    float vicon_roll, float vicon_pitch, float vicon_bil);

  // Trigger the sending of the command
  void SendCommand();
  
  void SetSerialCallback(void (*ptr)(bool /*serial status*/),
                         unsigned int count = 1);
  void SetVoltageCallback(void (*ptr)(float /*voltage*/), 
                          unsigned int count = 1);
  void SetIMUCallback(void (*ptr)(float /*roll*/, float /*pitch*/, float /*yaw*/,
                                  float /*wx*/, float /*wy*/, float /*wz*/,
                                  float /*ax*/, float /*ay*/, float /*az*/),
                      unsigned int count = 1);
  void SetPelicanStatusCallback(void (*ptr)(float /*voltage*/,
                                            unsigned int /*cpu load*/),
                                unsigned int count = 1);
  void SetAltitudeCallback(void (*ptr)(float /*height*/, float /*dheight*/),
                           unsigned int count = 1);
  void SetPitchCallback(void (*ptr)(float /*angle_pitch*/,
                                    float /*angvel_pitch*/),
                        unsigned int count = 1);
  
 private:
  double DegToRad(double deg);
  double RadToDeg(double rad);
  double GetTimeDouble();

  bool serial_callback_set;
  bool voltage_callback_set;
  bool imu_callback_set;
  bool pitch_callback_set;
  bool pelican_status_callback_set;
  bool altitude_callback_set;

  pthread_mutex_t cmd_buffer_mutex;
  pthread_mutex_t pd_cmd_buffer_mutex;

  char command_buffer[4 + sizeof(ctrl_input_t)];
  char pd_command_buffer[4 + sizeof(pd_ctrl_input_t)];
  bool cmd_ready;
  bool pd_cmd_ready;
  bool port_open;

  unsigned int imu_counter, imu_count;
  unsigned int voltage_counter, voltage_count;
  unsigned int serial_counter, serial_count;
  unsigned int pelican_status_counter, pelican_status_count;
  unsigned int altitude_counter, altitude_count;
  int mycnt;

  status_data_t* status_data;
  filter_data_t* filter_data;
  pitch_data_t* pitch_data;
  pd_ctrl_input_t* pd_ctrl_input_data;
  pelican_data_t* pelican_data;
  foo_data_t* foo_data;

  SerialDevice sd;
  AscTecPacket packet;

  void (*serial_ptr)(bool /*serial status*/);
  void (*voltage_ptr)(float /*voltage*/);
  void (*imu_ptr)(float /*roll*/, float /*pitch*/, float /*yaw*/,
                  float /*wx*/, float /*wy*/, float /*wz*/,
                  float /*ax*/, float /*ay*/, float /*az*/);
  void (*altitude_ptr)(float /*height*/, float /*dheight*/);
  void (*pitch_ptr)(float /*angle_pitch*/, float /*angvel_pitch*/);
  void (*pelican_status_ptr)(float /*voltage*/,
                             unsigned int /*cpu load*/);
};
#endif
