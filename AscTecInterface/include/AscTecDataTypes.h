#ifndef ASCTEC_DATA_TYPES_H
#define ASCTEC_DATA_TYPES_H

#define PD_FILTERDATA 0x05
#define PD_STATUSDATA 0x06
#define PD_ACCDATA 0x07
#define PD_PDINPUT 0x08
#define PD_PELICANDATA 0x09
#define PD_FOODATA 0x10
#define PD_PITCHDATA 0x17

typedef struct FOO_DATA
{
  unsigned char bar;
} foo_data_t;

typedef struct STATUS_DATA
{
  // Battery voltage
  short battery_voltage;

  // Serial data enable
  unsigned char serial;

  // CPU load
  unsigned short cpu_load;
} status_data_t;

typedef struct PITCH_DATA
{
  // angles derived by integration of gyro_outputs,
  // drift compensated by data fusion;
  // -9000..+9000 pitch(nick) and roll, 0..36000 yaw; 100 = 1 degree
  short angle_pitch;

  //angular velocities, raw values [16 bit], bias free,
  // in 0.0154 °/s (=> 64.8 = 1 °/s)
  short angvel_pitch;
} pitch_data_t;

typedef struct PELICAN_DATA
{
  // angles derived by integration of gyro_outputs,
  // drift compensated by data fusion;
  // -9000..+9000 pitch(nick) and roll, 0..36000 yaw; 100 = 1 degree
  short angle_roll;
  short angle_pitch;
  unsigned short angle_yaw;

  //angular velocities, raw values [16 bit], bias free,
  // in 0.0154 °/s (=> 64.8 = 1 °/s)
  short angvel_roll;
  short angvel_pitch;
  short angvel_yaw;

  short acc_x;
  short acc_y;
  short acc_z;

  //height in mm (after data fusion)
  int height;

  //diff. height in mm/s (after data fusion)
  int dheight;

  short battery_voltage;
  short cpu_load;
} pelican_data_t;

typedef struct FILTER_DATA
{
  // angles derived by integration of gyro_outputs,
  // drift compensated by data fusion; 
  // -9000..+9000 pitch(nick) and roll, 0..36000 yaw; 100 = 1 degree
  short angle_roll;
  short angle_pitch;
  short angle_yaw;
  
  //angular velocities, raw values [16 bit], bias free, 
  // in 0.0154 °/s (=> 64.8 = 1 °/s)
  short angvel_roll;
  short angvel_pitch;
  short angvel_yaw; 
} filter_data_t;

typedef struct ACC_DATA
{
  //acc-sensor outputs, calibrated: -1000..+1000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;
} acc_data_t;

typedef struct CTRL_INPUT 
{ //serial commands (= Scientific Interface)
  short pitch; //Pitch input: -2047..+2047 (0=neutral)
  short roll; //Roll input: -2047..+2047 (0=neutral)
  short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
  short thrust; //Collective: 0..4095 = 0..100%
  short ctrl; /*control byte:
                bit 0: pitch control enabled
                bit 1: roll control enabled
                bit 2: yaw control enabled
                bit 3: thrust control enabled
                These bits can be used to only enable one axis at a
                time and thus to control the other axes manually.
                This usually helps a lot to set up and finetune
                controllers for each axis seperately.
                */
  short chksum;
} ctrl_input_t;

typedef struct PD_INPUT
{
  unsigned short kp_roll, kd_roll;
  unsigned short kp_pitch, kd_pitch;
  unsigned short kd_yaw;
  
  short roll;
  short pitch;
  short yaw_delta;
  unsigned char thrust;
  
  short p_des, q_des, r_des;
  short roll_delta, pitch_delta;
  short vicon_roll, vicon_pitch, vicon_bil;

  short chksum;
} pd_ctrl_input_t;

#endif //ASCTEC_DATA_TYPES_H
