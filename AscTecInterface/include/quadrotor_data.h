// Quadrotor data structs sent from the hardware

#ifndef __QUADROTOR_DATA_H__
#define __QUADROTOR_DATA_H__

#define PD_HLSTATUS 0x04
#define PD_IMUDATA 0x05
#define PD_HLVOLTAGE 0x06
#define PD_HLSERIAL 0x07
#define PD_IODATA 0x08
#define PD_CTRLOUT 0x11

typedef struct IO_DATA
{
  // Battery voltage
  short battery_voltage;

  // Serial data enable
  unsigned char serial;

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

  //acc-sensor outputs, calibrated: -1000..+1000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;

  //Controller cycles per second (should be ~1000)
  short cpu_load;
} io_data_t;

typedef struct HL_VOLTAGE 
{
  // Battery voltage
  short battery_voltage;
} hl_voltage_t;

typedef struct HL_SERIAL 
{
  // Serial data enable
  unsigned char serial;
} hl_serial_t;

typedef struct IMU_DATA 
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

  //acc-sensor outputs, calibrated: -1000..+1000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;

} imu_data_t;

typedef struct HL_STATUS
{
  //battery voltages in mV
  short battery_voltage_1;
  short battery_voltage_2;
  
  //Time motors are turning
  short up_time;

  // Time flying
  short flight_time;

  // Current longitude
  int longitude;

  // Current latitude
  int latitude;

  //don't care
  short status;

  //Controller cycles per second (should be ~1000)
  short cpu_load;

  //don't care
  char yaw_enabled;
  char chksum_error; 

  //attitude controller outputs; 0..200 = -100 ..+100%
  short nick;
  short roll;
  short yaw;
  //current thrust (height controller output); 0..200 = 0..100%
  short thrust;

} hl_status_t;

typedef struct CONTROLLER_OUTPUT
{
  //attitude controller outputs; 0..200 = -100 ..+100%
  short nick;
  short roll;
  short yaw;
  //current thrust (height controller output); 0..200 = 0..100%
  short thrust;
} controller_output_t;

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
#endif
