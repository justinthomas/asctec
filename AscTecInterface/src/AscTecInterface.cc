#include <ros/ros.h>
#include "AscTecInterface.h"

//#define DEBUG


AscTec::AscTec()
{
  //mycnt = 0;
  serial_callback_set = false;
  voltage_callback_set = false;
  imu_callback_set = false;
  pitch_callback_set = false;
  pelican_status_callback_set = false;
  altitude_callback_set = false;

  imu_counter = 0;
  voltage_counter = 0;
  serial_counter = 0;
  pelican_status_counter = 0;
  altitude_counter = 0;

  port_open = false;

  cmd_ready = false;
  pd_cmd_ready = false;

  pthread_mutex_init(&cmd_buffer_mutex, NULL);
  pthread_mutex_init(&pd_cmd_buffer_mutex, NULL);

  char command[4];
  int csize = sizeof(command);
  
  command[0] = '>';
  command[1] = '*';
  command[2] = '>';
  command[3] = 'c';

  memset((void*)&command_buffer, 0, sizeof(command_buffer));
  memcpy((void*)&command_buffer, (void*)&command, csize);

  char pd_command[4];
  int pd_size = sizeof(pd_command);

  pd_command[0] = '>';
  pd_command[1] = '*';
  pd_command[2] = '>';
  pd_command[3] = 'p';

  memset((void*)&pd_command_buffer, 0, sizeof(pd_command_buffer));
  memcpy((void*)&pd_command_buffer, (void*)&pd_command, pd_size);

  AscTecPacketInit(&packet);

  // Zero pointer for casting data structures
  this->status_data = 0;
  this->filter_data = 0;
  this->pitch_data = 0;
  this->pd_ctrl_input_data = 0;
  this->pelican_data = 0;

  return;
}

AscTec::~AscTec()
{
  if (port_open)
    Disconnect();

  pthread_mutex_destroy(&cmd_buffer_mutex);
  pthread_mutex_destroy(&pd_cmd_buffer_mutex);


  return;
}

int AscTec::Connect(const char* port, unsigned int serial_speed)
{
  if (port_open)
    return 0;

  // Open the serial port 
  if (sd.Connect(port, serial_speed))
    {
      printf("Failed to connect to %s at %u\n", 
             port, serial_speed);
      return -1;
    }

  port_open = true;

  return 0;
}

void AscTec::Disconnect()
{
  if (!port_open)
    return;

  sd.Disconnect();

  port_open = false;

  return;
}

void AscTec::SetHWCommand(float thrust, float roll, float pitch, float yaw,
                          bool cmd_thrust, bool cmd_roll,
                          bool cmd_pitch, bool cmd_yaw)
{
  ctrl_input_t ctrl;
  memset((void*)&ctrl, 0, sizeof(ctrl_input_t));
  
  if (cmd_pitch)
    {
      ctrl.ctrl |= 0x1;
      if (pitch > 1)
        ctrl.pitch = 2047;
      else if (pitch < -1)
        ctrl.pitch = -2047;
      else
        ctrl.pitch = (short)floor(pitch*2047);
    }

  if (cmd_roll)
    {
      ctrl.ctrl |= 0x2;
      if (roll > 1)
        ctrl.roll = 2047;
      else if (roll < -1)
        ctrl.roll = -2047;
      else
        ctrl.roll = (short)floor(roll*2047);
    }

  if (cmd_yaw)
    {
      ctrl.ctrl |= 0x4;
      if (yaw > 1)
        ctrl.yaw = 2047;
      else if (yaw < -1)
        ctrl.yaw = -2047;
      else
        ctrl.yaw = (short)floor(yaw*2047);
    }

  if (cmd_thrust)
    {
      ctrl.ctrl |= 0x8;
      if (thrust > 1)
        ctrl.thrust = 4095;
      else if (thrust < 0)
        ctrl.thrust = 0;
      else
        ctrl.thrust = (short)floor(thrust*4095);
    } 

  ctrl.chksum = ctrl.pitch + ctrl.roll + ctrl.yaw + 
    ctrl.thrust + ctrl.ctrl + 0xAAAA;

  pthread_mutex_lock(&cmd_buffer_mutex);
  memcpy((void*)&command_buffer[4], (void*)&ctrl, sizeof(ctrl));
  // Signal that a command is ready
  cmd_ready = true;
  pthread_mutex_unlock(&cmd_buffer_mutex);

#ifdef DEBUG
  printf("AscTec HW: Thrust: %d, Roll: %d, Pitch: %d, Yaw: %d\n", 
         ctrl.thrust, ctrl.roll, ctrl.pitch, ctrl.yaw);
#endif

  return;
}

void AscTec::SetPDCommand(float thrust, float roll, float pitch, float yaw_delta,
                          float kp_roll, float kd_roll,
                          float kp_pitch, float kd_pitch,
                          float kd_yaw, float p_des, float q_des, float r_des,
			  float roll_delta, float pitch_delta,
			  float vicon_roll, float vicon_pitch, float vicon_bil)
{
  pd_ctrl_input_t ctrl;
  memset((void*)&ctrl, 0, sizeof(pd_ctrl_input_t));

  float tmp = floor(thrust + 0.5);
  tmp = tmp > 200 ? 200 : tmp;
  tmp = tmp < 0 ? 0 : tmp;
  ctrl.thrust = tmp;

  ctrl.roll = roll < 0 ? ceil(RadToDeg(roll)*1e2 - 0.5) : 
    floor(RadToDeg(roll)*1e2 + 0.5);
  ctrl.pitch = pitch < 0 ? ceil(RadToDeg(pitch)*1e2 - 0.5) : 
    floor(RadToDeg(pitch)*1e2 + 0.5);
  ctrl.yaw_delta = yaw_delta < 0 ? ceil(yaw_delta*1e2 - 0.5) : 
    floor(yaw_delta*1e2 + 0.5);

  ctrl.kp_roll = kp_roll < 0 ? 0 : floor(DegToRad(kp_roll)*1e3 + 0.5);
  ctrl.kp_pitch = kp_pitch < 0 ? 0 : floor(DegToRad(kp_pitch)*1e3 + 0.5);

  ctrl.kd_roll = kd_roll < 0 ? 0 : floor(DegToRad(kd_roll)*1e3 + 0.5);
  ctrl.kd_pitch = kd_pitch < 0 ? 0 : floor(DegToRad(kd_pitch)*1e3 + 0.5);
  ctrl.kd_yaw = kd_yaw < 0 ? 0 : floor(DegToRad(kd_yaw)*1e3 + 0.5);

  ctrl.p_des = floor(RadToDeg(p_des)*10 + 0.5);
  ctrl.q_des = floor(RadToDeg(q_des)*10 + 0.5);
  ctrl.r_des = floor(RadToDeg(r_des)*10 + 0.5);

  ctrl.roll_delta = floor(roll_delta + 0.5);
  ctrl.pitch_delta = floor(pitch_delta + 0.5);
  
  ctrl.vicon_roll = floor(vicon_roll + 0.5);
  ctrl.vicon_pitch = floor(vicon_pitch + 0.5);
  ctrl.vicon_bil = floor(vicon_bil + 0.5);

  ctrl.chksum = ctrl.roll + ctrl.pitch + ctrl.yaw_delta + 0xAAAA;

  
  //printf("r p y ctrl.chksum: %i %i %i %i\n",ctrl.roll,ctrl.pitch,ctrl.yaw_delta,ctrl.chksum);
  //printf("AscTec PD: Thrust: %c, Roll: %d, Pitch: %d, Yaw Delta: %d\n", 
  //       ctrl.thrust, ctrl.roll, ctrl.pitch, ctrl.yaw_delta);
  pthread_mutex_lock(&pd_cmd_buffer_mutex);
  memcpy((void*)&pd_command_buffer[4], (void*)&ctrl, sizeof(ctrl));
  pd_cmd_ready = true;
  pthread_mutex_unlock(&pd_cmd_buffer_mutex);

  #ifdef DEBUG
  printf("AscTec PD: Thrust: %c, Roll: %d, Pitch: %d, Yaw Delta: %d\n", 
         ctrl.thrust, ctrl.roll, ctrl.pitch, ctrl.yaw_delta);
  printf("Kp roll,pitch: %u, %u\n", ctrl.kp_roll, ctrl.kp_pitch);
  printf("Kd roll,pitch,yaw: %d, %d, %d\n", 
         ctrl.kd_roll, ctrl.kd_pitch, ctrl.kd_yaw);
  printf("Vicon Bil: %d, Vicon Roll: %d, Vicon Pitch: %d",ctrl.vicon_bil,ctrl.vicon_roll,ctrl.vicon_pitch);
  #endif

  return;
}

void AscTec::SetDMCommand(float u1, float u2, float u3, float u4)
{
  ctrl_input_t ctrl;
  memset((void*)&ctrl, 0, sizeof(ctrl_input_t));

  if (u1 > 1)
    ctrl.thrust = 200;
  else if (u1 < 0)
    ctrl.thrust = 0;
  else
    ctrl.thrust = (short)floor(200.0*u1);
 
  if (u2 > 1)
    ctrl.roll = 200;
  else if (u2 < -1)
    ctrl.roll = 0;
  else
    ctrl.roll = (short)floor(100.0*(u2 + 1.0));

  if (u3 > 1)
    ctrl.pitch = 200;
  else if (u3 < -1)
    ctrl.pitch = 0;
  else
    ctrl.pitch = (short)floor(100.0*(u3 + 1.0));

  if (u4 > 1)
    ctrl.yaw = 200;
  else if (u4 < -1)
    ctrl.yaw = 0;
  else
    ctrl.yaw = (short)floor(100.0*(u4 + 1.0));

  ctrl.chksum = ctrl.pitch + ctrl.roll + ctrl.yaw + 
    ctrl.thrust + ctrl.ctrl + 0xAAAA;

  pthread_mutex_lock(&cmd_buffer_mutex);
  memcpy((void*)&command_buffer[4], (void*)&ctrl, sizeof(ctrl));
  // Signal that a command is ready
  cmd_ready = true;
  pthread_mutex_unlock(&cmd_buffer_mutex);

#ifdef DEBUG
  printf("AscTec DM: Thrust: %d, Roll: %d, Pitch: %d, Yaw: %d\n", 
         ctrl.thrust, ctrl.roll, ctrl.pitch, ctrl.yaw);
#endif

  return;
}

void AscTec::SendCommand()
{
  pthread_mutex_lock(&cmd_buffer_mutex);
  if (cmd_ready)
    {
#ifdef DEBUG
      puts("AscTec sending command");
#endif
      sd.WriteChars(command_buffer, sizeof(command_buffer), 0);
      cmd_ready = false;
    }
  pthread_mutex_unlock(&cmd_buffer_mutex);
  pthread_mutex_lock(&pd_cmd_buffer_mutex);
  if (pd_cmd_ready)
    {
      //      puts("AscTec sending PD command");
      pd_cmd_ready = false;
    
#ifdef DEBUG
      puts("AscTec sending PD command");
#endif
      double tsend = ros::Time::now().toSec();
      //printf("Sending pitch_delta: %f\n", tsend);
      sd.WriteChars(pd_command_buffer, sizeof(pd_command_buffer), 0);

      //mycnt+=1;
      //printf("ctr: %i\n",mycnt);
    }
      pthread_mutex_unlock(&pd_cmd_buffer_mutex);
  return;
}

void AscTec::SetSerialCallback(void (*ptr)(bool /*serial status*/),
                               unsigned int count)
{
  serial_ptr = ptr;
  serial_callback_set = true;
  serial_count = count;
}

void AscTec::SetVoltageCallback(void (*ptr)(float /*voltage*/),
                               unsigned int count)
{
  voltage_ptr = ptr;
  voltage_callback_set = true;
  voltage_count = count;
}

void AscTec::SetPelicanStatusCallback(void (*ptr)(float /*voltage*/,
                                                  unsigned int /*cpu load*/),
                                      unsigned int count)
{
  pelican_status_ptr = ptr;
  pelican_status_callback_set = true;
  pelican_status_count = count;
}

void AscTec::SetAltitudeCallback(void (*ptr)(float /*height*/,
                                             float /*dheight*/),
                                 unsigned int count)
{
  altitude_ptr = ptr;
  altitude_callback_set = true;
  altitude_count = count;
}

void AscTec::SetPitchCallback(void (*ptr)(float /*angle_pitch*/,
      float /*angvel_pitch*/),
    unsigned int count)
{
  pitch_ptr = ptr;
  pitch_callback_set = true;
}

void AscTec::SetIMUCallback(void (*ptr)(float /*roll*/, 
                                        float /*pitch*/, float /*yaw*/,
                                        float /*wx*/, 
                                        float /*wy*/, float /*wz*/,
                                        float /*ax*/, 
                                        float /*ay*/, float /*az*/),
                            unsigned int count)
{
  imu_ptr = ptr;
  imu_callback_set = true;
  imu_count = count;
}

inline double AscTec::GetTimeDouble()
{
  struct timeval ts;
  gettimeofday(&ts,0);
  return ts.tv_sec + ts.tv_usec/1e6;
}

inline double AscTec::DegToRad(double deg)
{
  return deg*M_PI/180.0;
}

inline double AscTec::RadToDeg(double rad)
{
  return rad*180.0/M_PI;
}

void AscTec::Update(double timeout_ms)
{
  int i;
  char c[100];
  int nchars, ret;

  double dt = 0;
  double tstart = GetTimeDouble();
  double trec;
  do
    {
      nchars = sd.ReadChars(c, 100, 1e3);
      
      //if (nchars > 0)
      for(i=0; i < nchars; i++)
        {
          ret = AscTecPacketProcessChar(c[i], &packet);
          
          if (ret > 0)
            {
              uint8_t desc = AscTecPacketGetDescriptor(&packet);
              
              switch(desc)
                {
                case PD_STATUSDATA:
                  status_data = (status_data_t*)AscTecPacketGetPayload(&packet);
                  if (serial_callback_set)
                    {
                      if (status_data->serial > 0)
                        (*serial_ptr)(true);
                      else
                        (*serial_ptr)(false);
                    }
                  if (voltage_callback_set)
                    (*voltage_ptr)(status_data->battery_voltage/1e3);
                  printf("******************************CPU Load: %u\n", status_data->cpu_load);
                  break;
                case PD_FILTERDATA:
                  filter_data = (filter_data_t*)AscTecPacketGetPayload(&packet);
		  printf("yaw: %i q: %i\n ",filter_data->angle_yaw,filter_data->angvel_pitch);
		  if (imu_callback_set)
                    (*imu_ptr)(DegToRad(filter_data->angle_roll/1e2),
                               DegToRad(filter_data->angle_pitch/1e2),
                               DegToRad(filter_data->angle_yaw/1e2),
                               DegToRad(filter_data->angvel_roll*0.0154),
                               DegToRad(filter_data->angvel_pitch*0.0154),
                               DegToRad(filter_data->angvel_yaw*0.0154),
                               0, 0, 0);
                  break;
                case PD_PELICANDATA:
                  pelican_data = (pelican_data_t*)AscTecPacketGetPayload(&packet);

                  if (imu_callback_set)
                    {
                      imu_counter++;
                      if (imu_counter == imu_count)
                        {
                          imu_counter = 0;
                  
                          (*imu_ptr)(DegToRad(pelican_data->angle_roll/1e2),
                                     DegToRad(pelican_data->angle_pitch/1e2),
                                     DegToRad(pelican_data->angle_yaw/1e2),
                                     DegToRad(pelican_data->angvel_roll*0.0154),
                                     DegToRad(pelican_data->angvel_pitch*0.0154),
                                     DegToRad(pelican_data->angvel_yaw*0.0154),
                                     pelican_data->acc_x/1e3*9.81, 
                                     pelican_data->acc_y/1e3*9.81, 
                                     pelican_data->acc_z/1e3*9.81);
                        }
                    }
                 
                  if (pelican_status_callback_set)
                    {
                      pelican_status_counter++;
                      if (pelican_status_counter == pelican_status_count)
                        {
                          pelican_status_counter = 0;
                          (*pelican_status_ptr)(pelican_data->battery_voltage/1e3,
                                                pelican_data->cpu_load);
                        }
                    }

                  if (altitude_callback_set)
                    {
                      altitude_counter++;
                      if (altitude_counter == altitude_count)
                        {
                          altitude_counter = 0;
                          (*altitude_ptr)(pelican_data->height/1e3,
                                          pelican_data->dheight/1e3);
                        }
                    }

                  break;
                case PD_PITCHDATA:
                  pitch_data = (pitch_data_t*)AscTecPacketGetPayload(&packet);
                  //printf("pitch: %i q: %i\n ",pitch_data->angle_pitch,pitch_data->angvel_pitch);
                  if (pitch_callback_set)
                    (*pitch_ptr)(DegToRad(pitch_data->angle_pitch/1e2),
                        DegToRad(pitch_data->angvel_pitch*0.0154));
                  break;
                case PD_PDINPUT:
                  pd_ctrl_input_data = 
                    (pd_ctrl_input_t*)AscTecPacketGetPayload(&packet);
                  #if 0
		  puts("Got PD CTRL INPUT:");
                  printf("roll: kp, kd: %f, %f\n", 
                         pd_ctrl_input_data->kp_roll*180.0/M_PI/1e3, 
                         pd_ctrl_input_data->kd_roll*180.0/M_PI/1e3);
                  printf("pitch: kp, kd: %f, %f\n", 
                         pd_ctrl_input_data->kp_pitch*180.0/M_PI/1e3, 
                         pd_ctrl_input_data->kd_pitch*180.0/M_PI/1e3);
                  printf("yaw: kd: %f\n", pd_ctrl_input_data->kd_yaw*180.0/M_PI/1e3);
                  printf("des: rpyt: %f, %f, %f, %c\n", 
                         DegToRad(pd_ctrl_input_data->roll/1e2),
                         DegToRad(pd_ctrl_input_data->pitch/1e2),
                         pd_ctrl_input_data->yaw_delta/1e2,
                         pd_ctrl_input_data->thrust);
		  #endif
		  trec = ros::Time::now().toSec();
		  printf("Received pitch_delta at: %f\n", trec);
		  printf("pitch_delta: %i\n", pd_ctrl_input_data->pitch_delta);
		  printf("roll: %i\n", pd_ctrl_input_data->roll);
		  printf("pitch: %i\n", pd_ctrl_input_data->pitch);
		  printf("yaw_delta: %i\n", pd_ctrl_input_data->yaw_delta);
		  printf("chksum: %i\n", pd_ctrl_input_data->chksum);
		  
                  
		  break;
		case PD_FOODATA:
		  foo_data = 
                    (foo_data_t*)AscTecPacketGetPayload(&packet);
		  printf("bar = %c\n", foo_data->bar);
                default:
#ifdef DEBUG
                  printf("Unhandled type: %X\n", desc);
#endif
                  return;
                }

              break;
            }
          else if (ret < 0)
            {
#ifdef DEBUG
              printf("ERROR: %s\n", AscTecPacketPrintError(ret));
#endif
            }
        }

      dt = (GetTimeDouble() - tstart)*1e3;
      //printf("dt: %f\n",dt);
    }
  while (dt < timeout_ms);

  return;
}
