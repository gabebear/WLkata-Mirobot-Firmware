/**
  ******************************************************************************
  * @file	qinnew.h
  * @author	Thor Zhou	
  * @email	zhoudongxv@yeah.net
  * @date	2019-07-18
  ******************************************************************************
  */
#ifndef qinnew_h
#define qinnew_h


// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 6

#define minirobot

#define Laser

typedef struct {

  float D1;
  float A1;
  float A2;
  float A3;
  float D4;
  float L;

  float offset[N_AXIS];

  uint8_t use_interpolation;
  uint8_t interpolation_num;

  uint8_t use_compensation;
  uint8_t compensation_num;

  uint8_t use_reset_pos;
  uint8_t use_Back_to_text;
} robot_t;


#define QINNEW_VERSION "20191228_2"


#define _use_reset_pos_  


#define Cartesian_x (settings.robot_qinnew.A1 + settings.robot_qinnew.D4)//(208)
#define Cartesian_y (0)//(0)
#define Cartesian_z (settings.robot_qinnew.D1 + settings.robot_qinnew.A2 + settings.robot_qinnew.A3 + settings.robot_qinnew.L)//(143)
#define Cartesian_Rx (0)
#define Cartesian_Ry (0)
#define Cartesian_Rz (0)

#define serial2

#define mini6_board

#define MAX_TRAVEL (-200)

//#define debug

void Inverse(double x_wrist,double y_wrist,double z_wrist,double alpha,double beta,double gama);
void InverseInit(void);
void go_reset_pos();
void Forward(double *angle);
void angle_to_coordinate();
void coordinate_to_angle();
void start_calibration();
void write_reset_distance();
void reset_button_init();
void reset_button_check();








#endif

