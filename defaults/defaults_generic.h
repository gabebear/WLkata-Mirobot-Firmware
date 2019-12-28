/*
  defaults_generic.h - defaults settings configuration file
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The defaults.h file serves as a central default settings file for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   here are supplied by users, so your results may vary. However, this should give you
   a good starting point as you get to know your machine and tweak the settings for your
   nefarious needs. */

#ifndef defaults_h
#define defaults_h


#ifdef  Laser
  #define DEFAULT_A_STEPS_PER_MM 16.000
  #define DEFAULT_B_STEPS_PER_MM 22.220
  #define DEFAULT_C_STEPS_PER_MM 11.11
  #define DEFAULT_D_STEPS_PER_MM 200.0
  #define DEFAULT_E_STEPS_PER_MM 56.73
  #define DEFAULT_F_STEPS_PER_MM 113.33
  #define DEFAULT_G_STEPS_PER_MM 64.0
  #define DEFAULT_A_MAX_RATE 1800.0 // mm/min
  #define DEFAULT_B_MAX_RATE 1500.0 // mm/min
  #define DEFAULT_C_MAX_RATE 2500.0 // mm/min
  #define DEFAULT_D_MAX_RATE 500.0 // mm/min
  #define DEFAULT_E_MAX_RATE 1800.0 // mm/min
  #define DEFAULT_F_MAX_RATE 3000.0 // mm/min
  #define DEFAULT_G_MAX_RATE 1800.0 // mm/min
  #define DEFAULT_A_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_B_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_C_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_D_ACCELERATION 50*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_E_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_F_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_G_ACCELERATION 50.0*60*60 // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_A_MAX_TRAVEL 350.0 // mm
  #define DEFAULT_B_MAX_TRAVEL 36.0 // mm
  #define DEFAULT_C_MAX_TRAVEL 360.0 // mm
  #define DEFAULT_D_MAX_TRAVEL 1000.0 // mm
  #define DEFAULT_E_MAX_TRAVEL 160.0 // mm
  #define DEFAULT_F_MAX_TRAVEL 70.0 // mm
  #define DEFAULT_G_MAX_TRAVEL 60.0 // mm

  #define DEFAULT_A_MIN_TRAVEL 350
  #define DEFAULT_B_MIN_TRAVEL 205
  #define DEFAULT_C_MIN_TRAVEL 360
  #define DEFAULT_D_MIN_TRAVEL 1000
  #define DEFAULT_E_MIN_TRAVEL 100
  #define DEFAULT_F_MIN_TRAVEL 30
  #define DEFAULT_G_MIN_TRAVEL 170
  
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 119
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 0 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION)|(BITFLAG_RT_STATUS_Coordinate_MODE)|(BITFLAG_RT_STATUS_PUMP_PWM))
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 1 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 1  // false
  #define DEFAULT_HOMING_ENABLE 1  // 
  #define DEFAULT_HOMING_DIR_MASK 48 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 1500.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 2000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 12.0 // mm


  #define DEFAULTS_RESET_A 45.0//自己增加的复位值
  #define DEFAULTS_RESET_B 45.0
  #define DEFAULTS_RESET_C 0
  #define DEFAULTS_RESET_D 0
  #define DEFAULTS_RESET_E 45.0
  #define DEFAULTS_RESET_F 45.0
  #define DEFAULTS_RESET_G 45.0

  #define DEFAULT_HOMING_POS_MASK 65 //复位以后在运动的方向位掩码

  #define DEFAULTS_D1 78.0
  #define DEFAULTS_A1 32.0
  #define DEFAULTS_A2 108.0
  #define DEFAULTS_A3 20.0
  #define DEFAULTS_D4 170.0
  #define DEFAULTS_L -25.0
  #define DEFAULTS_use_interpolation 0
  #define DEFAULTS_interpolation_num 50
  #define DEFAULTS_use_compensation 0 
  #define DEFAULTS_compensation_num 2
  #define DEFAULTS_use_reset_pos 1	
  #define DEFAULTS_use_Back_to_text 0
  #define DEFAULTS_offset_x 0
  #define DEFAULTS_offset_y 0
  #define DEFAULTS_offset_z 0



#endif




#endif
