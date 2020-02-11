/*
  spindle_control.c - spindle control methods
  Part of Grbl
  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

/**
  ******************************************************************************
  * @file	spindle_control.c
  * @Modified by Thor Zhou	
  * @email	zhoudongxv@yeah.net
  * @date	2019-12
  ******************************************************************************
  */




#include "grbl.h"

#define RC_SERVO_SHORT     5       // Timer ticks for 0.6ms pulse duration  (9 for 0.6ms)
#define RC_SERVO_LONG      250       // Timer ticks for 2.5 ms pulse duration  (39 for 2.5ms)

#define RC_SERVO_SHORT_2     5       // Timer ticks for 0.6ms pulse duration  (9 for 0.6ms)
#define RC_SERVO_LONG_2      250       // Timer ticks for 2.5 ms pulse duration  (39 for 2.5ms)



void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
  // combined unless configured otherwise.
  #ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #endif
  // Configure no variable spindle and only enable pin.
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  #endif

  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
  #endif
  spindle_stop();
}

#ifdef VARIABLE_SPINDLE_2
void spindle_init_2()
{
  // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
  // combined unless configured otherwise.
    SPINDLE_PWM_DDR_2 |= (1<<SPINDLE_PWM_BIT_2); 
  spindle_stop_2();
}
#endif


void spindle_stop()
{     // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  OCR_REGISTER = 0;
}


#ifdef VARIABLE_SPINDLE_2
void spindle_stop_2()
{     // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  OCR_REGISTER_2 = 0;
	
   
}
#endif


void spindle_run(uint8_t direction, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  protocol_buffer_synchronize();

  if (direction == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

#ifdef VARIABLE_SPINDLE
		// TODO: Install the optional capability for frequency-based output for servos.
  #ifdef CPU_MAP_ATMEGA2560
		  TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);//TCCR4A:0010 0011
		  TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02 | (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER); 
		  // set to 1/8 Prescaler
		  //TCCR4A:0010 0011
	  //  TCCR4B£ºxxx1 1010 
		  OCR4A = 0xFFFF; 
		  uint16_t current_pwm;
  #else
		  TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
		  TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
		  uint8_t current_pwm;
  #endif
	  
		if (rpm <= 0.0) { spindle_stop(); } // RPM should never be negative, but check anyway.
		else {
	#define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
		  if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
		  else { 
			rpm -= SPINDLE_MIN_RPM; 
			if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
		  }
		  current_pwm = floor( rpm*(PWM_MAX_VALUE/SPINDLE_RPM_RANGE) + 0.5);
	#ifdef MINIMUM_SPINDLE_PWM
			if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
	#endif
		  OCR_REGISTER = current_pwm; // Set PWM pin output  OCR4B
	  
		  // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
	#if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) 
	  #ifdef INVERT_SPINDLE_ENABLE_PIN
			  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
	  #else
			  SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
	  #endif
	#endif
		}
		
#else
		// NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
		// if the spindle speed value is zero, as its ignored anyhow.	   
#endif
  }
}


void spindle_run_2(uint8_t direction, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  protocol_buffer_synchronize();

  if (direction == SPINDLE_DISABLE) {

    spindle_stop_2();

  } else {



      // TODO: Install the optional capability for frequency-based output for servos.
      #define SPINDLE_RPM_RANGE_2 (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      #define RC_SERVO_RANGE_2 (RC_SERVO_LONG_2-RC_SERVO_SHORT_2)


#ifdef VARIABLE_SPINDLE
	// TODO: Install the optional capability for frequency-based output for servos.
  #ifdef CPU_MAP_ATMEGA2560
	  TCCRA_REGISTER_2 = (1<<COMB_BIT_2) | (1<<WAVE1_REGISTER_2) | (1<<WAVE0_REGISTER_2);//TCCR4A:0010 0011
	  TCCRB_REGISTER_2 = (TCCRB_REGISTER_2 & 0b11111000) | 0x02 | (1<<WAVE2_REGISTER_2) | (1<<WAVE3_REGISTER_2); 
	  // set to 1/8 Prescaler
	  //TCCR4A:0010 0011
  //  TCCR4B£ºxxx1 1010 £¬
	  OCR3A = 0xFFFF; 
	  uint16_t current_pwm;
  
	if (rpm <= 0.0) { spindle_stop_2(); } // RPM should never be negative, but check anyway.
	else {
	#define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
	  if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
	  else { 
		rpm -= SPINDLE_MIN_RPM; 
		if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
	  }
	  current_pwm = floor( rpm*(PWM_MAX_VALUE_2/SPINDLE_RPM_RANGE) + 0.5);
	#ifdef MINIMUM_SPINDLE_PWM
		if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
	#endif
	  OCR_REGISTER_2 = current_pwm; // Set PWM pin output 
  
	  // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
	}
	
#else
	// NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
	// if the spindle speed value is zero, as its ignored anyhow.	   
  #ifdef INVERT_SPINDLE_ENABLE_PIN
	  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  #else
	  SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
  #endif
#endif
    
  }
}
#endif

spindle_set_state(uint8_t state, float rpm){
}
