/*
  cpu_map_atmega2560.h - CPU and pin mapping configuration file
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

/* This cpu_map file serves as a central pin mapping settings file for AVR Mega 2560 */


#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "Atmega2560"

// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

#define SERIAL2_RX USART2_RX_vect
#define SERIAL2_UDRE USART2_UDRE_vect


// Increase Buffers to make use of extra SRAM
//#define RX_BUFFER_SIZE		256
//#define TX_BUFFER_SIZE		128
//#define BLOCK_BUFFER_SIZE	36
//#define LINE_BUFFER_SIZE	100

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR_A      DDRA
#define STEP_PORT_A     PORTA
#define STEP_PIN_A      PINA

#define STEP_DDR_B      DDRC
#define STEP_PORT_B     PORTC
#define STEP_PIN_B      PINC

#define A_STEP0         1
#define A_STEP1         3
#define A_STEP2         5
#define A_STEP3         7
#define STEP_MASK_A     ((1<<A_STEP0)|(1<<A_STEP1)|(1<<A_STEP2)|(1<<A_STEP3)) // GropA step bits


#define B_STEP0         2
#define B_STEP1         4
#define B_STEP2         6
#define STEP_MASK_B     ((1<<B_STEP0)|(1<<B_STEP1)|(1<<B_STEP2)) // GropB step bits

#define X_STEP_BIT_T      A_STEP0
#define Y_STEP_BIT_T      A_STEP1
#define Z_STEP_BIT_T      A_STEP2
#define A_STEP_BIT_T      B_STEP0
#define B_STEP_BIT_T      B_STEP1
#define C_STEP_BIT_T      B_STEP2
#define D_STEP_BIT_T      A_STEP3
#define STEP_MASK_ALL ((1<<X_STEP_BIT_T)|(1<<Y_STEP_BIT_T)|(1<<Z_STEP_BIT_T)|(1<<A_STEP_BIT_T)|(1<<B_STEP_BIT_T)|(1<<C_STEP_BIT_T)|(1<<D_STEP_BIT_T)) // All step bits


// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIR_DDR_A     DDRA
#define DIR_PORT_A    PORTA
#define DIR_PIN_A     PINA

#define DIR_DDR_B     DDRC
#define DIR_PORT_B    PORTC
#define DIR_PIN_B     PINC

#define A_DIR0        0
#define A_DIR1        2
#define A_DIR2        4
#define A_DIR3        6
#define DIR_MASK_A    ((1<<A_DIR0)|(1<<A_DIR1)|(1<<A_DIR2)|(1<<A_DIR3))

#define B_DIR0        1
#define B_DIR1        3
#define B_DIR2        5
#define DIR_MASK_B    ((1<<B_DIR0)|(1<<B_DIR1)|(1<<B_DIR2))

#define X_DIR_BIT   A_DIR0
#define Y_DIR_BIT   A_DIR1 
#define Z_DIR_BIT   A_DIR2 
#define A_DIR_BIT   B_DIR0 
#define B_DIR_BIT   B_DIR1 
#define C_DIR_BIT   B_DIR2 
#define D_DIR_BIT   A_DIR3 
#define DIRECTION_MASK_ALL ((1<<X_DIR_BIT)|(1<<Y_DIR_BIT)|(1<<Z_DIR_BIT)|(1<<A_DIR_BIT)|(1<<B_DIR_BIT)|(1<<C_DIR_BIT)|(1<<D_DIR_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_DDR   DDRG
#define STEPPERS_DISABLE_PORT  PORTG
#define STEPPERS_DISABLE_BIT   1 // MEGA2560 Digital Pin 40
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
#define LIMIT_DDR       DDRD

#define LIMIT_PORT      PORTD
#define LIMIT_PIN       PIND


#define A_LIMIT_BIT     5 // SOPTO4//第二版电路图
#define B_LIMIT_BIT     6 // SOPTO5
#define C_LIMIT_BIT     7 // SOPTO6
#define D_LIMIT_BIT     2 // 未用
#define E_LIMIT_BIT     0 // SOPTO1
#define F_LIMIT_BIT     1 // SOPTO2
#define G_LIMIT_BIT     4 // SOPTO3

/*
#define A_LIMIT_BIT     2 // SOPTO4//这个顺序是赵家与第一版电路图的
#define B_LIMIT_BIT     1 // SOPTO5
#define C_LIMIT_BIT     0 // SOPTO6
#define D_LIMIT_BIT     4 // 未用
#define E_LIMIT_BIT     7 // SOPTO1
#define F_LIMIT_BIT     5 // SOPTO2
#define G_LIMIT_BIT     3 // SOPTO3
*/
																			//PL6_PIN 43对应了SOPT08
#define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT0_vect
#define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
#define LIMIT_MASK ((1<<A_LIMIT_BIT)|(1<<B_LIMIT_BIT)|(1<<C_LIMIT_BIT)|(1<<D_LIMIT_BIT)|(1<<E_LIMIT_BIT)|(1<<F_LIMIT_BIT)|(1<<G_LIMIT_BIT)) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_DDR      DDRH
#define SPINDLE_ENABLE_PORT     PORTH
#define SPINDLE_ENABLE_BIT      4 // MEGA2560 Digital Pin 7
#define SPINDLE_DIRECTION_DDR   DDRE
#define SPINDLE_DIRECTION_PORT  PORTE
#define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_DDR     DDRH
#define COOLANT_FLOOD_PORT    PORTH
#define COOLANT_FLOOD_BIT     5 // MEGA2560 Digital Pin 8
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_DDR    DDRH
#define COOLANT_MIST_PORT   PORTH
#define COOLANT_MIST_BIT    6 // MEGA2560 Digital Pin 9
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_DDR       DDRK
#define CONTROL_PIN       PINK
#define CONTROL_PORT      PORTK
#define RESET_BIT         0  // MEGA2560 Analog Pin 8
#define FEED_HOLD_BIT     1  // MEGA2560 Analog Pin 9
#define CYCLE_START_BIT   2  // MEGA2560 Analog Pin 10
#define SAFETY_DOOR_BIT   3  // MEGA2560 Analog Pin 11
#define CONTROL_INT       PCIE2  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT2_vect
#define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

// Define probe switch input pin.
#define PROBE_DDR       DDRK
#define PROBE_PIN       PINK
#define PROBE_PORT      PORTK
#define PROBE_BIT       7  // MEGA2560 Analog Pin 15
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  // Set Timer up to use TIMER4B which is attached to Digital Pin 7
  #define PWM_MAX_VALUE       65535.0
  #define TCCRA_REGISTER		TCCR4A
  #define TCCRB_REGISTER		TCCR4B
  #define OCR_REGISTER		OCR4B//设置比较值

  #define COMB_BIT			COM4B1
  #define WAVE0_REGISTER		WGM40
  #define WAVE1_REGISTER		WGM41
  #define WAVE2_REGISTER		WGM42
  #define WAVE3_REGISTER		WGM43

  #define SPINDLE_PWM_DDR		DDRG
  #define SPINDLE_PWM_PORT    PORTG
  #define SPINDLE_PWM_BIT		0 // MEGA2560 Digital Pin 41
#endif // End of VARIABLE_SPINDLE


/////////////////////////////新定义的PWM输出引脚//////////////////////////////////////////////////////
//PWM输出针脚初始化
#ifdef VARIABLE_SPINDLE_2

#define SPINDLE_PWM_DDR_2      DDRE
#define SPINDLE_PWM_PORT_2     PORTE
#define SPINDLE_PWM_BIT_2      4 // MEGA2560 Digital Pin 2  OC3B输出PWM

#define PWM_MAX_VALUE_2		65535.0
#define TCCRA_REGISTER_2		  TCCR3A
#define TCCRB_REGISTER_2		  TCCR3B
#define OCR_REGISTER_2	  OCR3B//设置比较值

#define COMB_BIT_2		  COM3B1
#define WAVE0_REGISTER_2		  WGM30
#define WAVE1_REGISTER_2		  WGM31
#define WAVE2_REGISTER_2		  WGM32
#define WAVE3_REGISTER_2		  WGM33

#endif 























