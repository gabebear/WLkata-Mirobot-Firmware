/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl
  
  Copyright (c) 2011-2015 Sungeun K. Jeon
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

#include "grbl.h"


// Declare system global variable structure
system_t sys; 


int main(void)
{
  // Initialize system upon power-up.
  serial_init();   // Setup serial baud rate and interrupts
  //printString("\r\nSerial1 init Successful!");
#ifdef serial2
  serial2_init();
  //printString("\r\nSerial2 init Successful!");
#endif
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt
  
  memset(&sys, 0, sizeof(system_t));  // Clear all system variables
  sys.abort = true;   // Set abort to complete initialization
  sei(); // Enable interrupts

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif
  
  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif

	sys.state_last = STATE_IDLE;//初始化最初的上一次系统状态为空闲状态

	sys.sym_homing = 0;//初始化复位为多轴复位
  	sys.reset_homing = 0;
  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.
  
    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
#ifdef serial2
    serial2_reset_read_buffer(); // Clear serial read buffer
#endif  
    gc_init(); // Set g-code parser to default state
    spindle_init();
#ifdef VARIABLE_SPINDLE_2
	spindle_init_2();//第二路PWM的输出
#endif
    coolant_init();
    limits_init(); 
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();//pl.position与sys.position同步
    gc_sync_position();//gc_state.position与sys.position同步

    // Reset system variables.
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
	gc_state.feed_rate = 200;
	
	sys.position_Cartesian[X_Cartesian] = Cartesian_x;//160
	sys.position_Cartesian[Y_Cartesian] = Cartesian_y;//0
	sys.position_Cartesian[Z_Cartesian] = Cartesian_z;//208;
	sys.position_Cartesian[RX_Cartesian] = Cartesian_Rx;// 0;
	sys.position_Cartesian[RY_Cartesian] = Cartesian_Ry;//0;
	sys.position_Cartesian[RZ_Cartesian] = Cartesian_Rz;//0;

	//use_interpolation = 0;//是否使用插补，0为不使用，后期写入EEPROM指令当中
	sys.home_complate_flag = 0;//初始化复位完成标志为假

    reset_button_init();//reset扩展板按键初始化
	
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();
    
  }
  return 0;   /* Never reached */
}
