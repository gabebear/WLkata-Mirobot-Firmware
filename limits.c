/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
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
  
#include "grbl.h"


// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR//这个是复位开始以后，最多走多少距离的系数，他是各轴最大运动距离的倍数
  #define HOMING_AXIS_SEARCH_SCALAR  3//1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR//这个是复位碰到形成开关以后的回弹距离系数
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

void limits_init() 
{
  LIMIT_DDR &= ~(LIMIT_MASK); // Set as input pins

  #ifdef DISABLE_LIMIT_PIN_PULL_UP
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Requires external pull-down.
  #else
    LIMIT_PORT |= (LIMIT_MASK);  // Enable internal pull-up resistors. Normal high operation.
  #endif

//由于PCINT中断只能在B端口起作用，而我们用的是D端口，因此无法使用引脚硬件中断！只能寻求软限位
  if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE)) {//如果开启了硬件限位
    LIMIT_PCMSK |= LIMIT_MASK_E;//LIMIT_MASK; // Enable specific pins of the Pin Change Interrupt
    PCICR |= (1 << LIMIT_INT); // Enable Pin Change Interrupt
  } else {
    limits_disable(); 
  }
  
  #ifdef ENABLE_SOFTWARE_DEBOUNCE//使用看门狗来给限位开关做软件消抖
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
  #endif
}


// Disables hard limits.
void limits_disable()
{
  LIMIT_PCMSK &= ~LIMIT_MASK;  // Disable specific pins of the Pin Change Interrupt
  PCICR &= ~(1 << LIMIT_INT);  // Disable Pin Change Interrupt
}


// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where 
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
//将限制状态返回为按位 uint8 变量。每个位指示轴限制, 其中触发为 1, 而未触发为0。应用反转掩码。
//轴是由它们在位位置上的数量定义的, 即 z _ axis 是 (1 < 2) 或位 2, y _ axis 是 (1 < 1) 或位1。
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;
  uint8_t pin = (LIMIT_PIN & LIMIT_MASK);//这句话具体读限位硬件引脚的电平状态
  #ifdef INVERT_LIMIT_PIN_MASK
    pin ^= INVERT_LIMIT_PIN_MASK;
  #endif
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
  if (pin) {  
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
      if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
    }
  }
  return(limit_state);//返回的是各个轴限位是否触发，轴编号的掩码
}

uint8_t limits_get_state_hardlimits()
{
  uint8_t limit_state = 0;
  uint8_t pin = (LIMIT_PIN & LIMIT_MASK);//这句话具体读限位硬件引脚的电平状态
  #ifdef INVERT_LIMIT_PIN_MASK
    pin ^= INVERT_LIMIT_PIN_MASK;
  #endif
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
  if (pin) {  
    uint8_t idx;
    for (idx=4; idx<N_AXIS; idx++) {
      if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
    }
  }
  return(limit_state);//返回的是各个轴限位是否触发，轴编号的掩码
}



// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing 
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a 
// bouncing pin without a debouncing method. A simple software debouncing feature may be enabled 
// through the config.h file, where an extra timer delays the limit pin read by several milli-
// seconds to help with, not fix, bouncing switches.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
#ifndef ENABLE_SOFTWARE_DEBOUNCE
  ISR(LIMIT_INT_vect) // DEFAULT: Limit pin change interrupt process. 
  {
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending 
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be 
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (sys.state != STATE_ALARM) { 
      if (!(sys_rt_exec_alarm)) {
        #ifdef HARD_LIMIT_FORCE_STATE_CHECK
          // Check limit pin state. 
          if (limits_get_state()) {

            mc_reset(); // Initiate system kill.
            bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
          }
        #else
          mc_reset(); // Initiate system kill.
          bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event表示硬极限临界事件
        #endif
      }
    }
  }  
#else // OPTIONAL: Software debounce limit pin routine.
  // Upon limit pin change, enable watchdog timer to create a short delay. 
  ISR(LIMIT_INT_vect) { if (!(WDTCSR & (1<<WDIE))) { WDTCSR |= (1<<WDIE); } }
  ISR(WDT_vect) // Watchdog timer ISR
  {
    WDTCSR &= ~(1<<WDIE); // Disable watchdog timer. 
    if (sys.state != STATE_ALARM) {  // Ignore if already in alarm state. 
      if (!(sys_rt_exec_alarm)) {
        // Check limit pin state. 
        if (limits_get_state()) {
          mc_reset(); // Initiate system kill.
          bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
        }
      }  
    }
  }
#endif

 
// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock 
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically 
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
void limits_go_home(uint8_t cycle_mask) 
{
  if (sys.abort) { return; } // Block if system reset has been issued.

  

  
  // Initialize
  uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
  uint8_t step_pin[N_AXIS];
  float target[N_AXIS];
  float max_travel = 0.0;
  uint8_t idx;

  //第一次遍历各轴
  //1、获取step_pin[idx] = 0000010的每个轴的完整掩码，注意这里是硬件的掩码，帅哥修改成了两组IO口，
  //好在IO编号不同，因此不出现问题！
  //2、按照各个轴最大的复位移动距离计算出公共的最大复位移动距离：max_travel
  
  for (idx=0; idx<N_AXIS; idx++) {  
    // Initialize step pin masks
    step_pin[idx] = get_step_pin_mask(idx);
    #ifdef COREXY    
      if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); } 
    #endif

    if (bit_istrue(cycle_mask,bit(idx))) { 
      // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
      // NOTE: settings.max_travel[] is stored as a negative value.

	  //max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);//这个确定的是复位开始以后最大的复位移动距离，超过该距离会报错homing failed
	  max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR) * MAX_TRAVEL);//这个确定的是复位开始以后最大的复位移动距离，超过该距离会报错homing failed
	}
  }

  // Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
  //设置搜索模式，以寻道速率逼近，快速启用指定的cycle_mask限位开关。
  bool approach = true;//approach代表是否是靠近限位开关的运动，真则是，假则非
  float homing_rate = settings.homing_seek_rate;

  uint8_t limit_state, axislock, n_active_axis;
  do {

    system_convert_array_steps_to_mpos(target,sys.position);//把系统各个轴当前位置的MM值保存到target中

    // Initialize and declare variables needed for homing routine.
    axislock = 0;
    n_active_axis = 0;

	//各个轴第二次轮询操作
	//1、n_active_axis记录了需要复位的轴的个数
	//2、将需要复位的轴的系统绝对位置设置为0
	//3、设置各个复位轴的最大复位移动距离为正或者负max_travel
	//4、axislock是一个整体的需要复位的轴的掩码，就是哪个轴需要复位，掩码的哪一位为1
    for (idx=0; idx<N_AXIS; idx++) {
		uint8_t temp_a = 1;
  			if(idx == B_AXIS)
  				{
					temp_a = 2;//增加的为了增加第五轴的复位距离防止失败
  				}

      // Set target location for active axes and setup computation for homing rate.
      if (bit_istrue(cycle_mask,bit(idx))) {//判断轮询到的轴是否需要复位
        n_active_axis++;//这里统计了cycle_mask掩码里所包含的需要复位轴的个数
        sys.position[idx] = 0;//将需要复位的轴的系统绝对位置设置为0
        // Set target direction based on cycle mask and homing cycle approach state.
        // NOTE: This happens to compile smaller than any other implementation tried.
        if (bit_istrue(settings.homing_dir_mask,bit(idx))) {//判断当前轴的复位方向是否设置为反向
          if (approach) { target[idx] = -max_travel; }//approach作用巧妙，用于控制三次运动过程！
          else { target[idx] = max_travel*temp_a; }
        } else { 
          if (approach) { target[idx] = max_travel; }
          else { target[idx] = -max_travel*temp_a; }
        }        
        // Apply axislock to the step port pins active in this cycle.
        axislock |= step_pin[idx];//这个axislock是一个整体的需要复位的轴的掩码（硬件），就是哪个轴需要复位，掩码的哪一位为1
      }

    }
    homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
                                        //调整使各个轴都以归位速度移动
    sys.homing_axis_lock = axislock;//在复位循环中，将不涉及复位运动的轴锁住，不让其运动

    plan_sync_position(); // Sync planner position to current machine position.将计划器位置同步到当前机器位置
    
    // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
    #ifdef USE_LINE_NUMBERS
      plan_buffer_line(target, homing_rate, false, HOMING_CYCLE_LINE_NUMBER); // Bypass mc_line(). Directly plan homing motion.
    #else
      plan_buffer_line(target, homing_rate, false ,false); // 此处执行复位运动！
    #endif
    
    st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
    st_wake_up(); // Initiate motion
    do {
      if (approach) {//只有approach为真才执行
        // Check limit state. Lock out cycle axes when they change.检查限制状态。当循环轴发生变化时, 锁定它们。
        limit_state = limits_get_state();//这句话具体获取每隔轴限位状态、
        //注意这里是一个所有轴限位状态的掩码，但是不是硬件掩码，而是轴编号的掩码！

	    //第三次轴轮询：
		for (idx=0; idx<N_AXIS; idx++) {
          if (axislock & step_pin[idx]) {//判断下当前轮询到的轴是否需要复位
            if (limit_state & (1 << idx)) //判断一下当前复位的轴是否已经触发限位
				{ axislock &= ~(step_pin[idx]); }//如果触发限位则修改axislock中对应轴位置为0，则ISR中该轴就不会继续发脉冲了
          }
        }
        sys.homing_axis_lock = axislock;//更新sys.homing_axis_lock
      }

      st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.

      // Exit routines: No time to run protocol_execute_realtime() in this loop.
      if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
        // Homing failure: Limit switches are still engaged after pull-off motion
        //这里表示了几种复位失败的情况
        if ( (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET)) ||  // Safety door or reset issued
           (!approach && (limits_get_state() & cycle_mask)) ||  // Limit switch still engaged after pull-off motion
           ( approach && (sys_rt_exec_state & EXEC_CYCLE_STOP)) ) { // Limit switch not found during approach.
          mc_reset(); // Stop motors, if they are running.
          protocol_execute_realtime();
          return;
        } else {
          // Pull-off motion complete. Disable CYCLE_STOP from executing.
          bit_false_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP);
          break;
        } 
      }

    } while (STEP_MASK_ALL & axislock);//这里就是判断只有当所有的轴限位都被触发后，才跳出！！！

    st_reset(); // Immediately force kill steppers and reset step segment buffer.
    plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.

    delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.

    // Reverse direction and reset homing rate for locate cycle(s).反向并重置定位周期的归零率。
    approach = !approach;

    // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
    if (approach) { 
      max_travel = settings.homing_pulloff * HOMING_AXIS_LOCATE_SCALAR; 
      homing_rate = settings.homing_feed_rate;
    } else {
      max_travel = settings.homing_pulloff;    
      homing_rate = settings.homing_seek_rate;
    }
    
  } while (n_cycle-- > 0);
	//这里一次回弹，就是对应n_cycle为3，进入循环三次，即：1、快速运动过去 2、回弹 3、慢速运动过去
	//认真读了以后发现approach参数的设置非常非常巧妙！！！
      
  // The active cycle axes should now be homed and machine limits have been located. By 
  // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
  // can be on either side of an axes, check and set axes machine zero appropriately. Also,
  // set up pull-off maneuver from axes limit switches that have been homed. This provides
  // some initial clearance off the switches and should also help prevent them from falsely
  // triggering when hard limits are enabled or when more than one axes shares a limit pin.
  #ifdef COREXY
    int32_t off_axis_position = 0;
  #endif
  int32_t set_axis_position;
  // Set machine positions for homed limit switches. Don't update non-homed axes.
  for (idx=0; idx<N_AXIS; idx++) {
    // NOTE: settings.max_travel[] is stored as a negative value.
    if (cycle_mask & bit(idx)) {
      #ifdef HOMING_FORCE_SET_ORIGIN
        set_axis_position = 0;
      #else 
        if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff * temp_a)*settings.steps_per_mm[idx]);
        } else {
          set_axis_position = lround(-settings.homing_pulloff * temp_a *settings.steps_per_mm[idx]);
        }
      #endif
      
      #ifdef COREXY
        if (idx==X_AXIS) { 
          off_axis_position = (sys.position[B_MOTOR] - sys.position[A_MOTOR])/2;
          sys.position[A_MOTOR] = set_axis_position - off_axis_position;
          sys.position[B_MOTOR] = set_axis_position + off_axis_position;          
        } else if (idx==Y_AXIS) {
          off_axis_position = (sys.position[A_MOTOR] + sys.position[B_MOTOR])/2;
          sys.position[A_MOTOR] = off_axis_position - set_axis_position;
          sys.position[B_MOTOR] = off_axis_position + set_axis_position;
        } else {
          sys.position[idx] = set_axis_position;
        }        
      #else 
        sys.position[idx] = set_axis_position;
      #endif

    }
  }
  plan_sync_position(); // Sync planner position to homed machine position.
    
  // sys.state = STATE_HOMING; // Ensure system state set as homing before returning. 
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
uint8_t limits_soft_check(float *target)
{
  uint8_t idx;
  uint8_t soft_limit_error = false;
  for (idx=0; idx<N_AXIS; idx++) {//遍历各轴
   
    #ifdef HOMING_FORCE_SET_ORIGIN
      // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      //当启用归位强制设置原点时，软限制检查需要考虑方向性
      // NOTE: max_travel is stored as negative注意：最大行程存储为负数
      /*if (bit_istrue(settings.homing_dir_mask,bit(idx))) {//复位方向检查
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { soft_limit_error = true; }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { soft_limit_error = true; }
      }*/
	  if (target[idx] < -settings.min_travel[idx] || target[idx] > settings.max_travel[idx])
	  	{ soft_limit_error = true; 
		  return idx;
	  	}//修改后只返回触发软限位的轴号

    #else  
      // NOTE: max_travel is stored as negative
      if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { soft_limit_error = true; }
    #endif
    
   // if (soft_limit_error) {
      // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within 
      // workspace volume so just come to a controlled stop so position is not lost. When complete
      // enter alarm mode.

	  
   /*   if (sys.state == STATE_CYCLE) {
        bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD);
        do {
          protocol_execute_realtime();
          if (sys.abort) { return; }
        } while ( sys.state != STATE_IDLE );
      }
    
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_SOFT_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate soft limit critical event  
	  protocol_execute_realtime(); // Execute to enter critical event loop and system abort

	  */
      
    //  return ;
   // }
  }
  return 8;
}
