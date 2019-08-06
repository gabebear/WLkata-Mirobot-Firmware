/*
  gcode.c - rs274/ngc parser.
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

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision)s to an integer.
#define MAX_LINE_NUMBER 9999999 

#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1 
#define AXIS_COMMAND_MOTION_MODE 2
#define AXIS_COMMAND_TOOL_LENGTH_OFFSET 3 // *Undefined but required

// Declare gc extern struct
parser_state_t gc_state;
parser_block_t gc_block;

#define FAIL(status) return(status);


void gc_init() 
{
  memset(&gc_state, 0, sizeof(parser_state_t));
  
  // Load default G54 coordinate system.
  if (!(settings_read_coord_data(gc_state.modal.coord_select,gc_state.coord_system))) { 
    report_status_message(STATUS_SETTING_READ_FAIL); 
  } 
}


// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
void gc_sync_position() 
{
  system_convert_array_steps_to_mpos(gc_state.position,sys.position);
}


static uint8_t gc_check_same_position(float *pos_a, float *pos_b) 
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    if (pos_a[idx] != pos_b[idx]) { return(false); }
  }
  return(true);
}
         
// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floating point values (no whitespace). Comments and block delete
// characters have been removed. In this function, all units and positions are converted and 
// exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine 
// coordinates, respectively.
uint8_t gc_execute_line(char *line) 
{
  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parser and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new 
     block. This struct contains all of the necessary information to execute the block. */

  memset(&gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct.
  memcpy(&gc_block.modal,&gc_state.modal,sizeof(gc_modal_t)); // Copy current modes
  uint8_t axis_command = AXIS_COMMAND_NONE;
  uint8_t axis_0, axis_1, axis_linear;
  uint8_t coord_select = 0; // Tracks G10 P coordinate selection for execution
  float coordinate_data[N_AXIS]; // Multi-use variable to store coordinate data for execution
  float parameter_data[N_AXIS]; // Multi-use variable to store parameter data for execution
  
  // Initialize bitflag tracking variables for axis indices compatible operations.
  uint8_t axis_words = 0; // XYZ tracking
  uint8_t ijk_words = 0; // IJK tracking 

  // Initialize command and value words variables. Tracks words contained in this block.
  uint16_t command_words = 0; // G and M command words. Also used for modal group violations.用来检测是否同一组的指令并存
  uint32_t value_words = 0; // Value words. 

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also, 
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */
     
  uint8_t word_bit; // Bit-value for assigning tracking variables
  uint8_t char_counter = 0;  
  char letter;
  float value;
  uint8_t int_value = 0;
  uint16_t mantissa = 0;

  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    
    // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
    letter = line[char_counter];
	if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]

    // Convert values to smaller uint8 significand and mantissa values for parsing this word.
    // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more 
    // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
    // accurate enough for value words that require integers to within 0.0001. This should be
    // a good enough comprimise and catch most all non-integer errors. To make it compliant, 
    // we would simply need to change the mantissa to int16, but this add compiled flash space.
    // Maybe update this later. 
    int_value = trunc(value);
    mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.
        // NOTE: Rounding must be used to catch small floating point errors. 

    // Check if the g-code word is supported or errors due to modal group violations or has
    // been repeated in the g-code block. If ok, update the command or record its value.
    switch(letter) {
    
      /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
         NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */
         
      case 'G':
        // Determine 'G' command and its modal group
        switch(int_value) {
          case 10: case 28: case 30: case 92: 
            // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
              if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
              axis_command = AXIS_COMMAND_NON_MODAL;
            }
            // No break. Continues to next line.
          case 4: case 53: 
            word_bit = MODAL_GROUP_G0; 
            switch(int_value) {
              case 4: gc_block.non_modal_command = NON_MODAL_DWELL; break; // G4
              case 10: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_DATA; break; // G10
              case 28:
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_0; break;  // G28
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_0; break; // G28.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G28.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 30: 
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_GO_HOME_1; break;  // G30
                  case 10: gc_block.non_modal_command = NON_MODAL_SET_HOME_1; break; // G30.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G30.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 53: gc_block.non_modal_command = NON_MODAL_ABSOLUTE_OVERRIDE; break; // G53
              case 92: 
                switch(mantissa) {
                  case 0: gc_block.non_modal_command = NON_MODAL_SET_COORDINATE_OFFSET; break; // G92
                  case 10: gc_block.non_modal_command = NON_MODAL_RESET_COORDINATE_OFFSET; break; // G92.1
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G92.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;      
            }
            break;
          case 0: case 1: case 2: case 3: case 38: 
            // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
            // * G43.1 is also an axis command but is not explicitly defined this way.
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict]
            axis_command = AXIS_COMMAND_MOTION_MODE; 
            // No break. Continues to next line.
          case 80: 
            word_bit = MODAL_GROUP_G1; 
            switch(int_value) {
              case 0: gc_block.modal.motion = MOTION_MODE_SEEK; break; // G0
              case 1: gc_block.modal.motion = MOTION_MODE_LINEAR; break; // G1
              case 2: gc_block.modal.motion = MOTION_MODE_CW_ARC; break; // G2
              case 3: gc_block.modal.motion = MOTION_MODE_CCW_ARC; break; // G3
              case 38: 
                switch(mantissa) {
                  case 20: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD; break; // G38.2
                  case 30: gc_block.modal.motion = MOTION_MODE_PROBE_TOWARD_NO_ERROR; break; // G38.3
                  case 40: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY; break; // G38.4
                  case 50: gc_block.modal.motion = MOTION_MODE_PROBE_AWAY_NO_ERROR; break; // G38.5
                  default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G38.x command]
                }
                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                break;
              case 80: gc_block.modal.motion = MOTION_MODE_NONE; break; // G80
            }            
            break;
          case 17: case 18: case 19: 
            word_bit = MODAL_GROUP_G2; 
            switch(int_value) {
              case 17: gc_block.modal.plane_select = PLANE_SELECT_XY; break;
              case 18: gc_block.modal.plane_select = PLANE_SELECT_ZX; break;
              case 19: gc_block.modal.plane_select = PLANE_SELECT_YZ; break;
            }
            break;
          case 90: case 91: 
            if (mantissa == 0) {
              word_bit = MODAL_GROUP_G3; 
              if (int_value == 90) { gc_block.modal.distance = DISTANCE_MODE_ABSOLUTE; } // G90
              else { gc_block.modal.distance = DISTANCE_MODE_INCREMENTAL; } // G91
            } else {
              word_bit = MODAL_GROUP_G4;
              if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
              mantissa = 0; // Set to zero to indicate valid non-integer G command.
              // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
            }
            break;
          case 93: case 94: 
            word_bit = MODAL_GROUP_G5; 
            if (int_value == 93) { gc_block.modal.feed_rate = FEED_RATE_MODE_INVERSE_TIME; } // G93   G93为在多少分钟内完成该动作
            else { gc_block.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN; } // G94   速度模式设置，G94为默认，mm/min
            break;
          case 20: case 21: 
            word_bit = MODAL_GROUP_G6; 
            if (int_value == 20) { gc_block.modal.units = UNITS_MODE_INCHES; }  // G20  mm/inch
            else { gc_block.modal.units = UNITS_MODE_MM; } // G21  默认的速度单位 mm/min
            break;
          case 40:
            word_bit = MODAL_GROUP_G7;
            // NOTE: Not required since cutter radius compensation is always disabled. Only here
            // to support G40 commands that often appear in g-code program headers to setup defaults.
            // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
            break;
          case 43: case 49:
            word_bit = MODAL_GROUP_G8;
            // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
            // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
            // all are explicit axis commands, regardless if they require axis words or not. 
            if (axis_command) { FAIL(STATUS_GCODE_AXIS_COMMAND_CONFLICT); } // [Axis word/command conflict] }
            axis_command = AXIS_COMMAND_TOOL_LENGTH_OFFSET;
            if (int_value == 49) { // G49
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_CANCEL; 
            } else if (mantissa == 10) { // G43.1
              gc_block.modal.tool_length = TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC;
            } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported G43.x command]
            mantissa = 0; // Set to zero to indicate valid non-integer G command.
            break;
          case 54: case 55: case 56: case 57: case 58: case 59: 
            // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
            word_bit = MODAL_GROUP_G12;
            gc_block.modal.coord_select = int_value-54; // Shift to array indexing.
            break;
          case 61:
            word_bit = MODAL_GROUP_G13;
            if (mantissa != 0) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G61.1 not supported]
            // gc_block.modal.control = CONTROL_MODE_EXACT_PATH; // G61
            break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported G command]
        }      
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [Unsupported or invalid Gxx.x command]
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) ) { FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;
        
      case 'M':
      
        // Determine 'M' command and its modal group
        if (mantissa > 0) { FAIL(STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER); } // [No Mxx.x commands]
        switch(int_value) {
          case 0: case 1: case 2: case 30: 
            word_bit = MODAL_GROUP_M4; 
            switch(int_value) {
              case 0: gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED; break; // Program pause
              case 1: break; // Optional stop not supported. Ignore.
              case 2: case 30: gc_block.modal.program_flow = PROGRAM_FLOW_COMPLETED; break; // Program end and reset 
            }
            break;
          #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
            case 4: 
          #endif
          case 3: case 5:
           // word_bit = MODAL_GROUP_M7; 
            switch(int_value) {
              case 3: gc_block.modal.spindle = SPINDLE_ENABLE_CW; break;//第一个PWM开启，PH4针脚
              #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
              case 4: gc_block.modal.spindle = SPINDLE_ENABLE_CCW; break;//第二个PWM开启，PE4针脚
              #endif
              case 5: gc_block.modal.spindle = SPINDLE_DISABLE; break;//PWM输出全部停止
            }
            break;            
         #ifdef ENABLE_M7  
          case 7:
         #endif
          case 8: case 9:
            word_bit = MODAL_GROUP_M8; 
            switch(int_value) {      
             #ifdef ENABLE_M7
              case 7: gc_block.modal.coolant = COOLANT_MIST_ENABLE; break;
             #endif
              case 8: gc_block.modal.coolant = COOLANT_FLOOD_ENABLE; break;
              case 9: gc_block.modal.coolant = COOLANT_DISABLE; break;
            }
            break;
          case 95:
            if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { 
              sys.state = STATE_HOMING; // Set system state variable
              // Only perform homing if Grbl is idle or lost.
              
              // TODO: Likely not required.
              if (system_check_safety_door_ajar()) { // Check safety door switch before homing.
                bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
                protocol_execute_realtime(); // Enter safety door mode.
              }
              
              
              mc_homing_cycle(); 
              if (!sys.abort) {  // Execute startup scripts after successful homing.
                sys.state = STATE_IDLE; // Set to IDLE when complete.
                st_go_idle(); // Set steppers to the settings idle state before returning.
              }
            } else { return(STATUS_SETTING_DISABLED); }
            break;
		 case 20:

		 	printString_debug("M20: Cartesian mode start.\r\n");
	
		 	gc_state.coord_mode = coordinate_mode;
			angle_to_coordinate();
			break;
		 case 21:
	
		 	printString_debug("M21: Angle mode start.\r\n");

		 	gc_state.coord_mode = angle_mode;
			coordinate_to_angle();
		 	break;
		 	
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); // [Unsupported M command]
        }
      
        // Check for more than one command per modal group violations in the current block
        // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
        if ( bit_istrue(command_words,bit(word_bit)) )//实际是检查了上次和这次的word_bit是否一样，确定是否有同一组的指令并存
			{ FAIL(STATUS_GCODE_MODAL_GROUP_VIOLATION); }
        command_words |= bit(word_bit);
        break;
      
      // NOTE: All remaining letters assign values.
      default: 
  
        /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
           legal g-code words and stores their value. Error-checking is performed later since some
           words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */
        switch(letter){
          // case 'A': // Not supported
          // case 'B': // Not supported
          // case 'C': // Not supported
          // case 'D': // Not supported
          case 'F': word_bit = WORD_F; gc_block.values.f = value; break;
          // case 'H': // Not supported
          case 'I': word_bit = WORD_I; gc_block.values.ijk[A_AXIS] = value; ijk_words |= (1<<A_AXIS); break;
          case 'J': word_bit = WORD_J; gc_block.values.ijk[B_AXIS] = value; ijk_words |= (1<<B_AXIS); break;
          case 'K': word_bit = WORD_K; gc_block.values.ijk[C_AXIS] = value; ijk_words |= (1<<C_AXIS); break;
          case 'L': word_bit = WORD_L; gc_block.values.l = int_value; break;
          case 'N': word_bit = WORD_N; gc_block.values.n = trunc(value); break;
          case 'P': word_bit = WORD_P; gc_block.values.p = value; break;
          // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
          // case 'Q': // Not supported
          case 'R': word_bit = WORD_R; gc_block.values.r = value; break;
          case 'S': word_bit = WORD_S; gc_block.values.s = value; break;
		#ifdef VARIABLE_SPINDLE_2
		  case 'E': word_bit = WORD_E;gc_block.values.s_2 = value; break;
		#endif
          case 'T': word_bit = WORD_T; break; // gc.values.t = int_value;//借用了原来的T，把word_bit改成了E
          case 'A': word_bit = WORD_A; gc_block.values.xyz[A_AXIS] = value; axis_words |= (1<<A_AXIS); break;
          case 'B': word_bit = WORD_B; gc_block.values.xyz[B_AXIS] = value; axis_words |= (1<<B_AXIS); break;
          case 'C': word_bit = WORD_C; gc_block.values.xyz[C_AXIS] = value; axis_words |= (1<<C_AXIS); break;
          case 'D': word_bit = WORD_D; gc_block.values.xyz[D_AXIS] = value; axis_words |= (1<<D_AXIS); break;
          case 'X': word_bit = WORD_X; gc_block.values.xyz[E_AXIS] = value; axis_words |= (1<<E_AXIS); break;
          case 'Y': word_bit = WORD_Y; gc_block.values.xyz[F_AXIS] = value; axis_words |= (1<<F_AXIS); break;
          case 'Z': word_bit = WORD_Z; gc_block.values.xyz[G_AXIS] = value; axis_words |= (1<<G_AXIS); break;
          default: FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND);
        } 
        
        // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
        if (bit_istrue(value_words,bit(word_bit))) { FAIL(STATUS_GCODE_WORD_REPEATED); } // [Word repeated]
        // Check for invalid negative values for words F, N, P, T, and S.
        // NOTE: Negative value check is done here simply for code-efficiency.
        if ( bit(word_bit) & (bit(WORD_F)|bit(WORD_N)|bit(WORD_P)|bit(WORD_T)|bit(WORD_S)|bit(WORD_E)) ) {
          if (value < 0.0) { FAIL(STATUS_NEGATIVE_VALUE); } // [Word value cannot be negative]
        }
        value_words |= bit(word_bit); // Flag to indicate parameter assigned.
      
    }   
  } 
  // Parsing complete!
  

  /* -------------------------------------------------------------------------------------
     STEP 3: Error-check all commands and values passed in this block. This step ensures all of
     the commands are valid for execution and follows the NIST standard as closely as possible.
     If an error is found, all commands and values in this block are dumped and will not update
     the active system g-code modes. If the block is ok, the active system g-code modes will be
     updated based on the commands of this block, and signal for it to be executed. 
     
     Also, we have to pre-convert all of the values passed based on the modes set by the parsed
     block. There are a number of error-checks that require target information that can only be
     accurately calculated if we convert these values in conjunction with the error-checking.
     This relegates the next execution step as only updating the system g-code modes and 
     performing the programmed actions in order. The execution step should not require any 
     conversion calculations and would only require minimal checks necessary to execute.
  */

  /* NOTE: At this point, the g-code block has been parsed and the block line can be freed.
     NOTE: It's also possible, at some future point, to break up STEP 2, to allow piece-wise 
     parsing of the block on a per-word basis, rather than the entire block. This could remove 
     the need for maintaining a large string variable for the entire block and free up some memory. 
     To do this, this would simply need to retain all of the data in STEP 1, such as the new block
     data struct, the modal group and value bitflag tracking variables, and axis array indices 
     compatible variables. This data contains all of the information necessary to error-check the 
     new g-code block when the EOL character is received. However, this would break Grbl's startup
     lines in how it currently works and would require some refactoring to make it compatible.
  */  
  
  // [0. Non-specific/common error-checks and miscellaneous setup]: 
  
  // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
  // command has been sent. If so, set axis command to current motion mode.
  if (axis_words) {
    if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // Assign implicit motion-mode
  }
  
  // Check for valid line number N value.
  if (bit_istrue(value_words,bit(WORD_N))) {
    // Line number value cannot be less than zero (done) or greater than max line number.
    if (gc_block.values.n > MAX_LINE_NUMBER) { FAIL(STATUS_GCODE_INVALID_LINE_NUMBER); } // [Exceeds max line number]
  }
  // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking.
  
  // Track for unused words at the end of error-checking.
  // NOTE: Single-meaning value words are removed all at once at the end of error-checking, because
  // they are always used when present. This was done to save a few bytes of flash. For clarity, the
  // single-meaning value words may be removed as they are used. Also, axis words are treated in the
  // same way. If there is an explicit/implicit axis command, XYZ words are always used and are 
  // are removed at the end of error-checking.  
  
  // [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol.
  
  // [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
  //   is not defined after switching to G94 from G93.
  if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
    // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
    if (axis_command == AXIS_COMMAND_MOTION_MODE) { 
      if ((gc_block.modal.motion != MOTION_MODE_NONE) || (gc_block.modal.motion != MOTION_MODE_SEEK)) {
        if (bit_isfalse(value_words,bit(WORD_F))) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [F word missing]
      }
    }
    // NOTE: It seems redundant to check for an F word to be passed after switching from G94 to G93. We would
    // accomplish the exact same thing if the feed rate value is always reset to zero and undefined after each
    // inverse time block, since the commands that use this value already perform undefined checks. This would
    // also allow other commands, following this switch, to execute and not error out needlessly. This code is 
    // combined with the above feed rate mode and the below set feed rate error-checking.

    // [3. Set feed rate ]: F is negative (done.)
    // - In inverse time mode: Always implicitly zero the feed rate value before and after block completion.
    // NOTE: If in G93 mode or switched into it from G94, just keep F value as initialized zero or passed F word 
    // value in the block. If no F word is passed with a motion command that requires a feed rate, this will error 
    // out in the motion modes error-checking. However, if no F word is passed with NO motion command that requires
    // a feed rate, we simply move on and the state feed rate value gets updated to zero and remains undefined.
  } else { // = G94
    // - In units per mm mode: If F word passed, ensure value is in mm/min, otherwise push last state value.
    if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { // Last state is also G94
      if (bit_istrue(value_words,bit(WORD_F))) {
        if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
      } else {
        gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
      }
    } // Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
  } 
  // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking.
  
  // [4. Set spindle speed ]: S is negative (done.)
  if (bit_isfalse(value_words,bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
//这里应该是让这一路的PWM保持输出，就是输入其他指令时，PWM不变化的继续输出原来的值
#ifdef VARIABLE_SPINDLE_2
  if (bit_isfalse(value_words,bit(WORD_E))) { gc_block.values.s_2 = gc_state.spindle_speed_2; }
#endif
  //printInteger(value_words);//这句话可以输出调试信息，包括正负值，和最大32位的整数


  // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking.
    
  // [5. Select tool ]: NOT SUPPORTED. Only tracks value. T is negative (done.) Not an integer. Greater than max tool value.
  // bit_false(value_words,bit(WORD_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

  // [6. Change tool ]: N/A
  // [7. Spindle control ]: N/A
  // [8. Coolant control ]: N/A
  // [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED.
  
  // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.
  if (gc_block.non_modal_command == NON_MODAL_DWELL) {
    if (bit_isfalse(value_words,bit(WORD_P))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P word missing]
    bit_false(value_words,bit(WORD_P));
  }
  
  // [11. Set active plane ]: N/A
  switch (gc_block.modal.plane_select) {
    case PLANE_SELECT_XY:
      axis_0 = A_AXIS;
      axis_1 = B_AXIS;
      axis_linear = C_AXIS;
      break;
    case PLANE_SELECT_ZX:
      axis_0 = C_AXIS;
      axis_1 = A_AXIS;
      axis_linear = B_AXIS;
      break;
    default: // case PLANE_SELECT_YZ:
      axis_0 = B_AXIS;
      axis_1 = C_AXIS;
      axis_linear = A_AXIS;
  }   
            
  // [12. Set length units ]: N/A
  // Pre-convert XYZ coordinate values to millimeters, if applicable.
  uint8_t idx;
  if (gc_block.modal.units == UNITS_MODE_INCHES) {
    for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
      if (bit_istrue(axis_words,bit(idx)) ) {
        gc_block.values.xyz[idx] *= MM_PER_INCH;
      }
    }
  }
  
  // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
  // [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
  //   NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. Grbl supports G40 
  //   only for the purpose to not error when G40 is sent with a g-code program header to setup the default modes.
  
  // [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are. 
  // [G43.1 Errors]: Motion command in same line. 
  //   NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid 
  //   axis that is configured (in config.h). There should be an error if the configured axis
  //   is absent or if any of the other axis words are present.
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates called in block.
    if (gc_block.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) {
      if (axis_words ^ (1<<TOOL_LENGTH_OFFSET_AXIS)) { FAIL(STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR); }
    }
  }
  
  // [15. Coordinate system selection ]: *N/A. Error, if cutter radius comp is active.
  // TODO: An EEPROM read of the coordinate data may require a buffer sync when the cycle
  // is active. The read pauses the processor temporarily and may cause a rare crash. For 
  // future versions on processors with enough memory, all coordinate data should be stored
  // in memory and written to EEPROM only when there is not a cycle active.
  memcpy(coordinate_data,gc_state.coord_system,sizeof(gc_state.coord_system));
  if ( bit_istrue(command_words,bit(MODAL_GROUP_G12)) ) { // Check if called in block
    if (gc_block.modal.coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
    if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
      if (!(settings_read_coord_data(gc_block.modal.coord_select,coordinate_data))) { FAIL(STATUS_SETTING_READ_FAIL); } 
    }
  }
  
  // [16. Set path control mode ]: N/A. Only G61. G61.1 and G64 NOT SUPPORTED.
  // [17. Set distance mode ]: N/A. Only G91.1. G90.1 NOT SUPPORTED.
  // [18. Set retract mode ]: NOT SUPPORTED.
  
  // [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets.
  // NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
  // commands all treat axis words differently. G10 as absolute offsets or computes current position as
  // the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
  // all the current coordinate system and G92 offsets. 
  switch (gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:  
      // [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
      // [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to nCoordSys(max 9). Axis words missing.
      // [G10 L20 Errors]: P must be 0 to nCoordSys(max 9). Axis words missing.
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS) }; // [No axis words]
      if (bit_isfalse(value_words,((1<<WORD_P)|(1<<WORD_L)))) { FAIL(STATUS_GCODE_VALUE_WORD_MISSING); } // [P/L word missing]
      coord_select = trunc(gc_block.values.p); // Convert p value to int.
      if (coord_select > N_COORDINATE_SYSTEM) { FAIL(STATUS_GCODE_UNSUPPORTED_COORD_SYS); } // [Greater than N sys]
      if (gc_block.values.l != 20) {
        if (gc_block.values.l == 2) {
          if (bit_istrue(value_words,bit(WORD_R))) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G10 L2 R not supported]
        } else { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [Unsupported L]
      }
      bit_false(value_words,(bit(WORD_L)|bit(WORD_P)));
      
      // Determine coordinate system to change and try to load from EEPROM.
      if (coord_select > 0) { coord_select--; } // Adjust P1-P6 index to EEPROM coordinate data indexing.
      else { coord_select = gc_block.modal.coord_select; } // Index P0 as the active coordinate system
      if (!settings_read_coord_data(coord_select,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); } // [EEPROM read fail]
    
      // Pre-calculate the coordinate data changes. NOTE: Uses parameter_data since coordinate_data may be in use by G54-59.
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
        if (bit_istrue(axis_words,bit(idx)) ) {
          if (gc_block.values.l == 20) {
            // L20: Update coordinate system axis at current position (with modifiers) with programmed value
            parameter_data[idx] = gc_state.position[idx]-gc_state.coord_offset[idx]-gc_block.values.xyz[idx];
            if (idx == TOOL_LENGTH_OFFSET_AXIS) { parameter_data[idx] -= gc_state.tool_length_offset; }
          } else {
            // L2: Update coordinate system axis to programmed value.
            parameter_data[idx] = gc_block.values.xyz[idx]; 
          }
        }
      }
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      // [G92 Errors]: No axis words.
      if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
    
      // Update axes defined only in block. Offsets current system to defined value. Does not update when
      // active coordinate system is selected, but is still active unless G92.1 disables it. 
      for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
        if (bit_istrue(axis_words,bit(idx)) ) {
          gc_block.values.xyz[idx] = gc_state.position[idx]-coordinate_data[idx]-gc_block.values.xyz[idx];
          if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] -= gc_state.tool_length_offset; }
        } else {
          gc_block.values.xyz[idx] = gc_state.coord_offset[idx];
        }
      }
      break;
      
    default:

      // At this point, the rest of the explicit axis commands treat the axis values as the traditional
      // target position with the coordinate system offsets, G92 offsets, absolute override, and distance
      // modes applied. This includes the motion mode commands. We can now pre-compute the target position.
      // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
      if (axis_command != AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // TLO block any axis command.
        if (axis_words) {
          for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
            if ( bit_isfalse(axis_words,bit(idx)) ) {
				if(coordinate_mode == gc_state.coord_mode)
						gc_block.values.xyz[idx] = gc_state.position_Cartesian[idx];
					else
						gc_block.values.xyz[idx] = gc_state.position[idx]; // No axis word in block. Keep same axis position.
            } else {//这里根据轴字来判断，哪个轴的指令是有数的，则执行下边的else，没有数的则执行上边的把原来的值赋值给他！
              // Update specified value according to distance mode or ignore if absolute override is active.
              // NOTE: G53 is never active with G28/30 since they are in the same modal group.
              if (gc_block.non_modal_command != NON_MODAL_ABSOLUTE_OVERRIDE) {
                // Apply coordinate offsets based on distance mode.
                if (gc_block.modal.distance == DISTANCE_MODE_ABSOLUTE) {	
				//	printString("\r\n Absolute mode");
                  gc_block.values.xyz[idx] += coordinate_data[idx] + gc_state.coord_offset[idx];
                  if (idx == TOOL_LENGTH_OFFSET_AXIS) { gc_block.values.xyz[idx] += gc_state.tool_length_offset; }
                } else {  // Incremental mode
               	//	 printString("\r\n Incremental mode");
                  if(coordinate_mode == gc_state.coord_mode)
				  	{

					printString_debug("\r\n Debug: Command raw value:");
					printString_debug("\r\n");printInteger_debug(gc_block.values.xyz[idx]);printString_debug("\r\n");


					gc_block.values.xyz[idx] += gc_state.position_Cartesian[idx];


					printString_debug("\r\n Debug: Last coordinate value:");
					printString_debug("\r\n");printInteger_debug(gc_state.position_Cartesian[idx]);printString_debug("\r\n");

					printString_debug("\r\n Debug: Current coordinate value:");
					printString_debug("\r\n");printInteger_debug(gc_block.values.xyz[idx]);printString_debug("\r\n");
			
	   

					//这里，如果输入的指令有两个后边数字值，则进入循环两次，原来不明白为什么这里会输出多次，就是这个原因！	
				  }
				  else
				  {
				//  printString("\r\n Debug: here!!!!");
				  gc_block.values.xyz[idx] += gc_state.position[idx];

				  }
                  	
                }
              }
            }
          }
        }
      }
          
      // Check remaining non-modal commands for errors.
      switch (gc_block.non_modal_command) {        
        case NON_MODAL_GO_HOME_0: 
          // [G28 Errors]: Cutter compensation is enabled. 
          // Retreive G28 go-home position data (in machine coordinates) from EEPROM
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; } // Set to none if no intermediate motion.
          if (!settings_read_coord_data(SETTING_INDEX_G28,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_GO_HOME_1:
          // [G30 Errors]: Cutter compensation is enabled. 
          // Retreive G30 go-home position data (in machine coordinates) from EEPROM
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; } // Set to none if no intermediate motion.
          if (!settings_read_coord_data(SETTING_INDEX_G30,parameter_data)) { FAIL(STATUS_SETTING_READ_FAIL); }
          break;
        case NON_MODAL_SET_HOME_0: case NON_MODAL_SET_HOME_1:
          // [G28.1/30.1 Errors]: Cutter compensation is enabled. 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_RESET_COORDINATE_OFFSET: 
          // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
          break;
        case NON_MODAL_ABSOLUTE_OVERRIDE:
          // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
          // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
          if (!(gc_block.modal.motion == MOTION_MODE_SEEK || gc_block.modal.motion == MOTION_MODE_LINEAR)) {
            FAIL(STATUS_GCODE_G53_INVALID_MOTION_MODE); // [G53 G0/1 not active]
          }
          break;
      }
  }
      
  // [20. Motion modes ]: 
  if (gc_block.modal.motion == MOTION_MODE_NONE) {
    // [G80 Errors]: Axis word exist and are not used by a non-modal command.
    if ((axis_words) && (axis_command != AXIS_COMMAND_NON_MODAL)) { 
      FAIL(STATUS_GCODE_AXIS_WORDS_EXIST); // [No axis words allowed]
    }

  // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or 
  // was explicitly commanded in the g-code block.
  } else if ( axis_command == AXIS_COMMAND_MOTION_MODE ) {
  
    if (gc_block.modal.motion == MOTION_MODE_SEEK) {
      // [G0 Errors]: Axis letter not configured or without real value (done.)
      // Axis words are optional. If missing, set axis command flag to ignore execution.
      if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

    // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
    // the value must be positive. In inverse time mode, a positive value must be passed with each block.
    } else {      
      // Check if feed rate is defined for the motion modes that require it.
      if (gc_block.values.f == 0.0) { FAIL(STATUS_GCODE_UNDEFINED_FEED_RATE); } // [Feed rate undefined]
     
      switch (gc_block.modal.motion) {
        case MOTION_MODE_LINEAR: 
          // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
          // Axis words are optional. If missing, set axis command flag to ignore execution.
          if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

          break;
        case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
          // [G2/3 Errors All-Modes]: Feed rate undefined.
          // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
          // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current 
          //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).   
          // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.        
          // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.
        
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (!(axis_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE); } // [No axis words in plane]
        
          // Calculate the change in position along each selected axis
          float x,y;
          x = gc_block.values.xyz[axis_0]-gc_state.position[axis_0]; // Delta x between current position and target
          y = gc_block.values.xyz[axis_1]-gc_state.position[axis_1]; // Delta y between current position and target

          if (value_words & bit(WORD_R)) { // Arc Radius Mode  
            bit_false(value_words,bit(WORD_R));
            if (gc_check_same_position(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          
            // Convert radius value to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
            /*  We need to calculate the center of the circle that has the designated radius and passes
                through both the current position and the target position. This method calculates the following
                set of equations where [x,y] is the vector from current to target position, d == magnitude of 
                that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the 
                length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point 
                [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.
    
                d^2 == x^2 + y^2
                h^2 == r^2 - (d/2)^2
                i == x/2 - y/d*h
                j == y/2 + x/d*h
    
                                                                     O <- [i,j]
                                                                  -  |
                                                        r      -     |
                                                            -        |
                                                         -           | h
                                                      -              |
                                        [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                  | <------ d/2 ---->|
              
                C - Current position
                T - Target position
                O - center of circle that pass through both C and T
                d - distance from C to T
                r - designated radius
                h - distance from center of CT to O
    
                Expanding the equations:
 
                d -> sqrt(x^2 + y^2)
                h -> sqrt(4 * r^2 - x^2 - y^2)/2
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2 
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
   
                Which can be written:
    
                i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
    
                Which we for size and speed reasons optimize to:
 
                h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                i = (x - (y * h_x2_div_d))/2
                j = (y + (x * h_x2_div_d))/2       
            */      

            // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
            // than d. If so, the sqrt of a negative number is complex and error out.
            float h_x2_div_d = 4.0 * gc_block.values.r*gc_block.values.r - x*x - y*y;

            if (h_x2_div_d < 0) { FAIL(STATUS_GCODE_ARC_RADIUS_ERROR); } // [Arc radius error]
    
            // Finish computing h_x2_div_d.
            h_x2_div_d = -sqrt(h_x2_div_d)/hypot_f(x,y); // == -(h * 2 / d)
            // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
            if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }  

            /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
               the left hand circle will be generated - when it is negative the right hand circle is generated.
          
                                                                   T  <-- Target position
                                                   
                                                                   ^ 
                        Clockwise circles with this center         |          Clockwise circles with this center will have
                        will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                         \         |          /   
            center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                   |
                                                                   |
                                                   
                                                                   C  <-- Current position                                
            */  
            // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!), 
            // even though it is advised against ever generating such circles in a single line of g-code. By 
            // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
            // travel and thus we get the unadvisably long arcs as prescribed.
            if (gc_block.values.r < 0) { 
                h_x2_div_d = -h_x2_div_d; 
                gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
            }        
            // Complete the operation by calculating the actual center of the arc
            gc_block.values.ijk[axis_0] = 0.5*(x-(y*h_x2_div_d));
            gc_block.values.ijk[axis_1] = 0.5*(y+(x*h_x2_div_d));
          
          } else { // Arc Center Format Offset Mode  
            if (!(ijk_words & (bit(axis_0)|bit(axis_1)))) { FAIL(STATUS_GCODE_NO_OFFSETS_IN_PLANE); } // [No offsets in plane]
            bit_false(value_words,(bit(WORD_I)|bit(WORD_J)|bit(WORD_K)));  
          
            // Convert IJK values to proper units.
            if (gc_block.modal.units == UNITS_MODE_INCHES) {
              for (idx=0; idx<N_AXIS; idx++) { // Axes indices are consistent, so loop may be used to save flash space.
                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
              }
            }         

            // Arc radius from center to target
            x -= gc_block.values.ijk[axis_0]; // Delta x between circle center and target
            y -= gc_block.values.ijk[axis_1]; // Delta y between circle center and target
            float target_r = hypot_f(x,y); 

            // Compute arc radius for mc_arc. Defined from current location to center.
            gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]); 
            
            // Compute difference between current location and target radii for final error-checks.
            float delta_r = fabs(target_r-gc_block.values.r);
            if (delta_r > 0.005) { 
              if (delta_r > 0.5) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.5mm
              if (delta_r > (0.001*gc_block.values.r)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Arc definition error] > 0.005mm AND 0.1% radius
            }
          }
          break;
        case MOTION_MODE_PROBE_TOWARD: case MOTION_MODE_PROBE_TOWARD_NO_ERROR:
        case MOTION_MODE_PROBE_AWAY: case MOTION_MODE_PROBE_AWAY_NO_ERROR:
          // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
          //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
          //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to 
          //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.
          if (!axis_words) { FAIL(STATUS_GCODE_NO_AXIS_WORDS); } // [No axis words]
          if (gc_check_same_position(gc_state.position, gc_block.values.xyz)) { FAIL(STATUS_GCODE_INVALID_TARGET); } // [Invalid target]
          break;
      } 
    }
  }
  
  // [21. Program flow ]: No error checks required.

  // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
  // radius mode, or axis words that aren't used in the block.  
  bit_false(value_words,(bit(WORD_N)|bit(WORD_F)|bit(WORD_S)|bit(WORD_T)|bit(WORD_E))); // Remove single-meaning value words. 
  if (axis_command) { bit_false(value_words,(bit(WORD_A)|bit(WORD_B)|bit(WORD_C)|bit(WORD_D)|bit(WORD_X)|bit(WORD_Y)|bit(WORD_Z))); } // Remove axis words. 
  if (value_words) { FAIL(STATUS_GCODE_UNUSED_WORDS); } // [Unused words]

   
  /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
  */ 
  
  // [0. Non-specific/common error-checks and miscellaneous setup]: 
  gc_state.line_number = gc_block.values.n;
  
  // [1. Comments feedback ]:  NOT SUPPORTED
  
  // [2. Set feed rate mode ]:
  gc_state.modal.feed_rate = gc_block.modal.feed_rate;
  
  // [3. Set feed rate ]:
  gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.

  // [4. Set spindle speed ]:
  if (gc_state.spindle_speed != gc_block.values.s) { 
    // Update running spindle only if not in check mode and not already enabled.
    if (gc_state.modal.spindle != SPINDLE_DISABLE) 
		{ 
			spindle_run(gc_state.modal.spindle, gc_block.values.s);
	    }
    gc_state.spindle_speed = gc_block.values.s; 
  }

#ifdef VARIABLE_SPINDLE_2
  if (gc_state.spindle_speed_2 != gc_block.values.s_2) { 
    // Update running spindle only if not in check mode and not already enabled.
    if (gc_state.modal.spindle != SPINDLE_DISABLE) 
		{ 
			spindle_run_2(gc_state.modal.spindle, gc_block.values.s_2);
	    }
    gc_state.spindle_speed_2 = gc_block.values.s_2; 
  }
#endif 
    
  // [5. Select tool ]: NOT SUPPORTED. Only tracks tool value.
  gc_state.tool = gc_block.values.t;

  // [6. Change tool ]: NOT SUPPORTED
  
#ifndef VARIABLE_SPINDLE_2
  // [7. Spindle control ]:
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
    // Update spindle control and apply spindle speed when enabling it in this block.    
    spindle_run(gc_block.modal.spindle, gc_state.spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;    
  }
#endif

#ifdef VARIABLE_SPINDLE_2
  if (gc_state.modal.spindle != gc_block.modal.spindle) {
    // Update spindle control and apply spindle speed when enabling it in this block.  
   if(gc_block.modal.spindle == SPINDLE_ENABLE_CW)//M3,pwm1输出
    {spindle_run(gc_block.modal.spindle, gc_state.spindle_speed);
    gc_state.modal.spindle = gc_block.modal.spindle;  } 
   if(gc_block.modal.spindle == SPINDLE_ENABLE_CCW)
   	{spindle_run_2(gc_block.modal.spindle, gc_state.spindle_speed_2);
    gc_state.modal.spindle = gc_block.modal.spindle;  } 
  }
#endif
  

  // [8. Coolant control ]:  
  if (gc_state.modal.coolant != gc_block.modal.coolant) {
    coolant_run(gc_block.modal.coolant);
    gc_state.modal.coolant = gc_block.modal.coolant;
  }
  
  // [9. Enable/disable feed rate or spindle overrides ]: NOT SUPPORTED

  // [10. Dwell ]:
  if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }
  
  // [11. Set active plane ]:
  gc_state.modal.plane_select = gc_block.modal.plane_select;  

  // [12. Set length units ]:
  gc_state.modal.units = gc_block.modal.units;

  // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
  // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.

  // [14. Cutter length compensation ]: G43.1 and G49 supported. G43 NOT SUPPORTED.
  // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
  // of execution. The error-checking step would simply load the offset value into the correct
  // axis of the block XYZ value array. 
  if (axis_command == AXIS_COMMAND_TOOL_LENGTH_OFFSET ) { // Indicates a change.
    gc_state.modal.tool_length = gc_block.modal.tool_length;
    if (gc_state.modal.tool_length == TOOL_LENGTH_OFFSET_ENABLE_DYNAMIC) { // G43.1
      gc_state.tool_length_offset = gc_block.values.xyz[TOOL_LENGTH_OFFSET_AXIS];
    } else { // G49
      gc_state.tool_length_offset = 0.0;
    }
  }
  
  // [15. Coordinate system selection ]:
  if (gc_state.modal.coord_select != gc_block.modal.coord_select) {
    gc_state.modal.coord_select = gc_block.modal.coord_select;
    memcpy(gc_state.coord_system,coordinate_data,sizeof(coordinate_data));
  }
  
  // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
  // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.
  
  // [17. Set distance mode ]:
  gc_state.modal.distance = gc_block.modal.distance;
  
  // [18. Set retract mode ]: NOT SUPPORTED
    
  // [19. Go to predefined position, Set G10, or Set axis offsets ]:
  switch(gc_block.non_modal_command) {
    case NON_MODAL_SET_COORDINATE_DATA:    
      settings_write_coord_data(coord_select,parameter_data);
      // Update system coordinate system if currently active.
      if (gc_state.modal.coord_select == coord_select) { memcpy(gc_state.coord_system,parameter_data,sizeof(parameter_data)); }
      break;
    case NON_MODAL_GO_HOME_0: case NON_MODAL_GO_HOME_1: 
      // Move to intermediate position before going home. Obeys current coordinate system and offsets 
      // and absolute and incremental modes.
      if (axis_command) {
        #ifdef USE_LINE_NUMBERS
          mc_line(gc_block.values.xyz, -1.0, false, gc_state.line_number);
        #else
          mc_line(gc_block.values.xyz, -1.0, false, false);
        #endif
      }
      #ifdef USE_LINE_NUMBERS
        mc_line(parameter_data, -1.0, false, gc_state.line_number); 
      #else
        mc_line(parameter_data, -1.0, false, false); 
      #endif
      memcpy(gc_state.position, parameter_data, sizeof(parameter_data));
      break;
    case NON_MODAL_SET_HOME_0: 
      settings_write_coord_data(SETTING_INDEX_G28,gc_state.position);
      break;
    case NON_MODAL_SET_HOME_1:
      settings_write_coord_data(SETTING_INDEX_G30,gc_state.position);
      break;
    case NON_MODAL_SET_COORDINATE_OFFSET:
      memcpy(gc_state.coord_offset,gc_block.values.xyz,sizeof(gc_block.values.xyz));
      break;
    case NON_MODAL_RESET_COORDINATE_OFFSET: 
      clear_vector(gc_state.coord_offset); // Disable G92 offsets by zeroing offset vector.
      break;
  }

  
  // [20. Motion modes ]:
  // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes. 
  // Enter motion modes only if there are axis words or a motion mode command word in the block.
  gc_state.modal.motion = gc_block.modal.motion;
  if (gc_state.modal.motion != MOTION_MODE_NONE) {
    if (axis_command == AXIS_COMMAND_MOTION_MODE) {
      switch (gc_state.modal.motion) {
        case MOTION_MODE_SEEK:
	#ifdef debug
			printString("\r\nMOTION_MODE_SEEK mode.");
			printString("\r\nFinally passed the instruction value of plan_buffer_line:");

			printString("\r\ngc_block.values.xyz[A]:");printInteger(gc_block.values.xyz[A_AXIS]);
			printString("\r\ngc_block.values.xyz[B]:");printInteger(gc_block.values.xyz[B_AXIS]);
			printString("\r\ngc_block.values.xyz[C]:");printInteger(gc_block.values.xyz[C_AXIS]);
			printString("\r\ngc_block.values.xyz[D]:");printInteger(gc_block.values.xyz[D_AXIS]);
			printString("\r\ngc_block.values.xyz[E]:");printInteger(gc_block.values.xyz[E_AXIS]);
			printString("\r\ngc_block.values.xyz[F]:");printInteger(gc_block.values.xyz[F_AXIS]);
			printString("\r\ngc_block.values.xyz[G]:");printInteger(gc_block.values.xyz[G_AXIS]);
		
	#endif
			if(gc_state.coord_mode == coordinate_mode)
				{	
			
					printString_debug("\r\nin case coordinate_mode");

					float difference[N_Cartesian];
					float current_position[N_Cartesian];
					
					for(int i = 0;i< N_Cartesian;i++ )
						{
						current_position[i] = sys.position_Cartesian[i];
						}
					
					//用来储存目标坐标和当前坐标之间的距离
					difference[X_Cartesian] = gc_block.values.xyz[E_AXIS] - current_position[X_Cartesian];
					difference[Y_Cartesian] = gc_block.values.xyz[F_AXIS] - current_position[Y_Cartesian];
					difference[Z_Cartesian] = gc_block.values.xyz[G_AXIS] - current_position[Z_Cartesian];
					difference[RX_Cartesian] = gc_block.values.xyz[A_AXIS] - current_position[RX_Cartesian];
					difference[RY_Cartesian] = gc_block.values.xyz[B_AXIS] - current_position[RY_Cartesian];
					difference[RZ_Cartesian] = gc_block.values.xyz[C_AXIS] - current_position[RZ_Cartesian];
					//TODO:增加姿态的插值。暂时不加，姿态维持不变，只位置插值。
					printString_debug("\r\ndifference[X_Cartesian]:");printInteger_debug(difference[X_Cartesian]);
					printString_debug("\r\ndifference[Y_Cartesian]:");printInteger_debug(difference[Y_Cartesian]);
					printString_debug("\r\ndifference[Z_Cartesian]:");printInteger_debug(difference[Z_Cartesian]);
					
					//把本次的笛卡尔值和姿态值保存起来
					//TODO：sys.position_cartesian应该初始化为0
					sys.position_Cartesian[X_Cartesian] = gc_block.values.xyz[E_AXIS];
					sys.position_Cartesian[Y_Cartesian] = gc_block.values.xyz[F_AXIS];
					sys.position_Cartesian[Z_Cartesian] = gc_block.values.xyz[G_AXIS];
					sys.position_Cartesian[RX_Cartesian] = gc_block.values.xyz[A_AXIS];
					sys.position_Cartesian[RY_Cartesian] = gc_block.values.xyz[B_AXIS];
					sys.position_Cartesian[RZ_Cartesian] = gc_block.values.xyz[C_AXIS];
					
					float cartesian_mm = sqrt(	pow(difference[X_Cartesian],2) +  //sq平方函数，在笛卡尔坐标模式下，求运动始末点的距离（mm）
									pow(difference[Y_Cartesian],2) +
									pow(difference[Z_Cartesian],2));
					//if (cartesian_mm < 0.000001) { break; }//注释这里就是因为如果cartesian_mm为0，则只改变姿态不运动！！！
					printString_debug("\r\ncartesian_mm:");printInteger_debug(cartesian_mm);
					float seconds = settings.robot_qinnew.interpolation_num * cartesian_mm / gc_state.feed_rate;//60表示插补精细程度
					printString_debug("\r\nseconds:");printInteger_debug(seconds);
					int steps = 1;
					if(settings.robot_qinnew.use_interpolation)
						{
						steps = max(1, (int)(25 * seconds));
						}
					printString_debug("\r\nsteps:");printInteger_debug(steps);
					float destination[N_Cartesian];
					

#if 0
					static bool axis_Directionflag_last = 0;
					bool axis_Directionflag;
					int32_t target_steps_temp;
					float temp[7] ={0,0,0,0,0,0,0};//用于补偿的
#endif

					for (int s = 1; s <= steps; s++) 
						{//这个大循环，应该是个在轨迹当中插值的过程！
						float fraction = (float)s / (float)steps;//计算比率
							//for(int8_t i=0; i < NUM_AXIS; i++) {
							//		destination[i] = current_position[i] + difference[i] * fraction;
							//			}
							destination[X_Cartesian] = current_position[X_Cartesian] + difference[X_Cartesian] * fraction;
							destination[Y_Cartesian] = current_position[Y_Cartesian] + difference[Y_Cartesian] * fraction;
							destination[Z_Cartesian] = current_position[Z_Cartesian] + difference[Z_Cartesian] * fraction;
							destination[RX_Cartesian] = current_position[RX_Cartesian] + difference[RX_Cartesian] * fraction;
							destination[RY_Cartesian] = current_position[RY_Cartesian] + difference[RY_Cartesian] * fraction;
							destination[RZ_Cartesian] = current_position[RZ_Cartesian] + difference[RZ_Cartesian] * fraction;
							
							InverseInit();
							//Inverse(destination[X_Cartesian],destination[Y_Cartesian],destination[Z_Cartesian],gc_block.values.xyz[A_AXIS],gc_block.values.xyz[B_AXIS],gc_block.values.xyz[C_AXIS]);
							Inverse(destination[X_Cartesian],destination[Y_Cartesian],destination[Z_Cartesian],destination[RX_Cartesian],destination[RY_Cartesian],destination[RZ_Cartesian]);

#if 0
							//x轴补偿
							memcpy(temp, gc_state.position, sizeof(gc_state.position));
							target_steps_temp = lround(temp[E_AXIS]*settings.steps_per_mm[E_AXIS]);
						if((target_steps_temp - get_pl(E_AXIS)) != 0)
							{
							if((target_steps_temp - get_pl(E_AXIS)) > 0)
								{
									axis_Directionflag = false;
								}
							if((target_steps_temp - get_pl(E_AXIS)) < 0)
								{
									axis_Directionflag = true;
								}
							//	printString("\r\naxis_Directionflag:\r\n");
							//	printInteger(axis_Directionflag);
							//	printString("\r\target_steps_temp - get_pl(E_AXIS):\r\n");
							//	printInteger(target_steps_temp - get_pl(E_AXIS));

							if(axis_Directionflag != axis_Directionflag_last)
								{
								
								if(axis_Directionflag == false)
									{temp[4] +=settings.robot_qinnew.compensation_num;
								//	printString("\r\nin forward compensation\r\n");
									}
								else
									{temp[4] -=settings.robot_qinnew.compensation_num;
								//	printString("\r\nin inverse compensation\r\n");
									}
								
								mc_line(temp, -1.0, false, true);
								}
							 axis_Directionflag_last = axis_Directionflag; 
							}
	#endif
							gc_state.position[D_AXIS] = gc_block.values.xyz[D_AXIS];//增加笛卡尔对D轴的支持
							mc_line(gc_state.position, -1.0, false, false);
							
						}
							
							break;

					
					
					//printString("\r\n");
					//printString("theta 1:");
					//printFloat(gc_block.values.xyz[E_AXIS],2);
					//printString("\r\n");
					//printString("theta 2:");
					//printFloat(gc_block.values.xyz[F_AXIS],2);
					//printString("\r\n");
					//printString("theta 3:");
					//printFloat(gc_block.values.xyz[G_AXIS],2);
					//printString("\r\n");
					//printString("theta 4:");
					//printFloat(gc_block.values.xyz[A_AXIS],2);
					//printString("\r\n");
					//printString("theta 5:");
					//printFloat(gc_block.values.xyz[B_AXIS],2);
					//printString("\r\n");
					//printString("theta 6:");
					//printFloat(gc_block.values.xyz[C_AXIS],2);
				//	printString("\r\n");
				}	
          #ifdef USE_LINE_NUMBERS
            mc_line(gc_block.values.xyz, -1.0, false, gc_state.line_number);
          #else
		/*  printString("\r\n");
					printString("a:");
					printFloat(gc_block.values.xyz[A_AXIS],2);
					printString("\r\n");
					printString("b:");
					printFloat(gc_block.values.xyz[B_AXIS],2);
					printString("\r\n");
					printString("c:");
					printFloat(gc_block.values.xyz[C_AXIS],2);
					printString("\r\n");
					printString("x:");
					printFloat(gc_block.values.xyz[E_AXIS],2);
					printString("\r\n");
					printString("y:");
					printFloat(gc_block.values.xyz[F_AXIS],2);
					printString("\r\n");
					printString("z:");
					printFloat(gc_block.values.xyz[G_AXIS],2);*/
            mc_line(gc_block.values.xyz, -1.0, false ,false);
          #endif
          break;
        case MOTION_MODE_LINEAR:

			printString_debug("in case MOTION_MODE_LINEAR\r\n");

			if(gc_state.coord_mode == coordinate_mode)
				{	
					printString_debug("in case coordinate_mode\r\n");
					float difference[N_Cartesian];
					float current_position[6];
					
					for(int i = 0;i< N_Cartesian;i++ )
						{
						current_position[i] = sys.position_Cartesian[i];
						}
					
					//用来储存目标坐标和当前坐标之间的距离
					difference[X_Cartesian] = gc_block.values.xyz[E_AXIS] - current_position[X_Cartesian];
					difference[Y_Cartesian] = gc_block.values.xyz[F_AXIS] - current_position[Y_Cartesian];
					difference[Z_Cartesian] = gc_block.values.xyz[G_AXIS] - current_position[Z_Cartesian];
					difference[RX_Cartesian] = gc_block.values.xyz[A_AXIS] - current_position[RX_Cartesian];
					difference[RY_Cartesian] = gc_block.values.xyz[B_AXIS] - current_position[RY_Cartesian];
					difference[RZ_Cartesian] = gc_block.values.xyz[C_AXIS] - current_position[RZ_Cartesian];
					//TODO:增加姿态的插值。暂时不加，姿态维持不变，只位置插值。
					printString_debug("difference[X_Cartesian]:");printInteger_debug(difference[X_Cartesian]);
					printString_debug("difference[Y_Cartesian]:");printInteger_debug(difference[Y_Cartesian]);
					printString_debug("difference[Z_Cartesian]:");printInteger_debug(difference[Z_Cartesian]);
					
					//把本次的笛卡尔值和姿态值保存起来
					//TODO：sys.position_cartesian应该初始化为0
					sys.position_Cartesian[X_Cartesian] = gc_block.values.xyz[E_AXIS];
					sys.position_Cartesian[Y_Cartesian] = gc_block.values.xyz[F_AXIS];
					sys.position_Cartesian[Z_Cartesian] = gc_block.values.xyz[G_AXIS];
					sys.position_Cartesian[RX_Cartesian] = gc_block.values.xyz[A_AXIS];
					sys.position_Cartesian[RY_Cartesian] = gc_block.values.xyz[B_AXIS];
					sys.position_Cartesian[RZ_Cartesian] = gc_block.values.xyz[C_AXIS];
					
					float cartesian_mm = sqrt(	pow(difference[X_Cartesian],2) +  //sq平方函数，在笛卡尔坐标模式下，求运动始末点的距离（mm）
									pow(difference[Y_Cartesian],2) +
									pow(difference[Z_Cartesian],2));
				//	if (cartesian_mm < 0.000001) { break; }
					printString_debug("\r\ncartesian_mm:");printInteger_debug(cartesian_mm);
					float seconds = settings.robot_qinnew.interpolation_num * cartesian_mm / gc_state.feed_rate;
					printString_debug("\r\nseconds:");printInteger_debug(seconds);
					int steps = 1;
					//	printString_debug("\r\nuse_interpolation:");printInteger_debug(settings.robot_qinnew.use_interpolation);
					if(settings.robot_qinnew.use_interpolation)
						{
						steps = max(1, (int)(25 * seconds));
						}
					printString_debug("\r\nsteps:");printInteger_debug(steps);
					float destination[N_Cartesian];
					

#if 1
					static bool axis_Directionflag_last = 0;
					bool axis_Directionflag;
					int32_t target_steps_temp;
					float temp[7] ={0,0,0,0,0,0,0};//用于补偿的
#endif

					for (int s = 1; s <= steps; s++) 
						{//这个大循环，应该是个在轨迹当中插值的过程！
						float fraction = (float)s / (float)steps;//计算比率
							//for(int8_t i=0; i < NUM_AXIS; i++) {
							//		destination[i] = current_position[i] + difference[i] * fraction;
							//			}
							destination[X_Cartesian] = current_position[X_Cartesian] + difference[X_Cartesian] * fraction;
							destination[Y_Cartesian] = current_position[Y_Cartesian] + difference[Y_Cartesian] * fraction;
							destination[Z_Cartesian] = current_position[Z_Cartesian] + difference[Z_Cartesian] * fraction;
							destination[RX_Cartesian] = current_position[RX_Cartesian] + difference[RX_Cartesian] * fraction;
							destination[RY_Cartesian] = current_position[RY_Cartesian] + difference[RY_Cartesian] * fraction;
							destination[RZ_Cartesian] = current_position[RZ_Cartesian] + difference[RZ_Cartesian] * fraction;
							
							InverseInit();
							Inverse(destination[X_Cartesian],destination[Y_Cartesian],destination[Z_Cartesian],destination[RX_Cartesian],destination[RY_Cartesian],destination[RZ_Cartesian]);

#if 1
                  if(1 == settings.robot_qinnew.use_compensation)
                  	{
							//x轴补偿
							memcpy(temp, gc_state.position, sizeof(gc_state.position));
							target_steps_temp = lround(temp[E_AXIS]*settings.steps_per_mm[E_AXIS]);
						if((target_steps_temp - get_pl(E_AXIS)) != 0)
							{
							if((target_steps_temp - get_pl(E_AXIS)) > 0)
								{
									axis_Directionflag = false;
								}
							if((target_steps_temp - get_pl(E_AXIS)) < 0)
								{
									axis_Directionflag = true;
								}
							//	printString("\r\naxis_Directionflag:\r\n");
							//	printInteger(axis_Directionflag);
							//	printString("\r\target_steps_temp - get_pl(E_AXIS):\r\n");
							//	printInteger(target_steps_temp - get_pl(E_AXIS));

							if(axis_Directionflag != axis_Directionflag_last)
								{
								
								if(axis_Directionflag == false)//E轴向正方向运动
									{temp[4] +=settings.robot_qinnew.compensation_num;
								//	printString("\r\nin forward compensation\r\n");
									}
								else//E轴向负方向运动
									{temp[4] -=settings.robot_qinnew.compensation_num;
								//	printString("\r\nin inverse compensation\r\n");
									}
								
								mc_line(temp, gc_state.feed_rate, gc_state.modal.feed_rate, true);
								}
							 axis_Directionflag_last = axis_Directionflag; 
							}
                  	}
	#endif
							gc_state.position[D_AXIS] = gc_block.values.xyz[D_AXIS];//增加笛卡尔对D轴的支持
							mc_line(gc_state.position, gc_state.feed_rate, gc_state.modal.feed_rate, false);
							
						}
							break;

					
					
					//printString("\r\n");
					//printString("theta 1:");
					//printFloat(gc_block.values.xyz[E_AXIS],2);
					//printString("\r\n");
					//printString("theta 2:");
					//printFloat(gc_block.values.xyz[F_AXIS],2);
					//printString("\r\n");
					//printString("theta 3:");
					//printFloat(gc_block.values.xyz[G_AXIS],2);
					//printString("\r\n");
					//printString("theta 4:");
					//printFloat(gc_block.values.xyz[A_AXIS],2);
					//printString("\r\n");
					//printString("theta 5:");
					//printFloat(gc_block.values.xyz[B_AXIS],2);
					//printString("\r\n");
					//printString("theta 6:");
					//printFloat(gc_block.values.xyz[C_AXIS],2);
				//	printString("\r\n");
				}	
          #ifdef USE_LINE_NUMBERS
            mc_line(gc_block.values.xyz, -1.0, false, gc_state.line_number);
          #else
		/*  printString("\r\n");
					printString("a:");
					printFloat(gc_block.values.xyz[A_AXIS],2);
					printString("\r\n");
					printString("b:");
					printFloat(gc_block.values.xyz[B_AXIS],2);
					printString("\r\n");
					printString("c:");
					printFloat(gc_block.values.xyz[C_AXIS],2);
					printString("\r\n");
					printString("x:");
					printFloat(gc_block.values.xyz[E_AXIS],2);
					printString("\r\n");
					printString("y:");
					printFloat(gc_block.values.xyz[F_AXIS],2);
					printString("\r\n");
					printString("z:");
					printFloat(gc_block.values.xyz[G_AXIS],2);*/
            mc_line(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false);
          #endif
          break;
        case MOTION_MODE_CW_ARC: 
			printString("in case MOTION_MODE_CW_ARC\r\n");
          #ifdef USE_LINE_NUMBERS
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, true, gc_state.line_number);  
          #else
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, true); 
          #endif
          break;        
        case MOTION_MODE_CCW_ARC:
          #ifdef USE_LINE_NUMBERS
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, false, gc_state.line_number);  
          #else
            mc_arc(gc_state.position, gc_block.values.xyz, gc_block.values.ijk, gc_block.values.r, 
              gc_state.feed_rate, gc_state.modal.feed_rate, axis_0, axis_1, axis_linear, false); 
          #endif
          break;
        case MOTION_MODE_PROBE_TOWARD: 
          // NOTE: gc_block.values.xyz is returned from mc_probe_cycle with the updated position value. So
          // upon a successful probing cycle, the machine position and the returned value should be the same.
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, false, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, false);
          #endif
          break;
        case MOTION_MODE_PROBE_TOWARD_NO_ERROR:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, true, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, false, true);
          #endif
          break;
        case MOTION_MODE_PROBE_AWAY:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, false, gc_state.line_number);
          #else
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, false);
          #endif
          break;
        case MOTION_MODE_PROBE_AWAY_NO_ERROR:
          #ifdef USE_LINE_NUMBERS
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, true, gc_state.line_number);
          #else        
            mc_probe_cycle(gc_block.values.xyz, gc_state.feed_rate, gc_state.modal.feed_rate, true, true);
          #endif
      }
    
      // As far as the parser is concerned, the position is now == target. In reality the
      // motion control system might still be processing the action and the real tool position
      // in any intermediate location.
      if(coordinate_mode == gc_state.coord_mode )//笛卡尔模式
	  	//memcpy(gc_state.position_Cartesian, gc_block.values.xyz, sizeof(gc_block.values.xyz));
		//笛卡尔模式gc_block.values.xyz会在逆解函数中被变成角度!!!!!
	  memcpy(gc_state.position_Cartesian, gc_block.values.xyz, sizeof(gc_block.values.xyz));
	  else//角度模式
      memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
      
    }
  }
  
  // [21. Program flow ]:
  // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may 
  // refill and can only be resumed by the cycle start run-time command.
  gc_state.modal.program_flow = gc_block.modal.program_flow;
  if (gc_state.modal.program_flow) { 
	protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
	if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
	  if (sys.state != STATE_CHECK_MODE) {
		bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); // Use feed hold for program pause.
		protocol_execute_realtime(); // Execute suspend.
	  }
	} else { // == PROGRAM_FLOW_COMPLETED
	  // Upon program complete, only a subset of g-codes reset to certain defaults, according to 
	  // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
	  // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
	  // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.
	  gc_state.modal.motion = MOTION_MODE_LINEAR;
	  gc_state.modal.plane_select = PLANE_SELECT_XY;
	  gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
	  gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
	  // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
	  gc_state.modal.coord_select = 0; // G54
	  gc_state.modal.spindle = SPINDLE_DISABLE;
	  gc_state.modal.coolant = COOLANT_DISABLE;
	  // gc_state.modal.override = OVERRIDE_DISABLE; // Not supported.
	  
	  // Execute coordinate change and spindle/coolant stop.
	  if (sys.state != STATE_CHECK_MODE) {
		if (!(settings_read_coord_data(gc_state.modal.coord_select,coordinate_data))) { FAIL(STATUS_SETTING_READ_FAIL); } 
		memcpy(gc_state.coord_system,coordinate_data,sizeof(coordinate_data));
		spindle_stop();
		#ifdef VARIABLE_SPINDLE_2
			spindle_stop_2();
		 #endif
		coolant_stop();		
	  }
	  
	  report_feedback_message(MESSAGE_PROGRAM_END);
	}
    gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // Reset program flow.
  }
    
  // TODO: % to denote start of program.
  return(STATUS_OK);
}
        

/* 
  Not supported:

  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Evaluation of expressions
  - Variables
  - Override control (TBD)
  - Tool changes
  - Switches
   
   (*) Indicates optional parameter, enabled through config.h and re-compile
   group 0 = {G92.2, G92.3} (Non modal: Cancel and re-enable G92 offsets)
   group 1 = {G81 - G89} (Motion modes: Canned cycles)
   group 4 = {M1} (Optional stop, ignored)
   group 6 = {M6} (Tool change)
   group 7 = {G41, G42} cutter radius compensation (G40 is supported)
   group 8 = {G43} tool length offset (G43.1/G49 are supported)
   group 8 = {*M7} enable mist coolant (* Compile-option)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 10 = {G98, G99} return mode canned cycles
   group 13 = {G61.1, G64} path control mode (G61 is supported)
*/
