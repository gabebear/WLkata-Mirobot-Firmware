修改了
1、增加了一路PWM输出，原来的PWM输出是在数字脚7，使用指令M3 S
     新增加的在数字脚2，使用指令M4 E
2、指令增加了E关键字，但是他里边的word_bit最多有16位，而且都用了，所以把原来的关键字T修改成了E，给E关键字来使用

3\20190403  增加了运动学正解，两种模式的切换，和两种模式的增量模式

4、2019.4.8这一版修改了引脚定义，与赵家与最新的电路图匹配

5、20190411 上一版在笛卡尔模式单独动姿态他不运动，原因是在插补中有//if (cartesian_mm < 0.000001) { break; 
}
也就是说在XYZ坐标位置不动的情况下，这里直接跳出了，不运动了
现在注释掉了

6、修改了默认参数 

7、20190412改变了逆解函数中各个角度的限位数值，增加了当限位超过限位数值时的处理

8、20190418修改了L的缺少负号的bug,笛卡尔初值加入了L的值影响

9、20190419对机械臂参数设置进行大改：把原来宏定义的参数设置都变成了指令，增加到$29以后，增加了对于是否启用插补，以及插补精度的设置，但是设置是否启用补偿没有使用，这是因为宏定义变成参数有点问题

10、20190422对逆解函数稍作修改，主要是在笛卡尔目标点超过外围最大位置时，显示无解直接return

10、20190422_2发现上一版G01姿态指令不执行，修改

11、20190423 把在位置不变时只改变姿态不动的影响：//	if (cartesian_mm < 0.000001) { break; }注释掉
把G0中的插补部分设置为永远不编译：#if 0

12、20190424 修改了复位回弹距离，单独把第五轴改成其他轴设置值的三倍！

13、20190425 
修改增加了串口2的函数，并且串口1和串口2都可以接受指令，并且下位机回文同时回复串口1和2

14、20190510
返回的信息在每次运动结束自动返回一次，并且加入了气泵和阀状态，以及运动状态笛卡尔或者角度。另外修改了增加了调试输出函数，但是没有全改

15、grbl_6v20190611
插补有一个错误，int step,修改了的

16、20190716
void angle_to_coordinate()
{
	double temp[N_AXIS];
	system_convert_array_steps_to_mpos(temp,sys.position);
	Forward(temp);//不论何时，sys.position中保存的都是角度的绝对位置值，但是是脉冲表示的！！！
	//执行完以后，得到的结果保存在sys.position_Cartesian中
	gc_state.position_Cartesian[E_AXIS] = sys.position_Cartesian[X_Cartesian];
	gc_state.position_Cartesian[F_AXIS] = sys.position_Cartesian[Y_Cartesian];
	gc_state.position_Cartesian[G_AXIS] = sys.position_Cartesian[Z_Cartesian];
	printString_debug("\r\gc_state.position_Cartesian[G_AXIS]:");
	printFloat(gc_state.position_Cartesian[G_AXIS],2);

	//增加对传动带D轴的赋值
	gc_state.position_Cartesian[D_AXIS] = sys.position[D_AXIS];
	
	gc_state.position_Cartesian[A_AXIS] = sys.position_Cartesian[RX_Cartesian];
	gc_state.position_Cartesian[B_AXIS] = sys.position_Cartesian[RY_Cartesian];
	gc_state.position_Cartesian[C_AXIS] = sys.position_Cartesian[RZ_Cartesian];
	//memcpy(gc_state.position_Cartesian,sys.position_Cartesian,sizeof(sys.position_Cartesian));
	//将正解得到的位姿赋值给gc_state.position_Cartesian

}
gc_state.position[D_AXIS] = gc_block.values.xyz[D_AXIS];//增加笛卡尔对D轴的支持
							mc_line(gc_state.position, -1.0, false, false);
增加了在笛卡尔模式下对D轴的支持，原来在笛卡尔模式下发D轴的移动指令是不动的
check door总报错，注释掉：
/*if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR); 
          // If already in active, ready-to-resume HOLD, set CYCLE_STOP flag to force de-energize.
          // NOTE: Only temporarily sets the 'rt_exec' variable, not the volatile 'rt_exec_state' variable.
          if (sys.suspend & SUSPEND_ENABLE_READY) { bit_true(rt_exec,EXEC_CYCLE_STOP); }
          sys.suspend |= SUSPEND_ENERGIZE;
          sys.state = STATE_SAFETY_DOOR;
        }*/

17、20190718
再20190716中，有一个bug使得D轴运动会崩溃，修改了
//增加对传动带D轴的赋值
	gc_state.position_Cartesian[D_AXIS] = temp[D_AXIS];

18、20190722修改了笛卡尔模式下的第三轴角度范围，扩大

19、20190806重要更新：修改原版的thor主轴控制PWM输出方式，改回了grbl标准的输出方式，为了与标准的grbl激光雕刻主轴输出方式匹配，经测试，使用65值正好可以打开PWM开关

20\20190827 
（1）修改上电调试信息为GRBL默认，为了与G代码发送器匹配。（2）修改增加了对于回文的设置，可以设置是否回文。
21、20190828
增加三轴笛卡尔绝对模式下的xyz偏移量设置与显示
22、20190903
增加M40清除复位距离值，M40一键读取复位距离值
23、王帅修改成新PCB
24、20191016
增加对外界扩展板上reset复位按钮的支持，可以通过外接按钮进行复位操作。
25、20191021
增加了串口2的软件上拉，防止串口2收到乱码，导致错误
26、20191024
找到了笛卡尔模式下一直上升到一定位置角度向上翻90的bug,原因在于在逆解函数中使用abs函数对浮点数求绝对值，应当使用fabs函数。
27、20191206
认真解读了复位函数与代码，修改成了五轴同步复位！然后修改了第五轴的第二次复位运动距离！
28、20191218
增加软限位功能、包括软限位的参数调整。修改默认参数为批量生产版本。