�޸���
1��������һ·PWM�����ԭ����PWM����������ֽ�7��ʹ��ָ��M3 S
     �����ӵ������ֽ�2��ʹ��ָ��M4 E
2��ָ��������E�ؼ��֣���������ߵ�word_bit�����16λ�����Ҷ����ˣ����԰�ԭ���Ĺؼ���T�޸ĳ���E����E�ؼ�����ʹ��

3\20190403  �������˶�ѧ���⣬����ģʽ���л���������ģʽ������ģʽ

4��2019.4.8��һ���޸������Ŷ��壬���Լ������µĵ�·ͼƥ��

5��20190411 ��һ���ڵѿ���ģʽ��������̬�����˶���ԭ�����ڲ岹����//if (cartesian_mm < 0.000001) { break; 
}
Ҳ����˵��XYZ����λ�ò���������£�����ֱ�������ˣ����˶���
����ע�͵���

6���޸���Ĭ�ϲ��� 

7��20190412�ı�����⺯���и����Ƕȵ���λ��ֵ�������˵���λ������λ��ֵʱ�Ĵ���

8��20190418�޸���L��ȱ�ٸ��ŵ�bug,�ѿ�����ֵ������L��ֵӰ��

9��20190419�Ի�е�۲������ý��д�ģ���ԭ���궨��Ĳ������ö������ָ����ӵ�$29�Ժ������˶����Ƿ����ò岹���Լ��岹���ȵ����ã����������Ƿ����ò���û��ʹ�ã�������Ϊ�궨���ɲ����е�����

10��20190422����⺯�������޸ģ���Ҫ���ڵѿ���Ŀ��㳬����Χ���λ��ʱ����ʾ�޽�ֱ��return

10��20190422_2������һ��G01��ָ̬�ִ�У��޸�

11��20190423 ����λ�ò���ʱֻ�ı���̬������Ӱ�죺//	if (cartesian_mm < 0.000001) { break; }ע�͵�
��G0�еĲ岹��������Ϊ��Զ�����룺#if 0

12��20190424 �޸��˸�λ�ص����룬�����ѵ�����ĳ�����������ֵ��������

13��20190425 
�޸������˴���2�ĺ��������Ҵ���1�ʹ���2�����Խ���ָ�������λ������ͬʱ�ظ�����1��2

14��20190510
���ص���Ϣ��ÿ���˶������Զ�����һ�Σ����Ҽ��������úͷ�״̬���Լ��˶�״̬�ѿ������߽Ƕȡ������޸��������˵����������������û��ȫ��

15��grbl_6v20190611
�岹��һ������int step,�޸��˵�

16��20190716
void angle_to_coordinate()
{
	double temp[N_AXIS];
	system_convert_array_steps_to_mpos(temp,sys.position);
	Forward(temp);//���ۺ�ʱ��sys.position�б���Ķ��ǽǶȵľ���λ��ֵ�������������ʾ�ģ�����
	//ִ�����Ժ󣬵õ��Ľ��������sys.position_Cartesian��
	gc_state.position_Cartesian[E_AXIS] = sys.position_Cartesian[X_Cartesian];
	gc_state.position_Cartesian[F_AXIS] = sys.position_Cartesian[Y_Cartesian];
	gc_state.position_Cartesian[G_AXIS] = sys.position_Cartesian[Z_Cartesian];
	printString_debug("\r\gc_state.position_Cartesian[G_AXIS]:");
	printFloat(gc_state.position_Cartesian[G_AXIS],2);

	//���ӶԴ�����D��ĸ�ֵ
	gc_state.position_Cartesian[D_AXIS] = sys.position[D_AXIS];
	
	gc_state.position_Cartesian[A_AXIS] = sys.position_Cartesian[RX_Cartesian];
	gc_state.position_Cartesian[B_AXIS] = sys.position_Cartesian[RY_Cartesian];
	gc_state.position_Cartesian[C_AXIS] = sys.position_Cartesian[RZ_Cartesian];
	//memcpy(gc_state.position_Cartesian,sys.position_Cartesian,sizeof(sys.position_Cartesian));
	//������õ���λ�˸�ֵ��gc_state.position_Cartesian

}
gc_state.position[D_AXIS] = gc_block.values.xyz[D_AXIS];//���ӵѿ�����D���֧��
							mc_line(gc_state.position, -1.0, false, false);
�������ڵѿ���ģʽ�¶�D���֧�֣�ԭ���ڵѿ���ģʽ�·�D����ƶ�ָ���ǲ�����
check door�ܱ���ע�͵���
/*if (rt_exec & EXEC_SAFETY_DOOR) {
          report_feedback_message(MESSAGE_SAFETY_DOOR_AJAR); 
          // If already in active, ready-to-resume HOLD, set CYCLE_STOP flag to force de-energize.
          // NOTE: Only temporarily sets the 'rt_exec' variable, not the volatile 'rt_exec_state' variable.
          if (sys.suspend & SUSPEND_ENABLE_READY) { bit_true(rt_exec,EXEC_CYCLE_STOP); }
          sys.suspend |= SUSPEND_ENERGIZE;
          sys.state = STATE_SAFETY_DOOR;
        }*/

17��20190718
��20190716�У���һ��bugʹ��D���˶���������޸���
//���ӶԴ�����D��ĸ�ֵ
	gc_state.position_Cartesian[D_AXIS] = temp[D_AXIS];

18��20190722�޸��˵ѿ���ģʽ�µĵ�����Ƕȷ�Χ������

19��20190806��Ҫ���£��޸�ԭ���thor�������PWM�����ʽ���Ļ���grbl��׼�������ʽ��Ϊ�����׼��grbl���������������ʽƥ�䣬�����ԣ�ʹ��65ֵ���ÿ��Դ�PWM����

20\20190827 
��1���޸��ϵ������ϢΪGRBLĬ�ϣ�Ϊ����G���뷢����ƥ�䡣��2���޸������˶��ڻ��ĵ����ã����������Ƿ���ġ�
21��20190828
��������ѿ�������ģʽ�µ�xyzƫ������������ʾ
22��20190903
����M40�����λ����ֵ��M40һ����ȡ��λ����ֵ
23����˧�޸ĳ���PCB
24��20191016
���Ӷ������չ����reset��λ��ť��֧�֣�����ͨ����Ӱ�ť���и�λ������
25��20191021
�����˴���2�������������ֹ����2�յ����룬���´���
26��20191024
�ҵ��˵ѿ���ģʽ��һֱ������һ��λ�ýǶ����Ϸ�90��bug,ԭ����������⺯����ʹ��abs�����Ը����������ֵ��Ӧ��ʹ��fabs������
27��20191206
�������˸�λ��������룬�޸ĳ�������ͬ����λ��Ȼ���޸��˵�����ĵڶ��θ�λ�˶����룡
28��20191218
��������λ���ܡ���������λ�Ĳ����������޸�Ĭ�ϲ���Ϊ���������汾��