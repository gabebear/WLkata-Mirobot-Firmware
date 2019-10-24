/**
  ******************************************************************************
  * @file	qinnew.c
  * @author	Thor Zhou	
  * @email	dongxvzhou@gmail.com
  * @date	2019-07-18
  ******************************************************************************
  */
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "grbl.h"//grbl.h里边包含了qinnew.h


void transpose(double *a, double *b, int dim)                   // 矩阵转置
{
  int i;
  int j;

  for (i = 0; i < dim; i++) {
    for (j = 0; j < dim; j++) {
      a[i*dim + j] = b[j*dim + i];
    }
  }
}

void MatrixMult(double *out, double *m1, double *m2, int dim)   // 矩阵乘法
{
  int i;
  int j;
  int k;
  double sum;
  for (i = 0; i < dim; i++) {
    for (j = 0; j < dim; j++) {
      sum = 0;
      for (k = 0; k < dim; k++) {
        sum += m1[i*dim + k] * m2[k*dim + j];
      }
      out[i*dim+j] = sum;
    }
  }
}
//---------------------------------------------------------------------------------------------------------
char            printfbuf[500];             // 打印缓冲区

#define pi          (3.1415926)     /* 圆周率 */
#define NOSOLUTION  (1000)
#define OUTANGLIMIT (1001)



#define N_SQRT      sqrt        /* 重定位开根号函数 */

double s;
double r;
double D4_A3;           /* 把 sqrt(pow(settings.robot_qinnew.D4,2) + pow(settings.robot_qinnew.A3,2)) 的三角计算提前计算好  */
double ATAN2D4_A3;      /* 把 atan2(settings.robot_qinnew.D4,settings.robot_qinnew.A3) 提前计算好 */
double atan2_s_r;       // 用于计算atan2(s,r)
double atan2_s_r1;      // 用于计算atan2(s,-r)
unsigned int _D1xD1_;            // 提前计算好平方 
unsigned int _A1xA1_;            // 提前计算好平方
unsigned int _A2xA2_;            // 提前计算好平方
unsigned int _A3xA3_;            // 提前计算好平方
unsigned int _D4xD4_;            // 提前计算好平方



void InverseInit(void) 
{
    _D1xD1_     = settings.robot_qinnew.D1 * settings.robot_qinnew.D1;
    _A1xA1_     = settings.robot_qinnew.A1 * settings.robot_qinnew.A1;
    _A2xA2_     = settings.robot_qinnew.A2 * settings.robot_qinnew.A2;
    _A3xA3_     = settings.robot_qinnew.A3 * settings.robot_qinnew.A3;
    _D4xD4_     = settings.robot_qinnew.D4 * settings.robot_qinnew.D4;
    D4_A3       = sqrt(_D4xD4_ + _A3xA3_);
    ATAN2D4_A3  = atan2(settings.robot_qinnew.D4,settings.robot_qinnew.A3);
}

// FIRST POSSIBLE SOLUTION OF THETA2 FUNCTION  cita2的第一个可能值
double THE2(double xtip,double ytip,double ztip,double theta1,double zeta, double lenda)
{  
//    double lenda = atan2(D4_A3 * sin (zeta) , settings.robot_qinnew.A2 + D4_A3 * cos(zeta));
    return (lenda - atan2_s_r) ;       // 这里的减去pi/2是表示，cita2，本身的位置不为0，是-pi/2，硬要让他为0
}

// SECOND POSSIBLE SOLUTION OF THETA2 FUNCTION  cita2的第二个可能值
double THE2COMP(double xtip,double ytip,double ztip,double theta1,double zeta , double lenda)
{  
//    double lenda = atan2(D4_A3 * sin (zeta) , settings.robot_qinnew.A2 + D4_A3 * cos(zeta));
    return (lenda - atan2_s_r1);                // 这里的减去pi/2是表示，cita2，本身的位置不为0，是-pi/2，硬要让他为0
}

// JOINT ANGLESLIMIT FUNCTION
double conversion(double theta,double upperlimit,double lowerlimit)
{
    if(theta > upperlimit * pi / 180)
        return OUTANGLIMIT;      // OUT = (' THE SOLUTION OUT OF JOINT ANGLE LIMIT ');
    else if(theta < lowerlimit * pi / 180)
        return OUTANGLIMIT;      //OUT = (' THE SOLUTION OUT OF JOINT ANGLE LIMIT ');
    else
        return (theta * 180 / pi);
}

/**
  * 函数功能: 逆解运算
  */
void Inverse(double x_wrist,double y_wrist,double z_wrist,double alpha,double beta,double gama)
{
 	alpha = alpha * pi/180;     
 	beta  = beta  * pi/180;
  	gama  = gama  * pi/180;
    
    double  temp1 = cos(alpha);
    double  temp2 = cos(beta);
    double  temp3 = cos(gama);
    double  temp4 = sin(alpha);
    double  temp5 = sin(beta);
    double  temp6 = sin(gama);
    
    double R60[3][3] = { { temp2*temp3 , temp3*temp4*temp5 - temp1*temp6 , temp1*temp3*temp5 + temp4*temp6},
                         { temp2*temp6 , temp4*temp5*temp6 + temp1*temp3 , temp1*temp5*temp6 - temp3*temp4},
                         {-temp5       , temp2*temp4                     , temp1*temp2                    }
                       };

    double xtip = x_wrist + R60[0][2] * abs(settings.robot_qinnew.L);
    double ytip = y_wrist + R60[1][2] * abs(settings.robot_qinnew.L);
    double ztip = z_wrist + R60[2][2] * abs(settings.robot_qinnew.L);
    
    // CALCULATING ALL THE POSSIBLE VALUES FOR THETA1
    double theta1  = atan2(ytip,xtip);
    double theta11 = pi + theta1;
    double THETA1  = theta1 * 180/pi;
    double THETA11 = theta11 * 180/pi;
    // CALCULATING ALL THE POSSIBLE VALUES FOR THETA3

    s = ztip - settings.robot_qinnew.D1;
    r = N_SQRT(pow(xtip - settings.robot_qinnew.A1*cos(theta1) ,2) + pow((ytip - settings.robot_qinnew.A1*sin(theta1)) ,2));

    atan2_s_r  = atan2(s, r);       // 为了减少计算次数 +++
    //atan2_s_r1 = atan 2(s, -r);    // 为了减少计算次数 +++
    if(s < 0) {
        atan2_s_r1 = -pi - atan2_s_r;
    } else {
        atan2_s_r1 = pi - atan2_s_r;
    }

    // double czeta = (pow(r,2) + pow(s,2)  - pow(settings.robot_qinnew.A2,2) - pow(settings.robot_qinnew.D4,2) - pow(settings.robot_qinnew.A3,2))/(2 * settings.robot_qinnew.A2 * D4_A3);
    double czeta = (r*r + s*s  - _A2xA2_ - _D4xD4_ - _A3xA3_) / (2 * settings.robot_qinnew.A2 * D4_A3);     // 这里占用60us
    
    // SINGULARTIY CONDTION, CHECK IF THE POSTION WITHIN THE WORKSPACE OR NOT
    double theta3 , theta33;
    double zeta   , zeta1;
    double THETA3 , THETA33;
    
    if(fabs(czeta) <= 1) {
        double szeta = N_SQRT(1 - czeta*czeta);
        zeta  = atan2(szeta,czeta);
       // zeta1 = atan2(-szeta,czeta);
        zeta1 = -zeta;  // 上面注释掉的先不要删除

        theta3  = -(zeta  + ATAN2D4_A3);
        theta33 = -(zeta1 + ATAN2D4_A3);
        
        THETA3  = conversion(theta3  , 70 ,-180);
        THETA33 = conversion(theta33 , 70 ,-180);
//        if(THETA3 < 0)      Jputs(0,"OUT OF JOINT ANGLE LIMIT 1\n"); 
//        if(THETA33 < 0)     Jputs(0,"OUT OF JOINT ANGLE LIMIT 2\n");
    } else {
        theta3 = NOSOLUTION;
        theta33= NOSOLUTION;
		printString("\r\nGOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR  THETA3!");
		return;

    }

    //s = ztip - settings.robot_qinnew.D1;
    temp1 = N_SQRT(pow((xtip - settings.robot_qinnew.A1*cos(theta11)),2) + pow((ytip - settings.robot_qinnew.A1*sin(theta11)),2));    // 这里原来是r 
    double czetai = (temp1 + pow(s,2)  - pow(settings.robot_qinnew.A2,2) - pow(settings.robot_qinnew.D4,2) - pow(settings.robot_qinnew.A3,2)) / (2 * settings.robot_qinnew.A2 * D4_A3);
    // SINGULARTIY CONDTION, CHECK IF THE POSTION WITHIN THE WORKSPACE OR NOT
    double theta3i;
    double theta33i;
    double zetai;
    double zeta1i;
    double THETA3i;
    double THETA33i;
    if(fabs(czetai) <= 1) {
        double szetai = N_SQRT(1-pow(czetai,2));   
        zetai  = atan2( szetai , czetai);
        //zeta1i = atan 2(-szetai , czetai);
        zeta1i   = -zetai;
        theta3i  = -(zetai  + ATAN2D4_A3);
        theta33i = -(zeta1i + ATAN2D4_A3);
        
        THETA3i  = conversion(theta3i  , 70 ,-180);
        THETA33i = conversion(theta33i , 70 ,-180);
//        if(THETA3i < 0)    Jputs(0,"OUT OF JOINT ANGLE LIMIT 3\n");
//        if(THETA33i < 0)   Jputs(0,"OUT OF JOINT ANGLE LIMIT 4\n");
    } else {
        theta3i =NOSOLUTION;
        theta33i=NOSOLUTION;
//		printString("\r\nGOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR  THETA3!");
//		return;  
    }

    // CALCULATING ALL THE POSSIBLE VALUES FOR THETA2
    double THETA2;
    double THETA22;
    if(theta3 == NOSOLUTION) {
//       Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2\n");
//       Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR  THETA22\n");
    } else {
        temp1 = atan2(D4_A3 * sin(zeta) , settings.robot_qinnew.A2 + D4_A3 * cos(zeta));
        double theta2  = THE2(xtip,ytip,ztip,theta1,zeta,temp1);          // 计算cita2 的两个可能值
        double theta22 = THE2COMP(xtip,ytip,ztip,theta1,zeta,temp1);
        
        THETA2  = conversion(theta2 , 0, -130);                    // 修改了cita2的初始值，从原来程序的0改为自然的-2/pi，这里也该减去90度
        THETA22 = conversion(theta22, 0, -130);
        
//        if(THETA2 < 0)   Jputs(0,"OUT OF JOINT ANGLE LIMIT 5\n");
//        if(THETA22 < 0)  Jputs(0,"OUT OF JOINT ANGLE LIMIT 6\n");
    }
    
    double theta2i;
    double theta22i;
    double THETA2i;
    double THETA22i;
    if(theta33 == NOSOLUTION) {
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2i\n");
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR  THETA22i\n");    
    } else {
//        sprintf(printfbuf,"xtip=%f \nytip=%f \nzytip=%f \ntheta1=%f \nzeta1=%f",xtip,ytip,ztip,theta1,zeta1);
//        Jprintf(0,"%s",printfbuf);
        temp1 = atan2(D4_A3 * sin(zeta1) , settings.robot_qinnew.A2 + D4_A3 * cos(zeta1));
        theta2i  = THE2(xtip,ytip,ztip,theta1,zeta1,temp1);
        theta22i = THE2COMP(xtip,ytip,ztip,theta1,zeta1,temp1);

        THETA2i  = conversion(theta2i , 0, -130);
        THETA22i = conversion(theta22i, 0, -130);
        
//        if(THETA2i < 0)   Jputs(0,"OUT OF JOINT ANGLE LIMIT 7\n");
//        if(THETA22i < 0)  Jputs(0,"OUT OF JOINT ANGLE LIMIT 8\n");
    }

    double THETA2j;
    double THETA22j;
    if(theta3i == NOSOLUTION) {
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2j\n");
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE ISNO VAILD VALUS FOR  THETA22j\n");    
    } else {
        temp1 = atan2(D4_A3 * sin(zetai) , settings.robot_qinnew.A2 + D4_A3 * cos(zetai));
        double theta2j  = THE2(xtip,ytip,ztip,theta11,zetai,temp1);
        double theta22j = THE2COMP(xtip,ytip,ztip,theta11,zetai,temp1);

        THETA2j  = conversion(theta2j  ,0, -130);
        THETA22j = conversion(theta22j ,0, -130);    
        
//        if(THETA2j < 0)   Jputs(0,"OUT OF JOINT ANGLE LIMIT 9\n");
//        if(THETA22j < 0)  Jputs(0,"OUT OF JOINT ANGLE LIMIT 10\n");
    }

    double THETA2k;
    double THETA22k;
    if(theta33i == NOSOLUTION) {
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR THETA2k\n");
//        Jprintf(0,"GOAL OUT OF WORKSPACE, THERE IS NO VAILD VALUS FOR  THETA22k\n");
     } else {
        temp1 = atan2(D4_A3 * sin(zeta1i) , settings.robot_qinnew.A2 + D4_A3 * cos(zeta1i));
        double theta2k  = THE2(xtip,ytip,ztip,theta11,zeta1i,temp1);
        double theta22k = THE2COMP(xtip,ytip,ztip,theta11,zeta1i,temp1);

        THETA2k  = conversion(theta2k  ,0, -130);
        THETA22k = conversion(theta22k ,0, -130);
//        if(THETA2k < 0)   Jputs(0,"OUT OF JOINT ANGLE LIMIT 11\n");
//        if(THETA22k < 0)  Jputs(0,"OUT OF JOINT ANGLE LIMIT 12\n");
    }
   

    temp1 = cos(theta1);
    temp2 = sin(theta1);
    temp3 = cos(theta2i+theta33);
    temp4 = sin(theta2i+theta33);
    double R30[3][3] = { 
                        { temp1 * temp3 , -temp1 * temp4 , -temp2},      // 这个使用T矩阵计算出来的，注意三角函数做了合并
                        { temp2 * temp3 , -temp2 * temp4 , temp1},      // 这里使用了上边cita123的SOL3这组解
                        {-temp4         , -temp3         , 0    }
                       };
    double RT30[3][3];
    double R63_cita456[3][3] = {{1,0,0},
                                {0,0,-1},
                                {0,1,0}}; //R63在cita456为 0/0/0时候的特殊值
    double RT63_cita456[3][3] ={{1,0,0},
                                {0,0,1},
                                {0,-1,0}}; //
    // R30*R63_cita456
    transpose(RT30[0],R30[0],3);
    double R63[3][3];
    double Rtemp[3][3];
    MatrixMult(Rtemp[0],RT63_cita456[0],RT30[0],3);     // 这部分把两个矩阵乘法分成两步,Rtemp做为中间缓存
    MatrixMult(R63[0],Rtemp[0],R60[0],3);               // R63 = RT63_cita456* RT30 * R60;
    
    double g11 = R63[0][0];                             // 加round元整数据到小数点后三位，否则，有的应该为0的数据显示为很小的正数
    double g12 = R63[0][1];
    double g13 = R63[0][2];
    double g23 = R63[1][2];
    double g21 = R63[1][0];
    double g22 = R63[1][1];
    
    double g31 = R63[2][0];
    double g32 = R63[2][1];
    double g33 = R63[2][2];
    
// THETA 4,5,6 CALCULATION
    double theta5 = atan2(N_SQRT(pow(g31,2) + pow(g32,2)), g33);
    
    double THETA4,theta4;
    double THETA5;
    double THETA6,theta6;
    
    temp1 = fabs(theta5 - pi);      // 为了应对浮点数精度问题  ,之前版本的bug在于使用了abs函数！
    if(theta5 < 0.0001) {
        THETA4= 0;
        THETA5= 0;
        theta6=  atan2(-g12, g11);   
        THETA6=  theta6*180/pi;   
        
   /*     tim_in = SysTick_Timer - tim_in;
        Jprintf(0,"\n\n===>2 Time use1 %dus\n\n",tim_in*10);*/
    } else if(temp1 < 0.0001) {
        THETA4=  0;
        THETA5=  0;
        theta6 = atan2 (g12,-g11);
				THETA6=  theta6*180/pi;
        
   /*     tim_in = SysTick_Timer - tim_in;
        Jprintf(0,"\n\n===>2 Time use2 %dus\n\n",tim_in*10);*/
    } else {
        temp1 = sin(theta5);
     
        theta4 = - atan2 (g23/ temp1, g13/ temp1);
				theta6 = atan2((g32/ temp1), (- g31/ temp1));
				THETA4=  conversion( theta4,180,-180);
				THETA5=  conversion( theta5,120,-90);
				THETA6=  conversion( theta6,180,-180);
        
        // FLIPPED POSTION
        double theta44 =  theta4 + pi;
        double theta55 = -theta5;
        double theta66 =  theta6 + pi;
        double THETA44 = conversion(theta44 , 180 , -180);        
        double THETA55 = conversion(theta55 , 120 , -90);        
        double THETA66 = conversion(theta66 , 180 , -180);
        
      //  tim_in = SysTick_Timer - tim_in;
        
//        if(THETA4 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 13\n");
//        if(THETA5 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 14\n");
//        if(THETA6 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 15\n");
//        
//        if(THETA44 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 16\n");
//        if(THETA55 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 17\n");
//        if(THETA66 < 0)   Serial.print("OUT OF JOINT ANGLE LIMIT 18\n");
        
        
      //  Jprintf(0,"\n\n===>2 Time use3 %dus\n\n",tim_in*10);
        
      //  Delay_msT(500);
      //  sprintf(printfbuf,"->THETA 4,5,6 SOLUTIONS\nTHETA4 = %f, THETA5 = %f, THETA6 = %f \n\nTHETA44 = %f, THETA55 = %f, THETA66 = %f \n\n",
      //                      THETA4,THETA5,THETA6,THETA44,THETA55,THETA66);
      //  Jprintf(0,"%s\n",printfbuf);
    }
#ifdef debug
  printString("\r\n");
  printString("THETA1:");
  printFloat(THETA1,2);
  printString("\r\n");
  printString("THETA2i:");
  printFloat(THETA2i+90,2);
  printString("\r\n");
  printString("THETA33:");
  printFloat(THETA33,2);
  printString("\r\n");
  printString("THETA4:");
  printFloat(THETA4,2);
  printString("\r\n");
  printString("THETA5:");
  printFloat(THETA5-90,2);
  printString("\r\n");
  printString("THETA6:");
  printFloat(THETA6,2);
  printString("\r\n");
#endif
if((THETA1 == 1000)||(THETA1 == 1001)||(THETA2i == 1000)||(THETA2i == 1001)||(theta33 == 1000)||(THETA33 == 1001)||\
	(THETA4 == 1000)||(THETA4 == 1001)||(THETA5 == 1000)||(THETA5 == 1001)||(THETA6 == 1000)||(THETA6 == 1001))
	{
	printString("\r\nThere is one angle out of limit!!!");
	return;
	}

  gc_state.position[E_AXIS] = THETA1;//x 第一轴
  gc_state.position[F_AXIS] = (THETA2i + 90);//y 第二轴
  gc_state.position[G_AXIS] = THETA33;//z 第三轴
  gc_state.position[A_AXIS] = THETA4;//A 第四轴
  gc_state.position[B_AXIS] = (THETA5 - 90);//B 第五轴
  gc_state.position[C_AXIS] = THETA6;//C 第六轴

 //   return 1;

}

void Forward(double *angle)
{
	for(int m =0;m<7;m++)
	{
	   if(m == F_AXIS)
	   	angle[m] = angle[m] - 90;
	   if(m == B_AXIS)
	   	angle[m] = angle[m] + 90;
	   
		angle[m] = angle[m] * pi/180;
		}
	
	double sin1 = sin(angle[4]);
	double cos1 = cos(angle[4]);
	
	double sin2 = sin(angle[5]);
	double cos2 = cos(angle[5]);
	
	double sin3 = sin(angle[6]);
	double cos3 = cos(angle[6]);
	
	double sin4 = sin(angle[0]);
	double cos4 = cos(angle[0]);
	
	double sin5 = sin(angle[1]);
	double cos5 = cos(angle[1]);
	
	double sin6 = sin(angle[2]);
	double cos6 = cos(angle[2]);
	
	double T10[4][4] = { 
                        { cos1, -sin1 , 0,  0},      
                        { sin1,  cos1 , 0,  0},      
                        {    0,     0 , 1, settings.robot_qinnew.D1},
                        {    0,     0 , 0,  1}
                       }; 
	double T21[4][4] = { 
                        { cos2, -sin2 , 0, settings.robot_qinnew.A1},      
                        {    0,     0 , 1,  0},      
                        {-sin2, -cos2 , 0,  0},
                        {    0,     0 , 0,  1}
                       };
  double T32[4][4] = { 
                        { cos3, -sin3 , 0, settings.robot_qinnew.A2},      
                        { sin3,  cos3 , 0,  0},      
                        {    0,     0 , 1,  0},
                        {    0,     0 , 0,  1}
                       };
  double T43[4][4] = { 
                        { cos4, -sin4 , 0, settings.robot_qinnew.A3},      
                        {    0,     0 , 1, settings.robot_qinnew.D4},      
                        {-sin4, -cos4 , 0,  0},
                        {    0,     0 , 0,  1}
                       };              
  double T54[4][4] = { 
                        { cos5, -sin5 , 0,  0},      
                        {    0,     0 ,-1,  0},      
                        { sin5,  cos5 , 0,  0},
                        {    0,     0 , 0,  1}
                       };                
	double T65[4][4] = { 
                        { cos6, -sin6 , 0,  0},      
                        {    0,     0 ,-1, -settings.robot_qinnew.L},      
                        { sin6,  cos6 , 0,  0},
                        {    0,     0 , 0,  1}
                       }; 
  double temp1[4][4];
  double temp2[4][4];
  //Serial.print("T10:");output(T10);
  MatrixMult(temp1[0],T10[0],T21[0],4);
  // Serial.print("T20:");output(temp1);
  MatrixMult(temp2[0],temp1[0],T32[0],4);
  // Serial.print("T30:");output(temp2);
  MatrixMult(temp1[0],temp2[0],T43[0],4);
  // Serial.print("T40:");output(temp1);
  MatrixMult(temp2[0],temp1[0],T54[0],4);
  // Serial.print("T50:");output(temp2);
  MatrixMult(temp1[0],temp2[0],T65[0],4);
  // Serial.print("T60:");output(temp1);
  
  double temp = sqrt(pow(temp1[2][1],2)+pow(temp1[2][2],2));
  sys.position_Cartesian[RX_Cartesian] = atan2(temp1[2][1],temp1[2][2])*180/pi;
  sys.position_Cartesian[RY_Cartesian] = atan2(-temp1[2][0],temp)*180/pi;
  sys.position_Cartesian[RZ_Cartesian] = atan2(temp1[1][0],temp1[0][0])*180/pi;

  sys.position_Cartesian[X_Cartesian] = temp1[0][3];
  sys.position_Cartesian[Y_Cartesian] = temp1[1][3];
  sys.position_Cartesian[Z_Cartesian] = temp1[2][3];
  
//  printString("\r\nsys.position_Cartesian[Z_Cartesian]");
//  printFloat(sys.position_Cartesian[Z_Cartesian],2);
	}



void go_reset_pos()
{

	//memset(sys.position,0,sizeof(sys.position));
	
	float temp[7] ={0,0,0,0,0,0,0};
	for(int idx = 0;idx < N_AXIS; idx++)
	{
			if(bit_istrue(settings.homing_pos_dir_mask,bit(idx)))
				{
				temp[idx] = - settings.Reset[idx];
			}else
				temp[idx] =  settings.Reset[idx];
	}
/*
	printString("\r\nA_AXIS:");
	printFloat(temp[A_AXIS],2);
	printString("\r\nB_AXIS:");
	printFloat(temp[B_AXIS],2);
	printString("\r\nC_AXIS:");
	printFloat(temp[C_AXIS],2);
	printString("\r\nX_AXIS:");
	printFloat(temp[E_AXIS],2);
	printString("\r\nY_AXIS:");
	printFloat(temp[F_AXIS],2);
	printString("\r\nZ_AXIS:");
	printFloat(temp[G_AXIS],2);

	printString("\r\nposition[A_AXIS]:");
	printFloat(sys.position[A_AXIS],2);
	printString("\r\nposition[B_AXIS]:");
	printFloat(sys.position[B_AXIS],2);
	printString("\r\nposition[C_AXIS]:");
	printFloat(sys.position[C_AXIS],2);
	printString("\r\nposition[E_AXIS]:");
	printFloat(sys.position[E_AXIS],2);
	printString("\r\nposition[F_AXIS]:");
	printFloat(sys.position[F_AXIS],2);
	printString("\r\nposition[G_AXIS]:");
	printFloat(sys.position[G_AXIS],2);
*/
	
	mc_line(temp, -1.0, false ,false);
	sys.home_complate_flag = 1;
	//printString("in homeing moving...");
}

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
	//printFloat(gc_state.position_Cartesian[G_AXIS],2);

	//增加对传动带D轴的赋值
	gc_state.position_Cartesian[D_AXIS] = temp[D_AXIS];
	
	gc_state.position_Cartesian[A_AXIS] = sys.position_Cartesian[RX_Cartesian];
	gc_state.position_Cartesian[B_AXIS] = sys.position_Cartesian[RY_Cartesian];
	gc_state.position_Cartesian[C_AXIS] = sys.position_Cartesian[RZ_Cartesian];
	//memcpy(gc_state.position_Cartesian,sys.position_Cartesian,sizeof(sys.position_Cartesian));
	//将正解得到的位姿赋值给gc_state.position_Cartesian

}
void coordinate_to_angle()
{
	system_convert_array_steps_to_mpos(gc_state.position,sys.position);

}

void start_calibration()//开始校准，清零原来复位参数
{
	settings_store_global_setting(140, 0);
	settings_store_global_setting(141, 0);
	settings_store_global_setting(144, 0);
	settings_store_global_setting(145, 0);
	settings_store_global_setting(146, 0);

}

void write_reset_distance()
{
	int32_t current_position[N_AXIS]; // Copy current state of the system position variable
	memcpy(current_position,sys.position,sizeof(sys.position));
	double print_position[N_AXIS];
	system_convert_array_steps_to_mpos(print_position,current_position);
	
	settings_store_global_setting(140, fabs(print_position[A_AXIS]));//a
	settings_store_global_setting(141, fabs(print_position[B_AXIS]));//b
	settings_store_global_setting(144, fabs(print_position[E_AXIS]));//e
	settings_store_global_setting(145, fabs(print_position[F_AXIS]));//f
	settings_store_global_setting(146, fabs(print_position[G_AXIS]));//g

}

void reset_button_init()
{
	BUTTON_RESET_DDR &= ~(BUTTON_RESET_MASK); // Set as input pins
	BUTTON_RESET_PORT |= (BUTTON_RESET_MASK);  // Enable internal pull-up resistors. Normal high operation.

}

void reset_button_check()
{
 if ((BUTTON_RESET_PIN & BUTTON_RESET_MASK)==0) {
 	delay_ms(10);
	   if ((BUTTON_RESET_PIN & BUTTON_RESET_MASK)==0) {
		if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { 
            sys.state = STATE_HOMING; // Set system state variable
            // Only perform homing if Grbl is idle or lost.
            
            // TODO: Likely not required.
            if (system_check_safety_door_ajar()) { // Check safety door switch before homing.
              bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              protocol_execute_realtime(); // Enter safety door mode.
            }

			printString("Button reset...");
			
            mc_homing_cycle(); 
			//这里是执行复位以后的初始化脚本，因为没有所以先注释掉
            if (!sys.abort) {  // Execute startup scripts after successful homing.
              sys.state = STATE_IDLE; // Set to IDLE when complete.
              st_go_idle(); // Set steppers to the settings idle state before returning.
              //system_execute_startup(line); 
            }
            
          } else { return(STATUS_SETTING_DISABLED); }

	   }
 	}
}