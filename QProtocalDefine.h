#ifndef _Q_PROTOCAL_H_
#define _Q_PROTOCAL_H_


typedef enum
{
			ENM_IDLE = 0,
			ENM_RUN = 1,
			ENM_HOLD = 2,
			ENM_OTHER = 3,
			ENM_MAX,
}RunState;

typedef enum
		{
			Cartesian = 0,
			Angle = 1,
		}Coordination;

typedef struct
{
RunState state;
Coordination coor;

}RobotProtocal;

RobotProtocal Protocal;

//#!1:s=%1,c=%1,r1=%1,r2=%1,r3=%1,r4=%1,r5=%1,r6=%1,x1=%1,y=%1,z=%1,a=%1,b=%1,c=%1,p1=%1,p2=%1

#endif // _Q_PROTOCAL_H_
