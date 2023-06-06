#include "main.h"
#include "Angle.h"

int8_t count;
//u8 init_condition;
float motorAngle;
motor_AngleTypeDef motor_AngleStructure;

float motorAngle_deal(int16_t angle)
{
	 int tend;
	
	 motor_AngleStructure.aglNow= angle; //电调当前角度值
	 tend= motor_AngleStructure.aglNow - motor_AngleStructure.aglLast;//电调当前角度值-电调上次返回角度值
	 
	 if(tend > 6000)       count+=1;    
	 else if(tend < -6000) count-=1;    
	 
	 motorAngle= count*360+ (motor_AngleStructure.aglOriginal-motor_AngleStructure.aglNow)/8191*360;
	 motorAngle= -motorAngle;           
	 motor_AngleStructure.aglLast= motor_AngleStructure.aglNow;        
	 return motorAngle;
}
