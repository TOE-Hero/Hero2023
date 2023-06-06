#include "myTask04.h"
#include "RM_Task.h"
#include "judge.h"
extern UI_RX uiRx;
void StartTask04(void const * argument)
{
    for (;;) 
	{
		CheckID();
		Start_UI_Task();
		//DrawCapVolRectangle(uiRx.realPowData,2);
		osDelay(2);
    }
}
