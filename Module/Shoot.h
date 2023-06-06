#ifndef __SHOOT_H
#define __SHOOT_H

#define TRANS_STEP			26215.6//8191/6*19.203208556
#define in_times(x) 		1000/1/x //5是程序运行的间隔,函数用来计算每秒钟进入任务的次数


void Shoot_Init(void);
void Shoot_Move(void);

#endif
