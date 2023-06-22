#ifndef __GIMBAL_H
#define __GIMBAL_H

typedef struct
{
    float pitchCenter;
    float pitchOffset;
    float pitPos;

    float disOffset;
    float shootOffset;
    float distance;

    float yawCenter;
    float filterOffset;
    float yawPos;
	
	float yawPre;
	float yawRelV;
	
}visInf_t;
/*********************** extern ***********************/
extern visInf_t s_visionInform;//自瞄结构体
/******************** function declaration ********************/
void Gimbal_Init(void);
void Gimbal_Move(void);

#endif

