#ifndef __HWT101_H
#define __HWT101_H

/*
 * 陀螺仪驱动头文件 —— 兼容 HWT101 / JY61P
 * 函数名保持 HWT101_ 前缀以最小化工程改动，实际已适配 JY61P 协议
 */

#include <stdint.h>

extern uint8_t HWT101_wRxFlag,HWT101_AngleRxFlag;	//角速度,角度接收完成标志位
extern float HWT101_Yaw_W;							//Yaw轴角速度(°/s)
extern float HWT101_Yaw;							//Yaw轴角度(°)
extern float HWT101_CheckYaw;						//Yaw轴角度校验值

void HWT101_Init(void);			//陀螺仪初始化（兼容 HWT101 / JY61P）
void HWT101_AngleCheck(void);	//陀螺仪角度校验
void HWT101_Callback(void);		//陀螺仪回调函数(USART2空闲中断)

#endif
