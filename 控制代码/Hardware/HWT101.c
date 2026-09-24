#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Delay.h"

/*
 * 陀螺仪驱动 —— 兼容 HWT101 / JY61P（维特系 0x55 串口协议）
 *
 * 适配说明：
 *   - HWT101 默认输出 角速度帧(0x55 0x52) + 角度帧(0x55 0x53)，各 11 字节
 *   - JY61P 默认输出 加速度帧(0x55 0x51) + 角速度帧(0x55 0x52) + 角度帧(0x55 0x53)
 *   - 本驱动扫描 0x55 0x53 帧头来提取偏航角，不再依赖固定偏移，因此无论
 *     IMU 输出几帧、顺序如何，都能正确解算
 *   - DMA 缓冲区扩大到 44 字节（最多容纳 4 帧），JY61P 多发的加速度帧不会越界
 *
 * JY61P 配置提示（如需修改输出内容，通过串口发送寄存器写入指令）：
 *   - REG 0x03 = 0x09 → 仅输出角速度+角度（与 HWT101 一致，最省带宽）
 *   - REG 0x03 = 0x0F → 输出加速度+角速度+角度（JY61P 默认）
 *   - 波特率默认 115200，与 HWT101 相同
 */

uint8_t HWT101_RxData0[44];//DMA数据存储器0（扩大至44字节以兼容JY61P多帧输出）
uint8_t HWT101_RxData1[44];//DMA数据存储器1

uint8_t HWT101_wRxFlag,HWT101_AngleRxFlag;//角速度,角度接收完成标志位
float HWT101_Yaw_W;//Yaw轴角速度(°/s)
float HWT101_LastYaw,HWT101_ThisYaw,HWT101_R,HWT101_Yaw;//Yaw轴角度(°)
float HWT101_CheckYaw;//Yaw轴角度校验值

#define IMU_DMA_BUF_SIZE 44//DMA缓冲区大小

/*
 *函数简介:陀螺仪初始化
 *参数说明:无
 *返回类型:无
 *备注:采用USART2空闲中断+DMA双缓冲接收读取数据
 *备注:默认采用PA3
 */
void HWT101_Init(void)
{
	/*===============配置时钟===============*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//开启时钟
	
	/*===============配置GPIO===============*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化USART2-Rx(PA3)
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);//开启PA3的USART2复用模式
	
	/*===============配置USART和串口接收DMA===============*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Rx;//配置为接收模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART2,&USART_InitStructure);//初始化USART2
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel=DMA_Channel_4;//选择DMA通道4
	DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;//普通模式(非自动重装)
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;//转运方向为外设到存储器
	DMA_InitStructure.DMA_BufferSize=IMU_DMA_BUF_SIZE;//数据传输量
	DMA_InitStructure.DMA_Priority=DMA_Priority_Low;//最低优先级
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(USART2->DR);//外设地址(USART的DR数据接收寄存器)
	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;//外设数据长度为1字节(8bits)
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//外设地址不自增
	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)HWT101_RxData0;//存储器地址(DMA数据存储器0)
	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//存储器数据长度为1字节(8bits)
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;//存储器地址自增
	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;//不使用FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOStatus_1QuarterFull;//设置FIFO阈值为1/4(不使用FIFO模式时,此位无意义)
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);//初始化数据流5
	
	DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)HWT101_RxData1,DMA_Memory_0);//设置双缓冲搬运从DMA数据存储器0开始
	DMA_DoubleBufferModeCmd(DMA1_Stream5,ENABLE);//使能DMA双缓冲功能

	/*===============配置空闲中断===============*/
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//打通USART2到NVIC的串口空闲中断通道
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;//选择USART2中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//响应优先级
	NVIC_Init(&NVIC_InitStructure);
	
	/*===============使能===============*/
	DMA_Cmd(DMA1_Stream5,ENABLE);//使能DMA1的数据流5
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//使能串口USART2的DMA搬运
	USART_Cmd(USART2,ENABLE);//启动USART2
}

/*
 *函数简介:陀螺仪角度校验
 *参数说明:无
 *返回类型:无
 *备注:用于消除陀螺仪初始偏移
 */
void HWT101_AngleCheck(void)
{
	HWT101_CheckYaw=HWT101_Yaw;//修复：原为累加式(HWT101_Yaw+HWT101_CheckYaw)，多次调用时零点偏移
}

/*
 *函数简介:陀螺仪 DMA计数复位
 *参数说明:无
 *返回类型:无
 *备注:双缓冲会自动重装,此函数用以在串口空闲时复位计数值,防止噪声影响后续DMA
 */
void HWT101_DMAReset(void)
{
	DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);//清除接收完成标志位
	DMA_Cmd(DMA1_Stream5,DISABLE);//失能DMA1的数据流5
	while(DMA_GetCmdStatus(DMA1_Stream5)!=DISABLE);//检测DMA1的数据流5为可配置状态
	DMA_SetCurrDataCounter(DMA1_Stream5,IMU_DMA_BUF_SIZE);//恢复传输计数器的值
	DMA_Cmd(DMA1_Stream5,ENABLE);//使能DMA1的数据流5
}

/*
 *函数简介:陀螺仪数据处理（兼容 HWT101 / JY61P）
 *参数说明:无
 *返回类型:无
 *备注:扫描 0x55 帧头，按帧ID解算，不再依赖固定偏移
 *
 * 维特系 IMU 串口帧格式（每帧 11 字节）：
 *   帧头0x55 + 帧ID + 8字节数据 + 校验和
 *
 * 帧ID:
 *   0x51  加速度输出（JY61P 默认包含，HWT101 不含）
 *   0x52  角速度输出
 *   0x53  角度输出
 *
 * 角速度帧(0x52): [0x55][0x52][reserved×2][RWzL RWzH][WzL WzH][reserved×2][SUM]
 * 角度帧(0x53):   [0x55][0x53][reserved×2][RollL RollH][YawL YawH][VerL VerH][SUM]
 */
void HWT101_DataProcess(void)
{
	uint8_t *Data;//选择存储器
	if(DMA_GetCurrentMemoryTarget(DMA1_Stream5)==0)Data=HWT101_RxData1;//若当前转运位于存储器0,则存储器1数据完整
	else Data=HWT101_RxData0;//若当前转运位于存储器1,则存储器0数据完整
	
	//扫描缓冲区中的所有帧（每帧11字节，缓冲区最多容纳4帧）
	for(uint8_t i=0; i<=IMU_DMA_BUF_SIZE-11; i++)
	{
		if(Data[i]!=0x55)continue;//寻找帧头
		
		uint8_t id=Data[i+1];//帧ID
		
		//校验和：SUM = 0x55 + ID + Data[2]~Data[9]
		uint8_t sum=0x55;
		for(uint8_t j=1;j<10;j++)sum+=Data[i+j];
		if(Data[i+10]!=sum)continue;//校验失败，跳过
		
		if(id==0x52)//角速度帧
		{
			HWT101_Yaw_W=(float)((int16_t)(Data[i+7]<<8)|Data[i+6])/32768.0f*2000.0f;//获取偏航角速度
			HWT101_wRxFlag=1;//置接收完成标志位
		}
		else if(id==0x53)//角度帧
		{
			HWT101_LastYaw=HWT101_ThisYaw;
			HWT101_ThisYaw=(float)((int16_t)(Data[i+7]<<8)|Data[i+6])/32768.0f*180.0f;//获取偏航角(Yaw)
			
			if(HWT101_LastYaw-HWT101_ThisYaw>180)HWT101_R++;
			else if(HWT101_LastYaw-HWT101_ThisYaw<-180)HWT101_R--;
			HWT101_Yaw=360.0f*HWT101_R+HWT101_ThisYaw-HWT101_CheckYaw;
			
			HWT101_AngleRxFlag=1;//置接收完成标志位
		}
		//0x51 加速度帧：不解算，直接跳过
	}
}

/*
 *函数简介:陀螺仪回调函数
 *参数说明:无
 *返回类型:无
 *备注:在USART2空闲中断调用,用以解算数据
 */
void HWT101_Callback(void)
{
	HWT101_DataProcess();//数据处理（不再依赖固定DMA计数，直接扫描帧头）
	
	HWT101_DMAReset();
}
