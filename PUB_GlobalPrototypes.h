/********************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.
 文件名:    PUB_GlobalPrototypes.h
 创建人：王治国               创建日期：12.03.05 
 描述：
       1.减少共享所有头文件的情况，避免数据随意更改,
       2.常规数据类型的声明，及其他模块或文件需要共享的函数的原型声明.
       3.该同文件共所有模块调用
 修改记录：  
    1 xx.xx.xx      XX  
       变更内容： xxxxxxxxxxx
    2 xx.xx.xx      XX
       变更内容： xxxxxxxxxxx

********************************************************************************/
#ifndef PUB_GLOBALPROTOTYPES_H
#define PUB_GLOBALPROTOTYPES_H 

#ifdef __cplusplus
extern "C" {
#endif   

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */
/*  驱动器系列号 0-64
    IS620P  通用平台的驱动器 00-09
    IS650P  通用平台的驱动器 10-19
    IS620P2 通用平台的驱动器 20-29
*/
//驱动板
#define  POWDRV_IS620               0
#define  POWDRV_IS650               1

//驱动器    
#define  SERVO_620P_407VG           0 
#define  SERVO_620N                 1
#define  SERVO_620PKS               2
#define  SERVO_620M                 3
#define  SERVO_620P2                4

#define  SERVO_650P                 10
#define  SERVO_650N                 11 


//驱动器型号
#define DRIVER_TYPE             SERVO_620N 
#define POWERDRIVER_TYPE        (DRIVER_TYPE / 10)

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#if DRIVER_TYPE == SERVO_620P_407VG
    #include "PUB_IS620P_Config.h"
#elif DRIVER_TYPE == SERVO_620N
    #include "PUB_IS620N_Config.h"
#elif DRIVER_TYPE == SERVO_620PKS     
    #include "PUB_IS620PKS_Config.h"
#elif DRIVER_TYPE == SERVO_650P
    #include "PUB_IS650P_Config.h"
#elif DRIVER_TYPE == SERVO_650N
    #include "PUB_IS650N_Config.h"
#endif 

/* 配置头文件引用关系
    PUB_ServoConfiguration.h
    |
    |--->PUB_IS620P_Config.h
    |      
    |--->PUB_IS620N_Config.h    
    |
    |--->PUB_IS620K_Config.h
    |    
    |--->PUB_IS650P_Config.h
    |    
    |--->PUB_IS650N_Config.h
    |      
*/ 

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */
/*exact-width signed integer types */
typedef   signed          char int8;
typedef   signed short     int int16;
typedef   signed           int int32;
typedef   signed       __int64 int64;

/* exact-width unsigned integer types */
typedef unsigned           char Uint8;
typedef unsigned short     int Uint16;
typedef unsigned           int Uint32;
typedef unsigned       __int64 Uint64;


//声明指针变量初始化地址对应的变量
extern Uint32 PUB_Null_PointVar32Init;
extern Uint16 PUB_Null_PointVar16Init;

//DSP分配给FPGA的存储器起始映像地址
#define FPGA_BASE  0x60000000

    /* 内联函数关键字宏定义 */
    #define Static_Inline   static __inline

    /* 内嵌汇编指令函数宏定义,使用时需要包含stm32f10x.h文件 */
    //开中断
    #define  EINT    __set_PRIMASK(0)
    //关中断
    #define  DINT    __set_PRIMASK(1)
    //空操作
    #define  NOP     __NOP()
/* The table below gives the allowed values of the pre-emption priority and subpriority according
 * to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
 * ============================================================================================================================
 *   NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
 * ============================================================================================================================
 *  NVIC_PriorityGroup_0  |                0                  |            0-15             |   0 bits for pre-emption priority
 *                        |                                   |                             |   4 bits for subpriority
 * ----------------------------------------------------------------------------------------------------------------------------
 *  NVIC_PriorityGroup_1  |                0-1                |            0-7              |   1 bits for pre-emption priority
 *                        |                                   |                             |   3 bits for subpriority
 * ----------------------------------------------------------------------------------------------------------------------------
 *  NVIC_PriorityGroup_2  |                0-3                |            0-3              |   2 bits for pre-emption priority
 *                        |                                   |                             |   2 bits for subpriority
 * ----------------------------------------------------------------------------------------------------------------------------
 *  NVIC_PriorityGroup_3  |                0-7                |            0-1              |   3 bits for pre-emption priority
 *                        |                                   |                             |   1 bits for subpriority
 * ----------------------------------------------------------------------------------------------------------------------------
 *  NVIC_PriorityGroup_4  |                0-15               |            0                |   4 bits for pre-emption priority
 *                        |                                   |                             |   0 bits for subpriority  
 * ============================================================================================================================
 */
    //EXTI线0中断优先级(辅助FPGA发送的外部中断 转矩中断)
    #define  EXTI0_UP_IRQ_PreemptionPriority        0
    #define  EXTI0_UP_IRQ_SubPriority               0

    //定时器TIM2更新中断优先级(辅助中断)
    #define  TIM2_UP_IRQ_PreemptionPriority         2
    #define  TIM2_UP_IRQ_SubPriority                2

	//CANopen供EtherCAT测试用
		
	//定时器TIM9更新中断优先级(辅助中断)
//	#define  TIM9_UP_IRQ_PreemptionPriority 		1
//	#define  TIM9_UP_IRQ_SubPriority				2

	//CAN SDO接收中断
//	#define CAN_RX0_IRQ_PreemptionPriority          3
//	#define CAN_RX0_IRQ_SubPriority 				1
	//CAN 同步和PDO接收中断
//	#define CAN_RX1_IRQ_PreemptionPriority          1
//	#define CAN_RX1_IRQ_SubPriority                 2
	//CAN 发送中断
//	#define CAN_TX_IRQ_PreemptionPriority       2
//	#define CAN_TX_IRQ_SubPriority 				2
	//CANopen供EtherCAT测试用
	
	//串口UART1全局中断优先级(MODBUAS中断)
    #define  USART1_IRQ_PreemptionPriority          2
    #define  USART1_IRQ_SubPriority                 0

    //EXTI线1中断优先级(软件触发中断 位置中断)
    #define  EXTI1_IRQ_PreemptionPriority           1
    #define  EXTI1_IRQ_SubPriority                  2 
    
	//EXTI线2中断优先级(Et1100  SYNC0中断)
    #define  EXTI2_IRQ_PreemptionPriority           0
    #define  EXTI2_IRQ_SubPriority                  1 

    //EXTI线4中断优先级(Et1100  IRQ中断)
    #define  EXTI4_IRQ_PreemptionPriority           0
    #define  EXTI4_IRQ_SubPriority                  1 
    

//    //EXTI线7中断优先级(PB7中断 Z中断) 
//    //标准程序采用在电流环里面查询的方式处理Z中断相关功能
//    #define  EXTI9_5_IRQ_PreemptionPriority         2
//    #define  EXTI9_5_IRQ_SubPriority                0

    //CANLINK CAN1_RX0_IRQ中断优先级
//    #define  CAN1_RX0_IRQ_PreemptionPriority           2
//    #define  CAN1_RX0_IRQ_SubPriority                  6
//
//    //CANLINK CAN1_RX1_IRQ中断优先级
//    #define  CAN1_RX1_IRQ_PreemptionPriority           2
//    #define  CAN1_RX1_IRQ_SubPriority                  6 
     

#define     PUB_SYSCLK_FREQ_168MHz      168000000L
#define     PUB_APB1_FREQ_42MHz         42000000L
#define     PUB_APB1_TIM_FREQ_84MHz     84000000L
#define     PUB_APB2_FREQ_84MHz         84000000L  
#define     PUB_APB2_TIM_FREQ_168MHz    168000000L

#define     SQRT2_Q10       1448L
#define     PI_Q12          12868L

//-----------使能PWM----------------
#define   ENPWM          1
//-----------关PWM----------------
#define   DISPWM         0

// 运行模式宏定义：
#define TOQMOD      0x01
#define SPDMOD      0x02
#define POSMOD      0x04

//ECT运行模式宏定义
#define ECTPOSMOD      0x01
#define ECTSPDMOD      0x03
#define ECTTOQMOD      0x04
#define ECTHOMMOD      0x06
#define ECTCSPMOD      0x08
#define ECTCSVMOD      0x09
#define ECTCSTMOD      0x0A

//ECT运行模式宏定义
#define CANOPENPROPOSMOD   0x01
#define CANOPENPROVELMOD   0x03
#define CANOPENPROTOQMOD   0x04
#define CANOPENHOMMOD      0x06
#define CANOPENINTPMD      0x07



// 运行状态宏定义：
//0 伺服未准备好状态 此时SVRDY无效
#define NRD         0x00
//1 准备好状态  SERVO_OFF
#define RDY         0x01
//2 伺服正常运行态  SERVO_ON
#define RUN         0x02
//3 伺服故障态
#define ERR         0x03

/* Exported_Macros -----------------------------------------------------------*/
/* 带参数的宏定义 */
//求取绝对值
#define ABS(A)      (((A)>=0) ? (A) : (-(A)))

//取参数A、B的最小值
#define MIN(A,B)    (((A)<(B))? (A) : (B))

//取参数A、B的最大值
#define MAX(A,B)    (((A)>(B))? (A) : (B))

//下限幅
#define MINLMT(A,Min) (((A)<(Min))? (Min) : (A))

//上限幅
#define MAXLMT(A,Max) (((A)>(Max))? (Max) : (A))

//上下限幅
#define MAX_MIN_LMT(A,Pos,Neg)  MAX(MIN(A,Pos),Neg)

//求(A<<16 + B)的值，常用来求32位功能码的值
//如：A_SHIFT16_PLUS_B(FunCodeUnion.code.PL_PosSecCmxHigh,FunCodeUnion.code.PL_PosSecCmxLow)
#define A_SHIFT16_PLUS_B(A,B)   (((Uint32)(A) << 16) + (Uint32)(B))

//仅对除以非2的除法四舍五入使用
#define Sign_NP(A)   (((A)>=0) ? 1 : -1)



/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */

/* Inline Function --------------------------------------------------------*/
/* 内联函数定义 */
/*******************************************************************************
  函数名: Static_Inline void DELAY_US(Uint32 Nus)
  描述：延时函数
  测试函数：
            ...
            DINT;
            DelayCnt1 = GetSysTime_168MHzClk();
            DELAY_US(i);
            DelayCnt2 = GetSysTime_168MHzClk();
            EINT;
            DelayCnt3[i] = DelayCnt2 - DelayCnt1;
            ...
  测试结果：    输入        时间us
                0           14/168=0.08
                1           184/168=1.09
                2           346/168=2.05
                3           510/168=3.03
                4           672/168=4.00
                5           836/168=4.97
                6           998/168=5.94
                7           836/168=6.91
                8           836/168=7.88
                9           1488/168=8.85
                10          1650/168=9.82
                11          1814/168=10.79
                15          2466/168=14.67
                20          3280/168=19.52
                25          4096/168=24.38
                30          4910/168=29.22
                35          5726/168=34.08
                40          6540/168=38.92
                50          8170/168=48.63
                60          9800/168=58.33
                70          11430/168=68.03
                80          13060/168=77.73
                90          14690/168=87.44
                1000        15506/168=92.29 
********************************************************************************/
Static_Inline void DELAY_US(Uint32 Nus)
{    
   Uint16 i=0;
   while(Nus--)
   {
      i = 31;    //测试结果如上
      while(i--);
   }
}

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 




#ifdef __cplusplus
}
#endif 

#endif /* end of PUB_GlobalPrototypes */

/********************************* END OF FILE *********************************/




