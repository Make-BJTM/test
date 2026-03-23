#ifndef CANOPENHOMING_H
#define CANOPENHOMING_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */

#include "PUB_GlobalPrototypes.h"
/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */	

/* Exported_Constants --------------------------------------------------------*/
/* 宏定义 常数类*/
#define CanopenHomeDflts  {0, 0, 0, 0, 0,0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0, 0, 0, 0, 0, \
                           0,0}
#define HIGH     2
#define LOW      1
#define ZERO     0													 

#define POS 0
#define NEG 1
#define VALID 1
#define INVALID 0

#define HIGHSPPENULL  0x00
#define HIGHSPEED0    0x10  //高速找零初始化
#define LIMTSWSTEP1   0x11  //限位开关检测
#define LIMTSWSTEP2   0x12  //限位段位置指令处理
#define LIMTSWSTEP3   0x13  //减速段遇到限位信号位置指令处理

#define DECESWSTEP1   0x14  //减速信号检测						   
#define DECESWSTEP2   0x15  //减速信号检测
#define DECESWSTEP3   0x16  //减速段位置指令处理
#define DECESWSTEP4   0x17  //减速段遇到限位信号后的减速段位置指令处理

#define REVSWSTEP0    0x1A  //反向点信号检测
#define REVSWSTEP1    0x1B  //反向停止
#define REVSWSTEP2    0x20  //反向点位置指令处理
#define FINDINDESTEP1 0x21  //找零开始点检测 不用Z信号做原点，回零完成
#define FINDINDESTEP2 0x22  //Z信号检测
#define HOMEOK        0x23  //回零OK，停机处理
#define HOMINSUCCESS   0x30

#define START0         0x00
#define START1         0x01


#define INDETERMINATE   2

#define LOGIC_POS   3    //旧信号高位 新型号低位 01 上升沿  11高电平 00 低电平 10 下降沿
#define LOGIC_NEG   0
#define LOGIC_RISE  1
#define LOGIC_DOWN  2
#define LOGIC_NULL  4
/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */

typedef struct{
Uint16 Logic :2;    //电平有效逻辑
Uint16 Flag  :1;    //有效后标志
Uint16 RunDir:1;    //运行行方向
Uint16 RunSpd:2;    //运行速度
Uint16 Rsvd  :10;
}STRHOMEKEY_BIT;
typedef union{
    Uint16 all;
    volatile STRHOMEKEY_BIT bit;
}UNI_HOMEKEY;

typedef struct{
Uint16 ZIndex:1;
Uint16 HomingAttain:1;
Uint16 HomingError:1;
Uint16 HomingEn:2;
Uint16 OpModeSpebit:1;
Uint16 HomeKey   :2   ; 
Uint16 HomeLimt  :2   ; //原点信号      旧信号高位 新型号低位
Uint16 HMoveDir:1;
Uint16 ZeroSpeed:1;
Uint16 HomingClrFlag:1;//by huangxin201711_13位置环找到零点标志位
Uint16 Rsvd:3;
}STRHOMINGSTATUS_BIT;

typedef union{
    Uint16 all;
    volatile STRHOMINGSTATUS_BIT bit;
}UNI_HOMINGSTATUS;

typedef struct{
Uint16  StartLogic_LowSpd:2; //减速开关判断
Uint16  LimtSwtich_RevHSpd:2; //是否需要高速反向找零 0,无高速过程 1，有高速过程，2未确定
Uint16  RunEn:1;
Uint16 Rsvd:11;
}STRHOMINGFLAG_BIT;

typedef union{
    Uint16 all;
    volatile STRHOMINGFLAG_BIT bit;
}UNI_HOMINGFLAGE;

typedef struct{
    UNI_HOMINGSTATUS HomingStatus;
    UNI_HOMINGFLAGE  HomingFlag;
	UNI_HOMEKEY      DeceSwtich;
	UNI_HOMEKEY      RevSwtich ;
	UNI_HOMEKEY      FindIndex ;
	UNI_HOMEKEY      LimtSwtich;
    int64 HomeOffsetInc;//607Ch   int32
    int8  HomingMethod ;//6098h     int8
    Uint8 PosCalMethod ;//60E6  Uint8 
    int64  PulseDuringSearchForSwitchQ16;//

	int64  HomingSpeedRefQ16;
	int64  HomingSpeedOutputQ16;
	int32  HomingSpeedOutput;
	int32  HomingSpeedRem;

	int64  HomingAccQ16;

    
    int64  PulseDuringSearchForZeroQ16;//
    Uint32 HomingAccelerationInc;//609Ah
    Uint32 SearchTime;
    Uint32 TimeCount;
    
    Uint32 RiseDownTime;
    Uint64 RiseDownPulseQ16;
//    int64  Dist;
//    int64  DistQ16;
    Uint8  MoveStart; //启动标志，0：未启动；1：启动
    
    Uint8  MoveStatus;//运动状态
    Uint8  FindIndexFlag;//找零标志
    //Uint8  DownFlag;//经历过高速减速标志
    //Uint8  OTFlag;//碰到OT开关标志
    Uint8  HomingStep;   //回零步骤
    Uint8  DeceleratetoZero;  //是否减速到零
	Uint8  ReverencetoZero;  //是否减速到零
    
    Uint8  FindIndexStartFlag;
}STR_HOMINGVARIBLES;
/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */

extern STR_HOMINGVARIBLES STR_CanopenHome;
/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void CanopenHomingInit(void);

extern void CanopenHomingModeFuncVarStop(void);

extern void CanopenHomeRunUpdata(void);

extern void CanopenZeroIndexISR(void);

extern void CanopenHomingModeFunc(void);

extern int32 CanopenHomingPosCmd(void);

extern void CanopenHomingReset(void);

extern void HomingSwtichHandle(void);

extern void HomingHandle(void);
extern void HomingClearPos(void);
#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of FUNC_GlobalVariable.h*/

/********************************* END OF FILE *********************************/




