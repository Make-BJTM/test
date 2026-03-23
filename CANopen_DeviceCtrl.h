#ifndef CANOPENDEVICECONTROL_H
#define CANOPENDEVICECONTROL_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */

#include "PUB_GlobalPrototypes.h"
#include "CANopen_OD.h"
#include "FUNC_ServoMonitor.h"

/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */	

/* Exported_Constants --------------------------------------------------------*/
/* 宏定义 常数类*/

//控制字宏指令
#define  CMD_SHUTDOWN0          0x06
#define  CMD_SHUTDOWN1          0x0E

#define  CMD_SWITCHON0          0x07
#define  CMD_SWITCHON1          0x0F

#define  CMD_DISABLEVOLTAGE0    0x00
#define  CMD_DISABLEVOLTAGE1    0x01
#define  CMD_DISABLEVOLTAGE2    0x04
#define  CMD_DISABLEVOLTAGE3    0x08
#define  CMD_DISABLEVOLTAGE4    0x05
#define  CMD_DISABLEVOLTAGE5    0x09
#define  CMD_DISABLEVOLTAGE6    0x0C
#define  CMD_DISABLEVOLTAGE7    0x0D

#define  CMD_QUICKSTOP0         0x02
#define  CMD_QUICKSTOP1         0x03
#define  CMD_QUICKSTOP2         0x0A
#define  CMD_QUICKSTOP3         0x0B

#define  CMD_DISABLEOPERATION   0x07
#define  CMD_ENABLEOPENRATION   0x0F

#define  FAULTESET              0x10

//状态字宏指令 控制字低7位
#define  NOTREADYTOSWITCHON      0x00 //000 0000
#define  SWITCHONDISABLEED       0x40 //100 0000
#define  READYTOSWITCHON         0x21 //010 0001
#define  SWITCHON                0x23 //010 0011
#define  OPERATIONENABLE         0x27 //010 0111
#define  QICKSTOPACTIVE          0x07 //000 0111
#define  FAULTREACTIONACTIVE     0x0F //000 1111
#define  FAULT                   0x08 //001 1000
//状态字宏指令 控制字低7位 OVER

//状态机状态宏定义
#define  STATUS_NOTRDYSWICHON     0x01
#define  STATUS_SWITCHONDIS       0x02
#define  STATUS_RDYTOSWITCHON     0x03
#define  STATUS_SWITCHON          0x04
#define  STATUS_OPERENABLE        0x05
#define  STATUS_QUICKSTOPACTIVE   0x06
#define  STATUS_FAULTACTIVE       0x07
#define  STATUS_FAULT             0x08
#define  STATUS_HALT              0x09

#define  STATUS_NRD_FAULT         0x0A
#define  STATUS_NRD_SWITCHONDIS   0x0B
#define  STATUS_NRD_RDYTOSWITCHON 0x0C
//状态机状态宏定义OVER

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */

typedef struct{
    int16 QuickStopOptionCode;//605Ah  int16
    volatile UNI_CONTROLWORD ControlWordBack;//控制字锁存
    Uint16 AlmRstFlag;
	Uint16 FaultFlagLast;
	Uint16 WarningFlagLast;
    Uint16 VoltageEnFlag;
    Uint16 SwitchOnFlag;
    Uint16 OperationEnFlag;
    Uint16 StatusStep;
    Uint16 OEMStatus;
    Uint16 QuickStopFinishFlag;
    Uint16 PosSlopeStopActiveFlag;
    Uint16 HaltFinishFlag;

}STR_DEVICECONTROLVAR;

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */
extern STR_DEVICECONTROLVAR DeviceControlVar;

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void CanopenSupportedDriveModeSet(void);
extern void CanopenDIRefresh1k(void);
extern void CanopenDIRefresh4k(void);
extern void ComControlBK(void);
extern void CanopenDeviceControlStop(void);
extern void CanopenDeviceControlFunc(void);
extern void CanopenQuickStopDeal(STR_SERVO_MONITOR *p);
extern void CanopenQuickStopProcess(void);
extern void CANopenPosLimt(void);
//extern void NodeStateCtrl(void);
//extern void ComErrStateCtrl(void);
extern int32 CanopenPosModeSlopeStop (void);
extern void CanopenHalt(void);
extern void CanopenHaltDeal(STR_SERVO_MONITOR *p);
extern void MCZindex(void);
extern void MCZindexClear(void);


#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of FUNC_GlobalVariable.h*/

/********************************* END OF FILE *********************************/
