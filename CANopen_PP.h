#ifndef CANOPENPROFILEPOSITION_H
#define CANOPENPROFILEPOSITION_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h"
#include "CANopen_OD.h"


/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */	


/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */
typedef struct{   
    //Uint8  PPPosCmdType;
//	Uint8   PPOperModLatch;
	Uint16 	CtrlWord;

    Uint8  ChangeSetImmediately;        //0-不立刻切换  1-立刻切换
    Uint8  New_SetPoint;                //0-无新的位置指令  1-有新的位置指令
    Uint8  New_SetPointLatch;           //0-无新的位置指令  1-有新的位置指令
    Uint8  CmdBusy;                     //0-不忙  1-忙
    Uint8  PosBuffSt;
	int32  TargetPos;                   //607Ah    int32
    int32  TargetPosLatch;
//    int32  CurrentAbsPosUserUnit;
    int64  DeltaPosIncQ16;
    //int32  UserPosInputLatch;  //已执行的用户单位位置指令累加值
    Uint32 FollowingErrorWindow;//6065h Uint32
	Uint32 FollowingErrorTimeOut;
	Uint32 FollowingErrorCnt;

    Uint32 PositionWindow;//6067h           Uint32
    Uint32 PositionWindowTime;//6068h       Uint16
    int16  MotionProfileType;//6086h     int16
    int64  HaltReLenQ16;
    Uint8  PosCmdReachLimit;
    Uint8  PosFedReachPosLimit;
    Uint8  PosFedReachNegLimit;
	int64  LPStartSPD;
	int64  LPStopSPD;
    Uint8  AbsPosActSet;
    Uint8  PosWinUnitSet;
}STR_POSITIONCONTROLVARIBLES;

typedef struct{   

    Uint8  HaltPosRefOk;//暂停段已将指令执行完标志，若未执行完，则需要重新初始化
	//int64  PPStartPulseQ16;           //启动速度对应每个插补周期的脉冲个数, Q16---1rpm
	//int64  PPStopPulseQ16;            //停止速度对应每个插补周期的脉冲个数, Q16---1rpm
	int64  PPUpPulseRevQ16;           //加速脉冲增量每个插补周期, Q16
	int64  PPAvergePulseQ16;          //匀速脉冲个数每个插补周期, Q16
		
    int64  PPDownPulseRevQ16;         //减速脉冲增量每个插补周期, Q16
	int64  PPLineLengthQ16;           //直线段总长度, Q16
    

}STR_POSHALTVARIBLES;

//PP模式缓存结构体
typedef struct{
    int32  UserTargetPos;       //
    Uint32 UserProVel;
    Uint32 UserProAcc;
    Uint32 UserProDec; 
    int8   PPPosCmdType;      //
}STR_SETPOINTBUFFER;


/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */

extern STR_POSITIONCONTROLVARIBLES STR_PosCtrlVar;
extern STR_POSHALTVARIBLES STR_PosHaltVar;

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void CanopenPPModeStopUpdata(void);
extern void InitCanopenPPMode(void);
extern void CanopenPPModeUpdate(void);
extern void CanopenPosArrive(int32 PosAmplifErrTemp);
extern int32 CanopenPPModePosRefCal (void);
extern void CanopenClrPosReg(void);
extern void CanopenPosMonitor(int32 PosAmplifErrTemp);
extern void CanopenHomeReset(void);
extern void CanopenPPReset(void);
extern void HaltVarReset(void);
extern void PosLatchUpdate(void);

extern void CanopenPPPosBuffReset(void);
#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of FUNC_GlobalVariable.h*/

/********************************* END OF FILE *********************************/









