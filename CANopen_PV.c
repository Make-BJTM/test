
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "CANopen_PV.h"
#include "FUNC_GlobalVariable.h"
#include "FUNC_FunCode.h"
#include "FUNC_InterfaceProcess.h"
#include "FUNC_SpdCtrl.h"
#include "CANopen_Pub.h"
#include "FUNC_StopProcess.h"
#include "FUNC_ServoMonitor.h"
#include "CANopen_DeviceCtrl.h"
#include "FUNC_ErrorCode.h"
#include "FUNC_Filter.h" 
STR_BILINEAR_LOWPASS_FILTER     CANSpeedDisplayFilter = BILINEAR_LOWPASS_FILTER_Defaults;  //定义电机实际输出转速显示滤波器
STR_BILINEAR_LOWPASS_FILTER     CANSpeedDoFilter = BILINEAR_LOWPASS_FILTER_Defaults;       //定义转速DO滤波器
/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/


/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */





/*******************************************************************************
  函数名: extern  void InitCanopenPVMode(void) 
  输入:   无 
  输出:   
  子函数: 无       
  描述:   PV模式初始化
********************************************************************************/ 
void InitCanopenPVMode(void)
{
    /* 速度显示滤波器初始化 */
    CANSpeedDisplayFilter.Ts = 1000;  //速度显示滤波的Ts采样时间按1K进行采样 (((Uint32)1000000 << 10) / 1000) >> 10;
    CANSpeedDisplayFilter.Tc = 1000L * FunCodeUnion.code.ER_SpdDispFilt;
    CANSpeedDisplayFilter.InitLowPassFilter(&CANSpeedDisplayFilter);


    /* 速度Do滤波器初始化 */
    CANSpeedDoFilter.Ts = 1000;  //滤波的Ts采样时间按1K进行采样 (((Uint32)1000000 << 10) / 1000) >> 10;
    CANSpeedDoFilter.Tc = 1000L * FunCodeUnion.code.ER_SpdDoFilt;
    CANSpeedDoFilter.InitLowPassFilter(&CANSpeedDoFilter);
    
}

/*******************************************************************************
  函数名: extern  void CanSpdCtrlUpdate(void) 
  输入:   无 
  输出:   
  子函数: 无       
  描述:   PV模式初始化
********************************************************************************/ 
void CanSpdCtrlUpdate(void)
{
    /* 速度显示滤波器初始化 */
    CANSpeedDisplayFilter.Ts = 1000;  //速度显示滤波的Ts采样时间按1K进行采样 (((Uint32)1000000 << 10) / 1000) >> 10;
    CANSpeedDisplayFilter.Tc = 1000L * FunCodeUnion.code.ER_SpdDispFilt;
    CANSpeedDisplayFilter.InitLowPassFilter(&CANSpeedDisplayFilter);

    /* 速度Do滤波器初始化 */
    CANSpeedDoFilter.Ts = 1000;  //滤波的Ts采样时间按1K进行采样 (((Uint32)1000000 << 10) / 1000) >> 10;
    CANSpeedDoFilter.Tc = 1000L * FunCodeUnion.code.ER_SpdDoFilt;
    CANSpeedDoFilter.InitLowPassFilter(&CANSpeedDoFilter);
    
}
/*******************************************************************************
  函数名: void CanopenOEMAccLmt (); 
  输入:   无
  输出:   无
  子函数: void CanopenUserAccLmt (Uint32 Acc,Uint32 Dec,Uint32 Quick_Stop_Dec)       
  描述:   根据加速度限定60C5,减速度限定60C6,得出标准加速度、标准减速度、急停减速度限定值
********************************************************************************/ 
void ECTPVAccLmt (void)
{
    Uint64 MaxSpdDelta = 0;
    
    MaxSpdDelta = ((Uint64)FunCodeUnion.code.MT_MaxSpd * 10000L)<<17;
    
    //加速度
    STR_LmtVar.Vel_ProfAccQ16      =  UserAccUnit2Rpmps2Unit(&STR_Acc_Factor,ObjectDictionaryStandard.ProPosMode.ProfileAcceleration);
    STR_LmtVar.Vel_ProfDecQ16      =  UserAccUnit2Rpmps2Unit(&STR_Acc_Factor,ObjectDictionaryStandard.ProPosMode.ProfileDeceleration);
    STR_LmtVar.Vel_QuickStopDecQ16 =  UserAccUnit2Rpmps2Unit(&STR_Acc_Factor,ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration);


    if(STR_LmtVar.Vel_ProfAccQ16 > MaxSpdDelta)STR_LmtVar.Vel_ProfAccQ16 = MaxSpdDelta;
    if(STR_LmtVar.Vel_ProfDecQ16 > MaxSpdDelta)STR_LmtVar.Vel_ProfDecQ16 = MaxSpdDelta;
    if(STR_LmtVar.Vel_QuickStopDecQ16 > MaxSpdDelta)STR_LmtVar.Vel_QuickStopDecQ16 = MaxSpdDelta;


    //② 区分不同情况下的加速度
    //快速停车
    if(STR_ServoMonitor.StopCtrlFlag.bit.CanQuickStopAck == VALID)
    {
        if(STR_ServoMonitor.StopCtrlVar.CanQuickStopMode == 5)
        {
            STR_SpdCtrl.DeltaSpeedRise_Q16 = STR_LmtVar.Vel_ProfAccQ16;
            STR_SpdCtrl.DeltaSpeedDown_Q16 = STR_LmtVar.Vel_QuickStopDecQ16;
        } 
        else if(STR_ServoMonitor.StopCtrlVar.CanQuickStopMode == 4)
        {
            STR_SpdCtrl.DeltaSpeedRise_Q16 = STR_LmtVar.Vel_ProfAccQ16;
            STR_SpdCtrl.DeltaSpeedDown_Q16 = STR_LmtVar.Vel_ProfDecQ16;
        }
    }
    //暂停
    else if(STR_ServoMonitor.StopCtrlFlag.bit.CanHaltStopAck == VALID)
    {
        if(STR_ServoMonitor.StopCtrlVar.CanHaltStopMode == 5)
        {
            STR_SpdCtrl.DeltaSpeedRise_Q16 = STR_LmtVar.Vel_ProfAccQ16;
            STR_SpdCtrl.DeltaSpeedDown_Q16 = STR_LmtVar.Vel_QuickStopDecQ16;
        } 
        else if(STR_ServoMonitor.StopCtrlVar.CanHaltStopMode == 4)
        {
            STR_SpdCtrl.DeltaSpeedRise_Q16 = STR_LmtVar.Vel_ProfAccQ16;
            STR_SpdCtrl.DeltaSpeedDown_Q16 = STR_LmtVar.Vel_ProfDecQ16;
        }
    }
    //周期同步速度模式除快速停机、暂停、模式切换，正常运行时无减速时间    
    else if((STR_CanSyscontrol.Mode == ECTCSVMOD)&&((STR_CanSyscontrol.ModeSwitchFlag != 1))) 
    {
        STR_SpdCtrl.DeltaSpeedRise_Q16 = MaxSpdDelta;
        STR_SpdCtrl.DeltaSpeedDown_Q16 = MaxSpdDelta;
    }
    //PV正常运行
    else
    {
        STR_SpdCtrl.DeltaSpeedRise_Q16 = STR_LmtVar.Vel_ProfAccQ16;
        STR_SpdCtrl.DeltaSpeedDown_Q16 = STR_LmtVar.Vel_ProfDecQ16;
    }
}

/*******************************************************************************
  函数名: extern  ECTInteruptSpdShow (void)
  输入:   无
  输出:   STR_OEMVaribles.OEMVel
  子函数:      
  描述:  根据负载速度给定，计算电机速度给定
********************************************************************************/ 
void ECTInteruptSpdShow(void)
{
    int32 SpdTemp111 = 0;

    if(STR_FUNC_Gvar.MonitorFlag.bit.ESMState==0)return;
    
    SpdTemp111 = UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb / 1000;

    //计算606C速度反馈（用户单位），内部速度反馈保留一位小数点参与运算
    SpdTemp111 = Rpm2UserVel(&STR_Vel_Factor,SpdTemp111);
    ObjectDictionaryStandard.ProVelMode1.VelocityActualValue = (int32)(SpdTemp111/10);
    

}


/*******************************************************************************
  函数名: extern int32 CANopenSpdMonitor (void)
  输入:   无
  输出:   STR_OEMVaribles.OEMVel
  子函数:      
  描述:  根据负载速度给定，计算电机速度给定
********************************************************************************/ 
void CANopenSpdMonitor (void)
{
    int32 SpdReachVal;
    Uint16 SpdReachCnt;
    int64 temp111=0;
    static Uint32 Cnttemp2=0;
    int32 SpdTemp111 = 0;

    
    // 实际电机转速输出显示需要进行滤波处理
    if(STR_FUNC_Gvar.MonitorFlag.bit.ESMState==0)
    {
        CANSpeedDisplayFilter.Input = UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb;
        CANSpeedDisplayFilter.LowPassFilter(&CANSpeedDisplayFilter);
        SpdTemp111 = CANSpeedDisplayFilter.Output;
        SpdTemp111 = (SpdTemp111 + Sign_NP(SpdTemp111) * 500) / 1000;    //H0B55_速度显示0.1rpm计算（加滤波）

        //计算606C速度反馈（用户单位），内部速度反馈保留一位小数点参与运算
        temp111 = Rpm2UserVel(&STR_Vel_Factor,SpdTemp111);
        ObjectDictionaryStandard.ProVelMode1.VelocityActualValue = (int32)(temp111/10);
    }
    
    //速度一致 判断
    if( (STR_FUNC_Gvar.MonitorFlag.bit.RunMod != SPDMOD) || (STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus != RUN) )
    {
        STR_FUNC_Gvar.SpdCtrl.DovarReg_VCmp = 0;
    }
    else
    {
        // 实际电机转速输出显示需要进行滤波处理
        CANSpeedDoFilter.Input = UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb / 100;
        CANSpeedDoFilter.LowPassFilter(&CANSpeedDoFilter);

        //计算速度一致阈值606Dh
        SpdReachVal = (int32)ObjectDictionaryStandard.ProVelMode1.VelocityWindow;//rpm
        //速度一致判断窗口时间606Eh(ms)
        SpdReachCnt = ObjectDictionaryStandard.ProVelMode1.VelocityWindowTime;

        SpdTemp111 = CANSpeedDoFilter.Output - (STR_SpdCtrl.SpdCmdLatch / (int32)100);
        SpdTemp111 = ABS(SpdTemp111 / (int32)100);
        
        if(SpdTemp111 <= SpdReachVal)
        {
            if(Cnttemp2 >= SpdReachCnt)
            {
                Cnttemp2 = SpdReachCnt;
                STR_FUNC_Gvar.SpdCtrl.DovarReg_VCmp = 1;
            }
            else
            {
                Cnttemp2 ++;
                STR_FUNC_Gvar.SpdCtrl.DovarReg_VCmp = 0;
            }

        }
        else
        {
            Cnttemp2 = 0;
            STR_FUNC_Gvar.SpdCtrl.DovarReg_VCmp = 0;
        }

    }
    
    if((STR_CanSyscontrol.Mode == ECTSPDMOD)||(STR_CanSyscontrol.Mode == ECTCSVMOD))
    {
        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = STR_FUNC_Gvar.SpdCtrl.DovarReg_VCmp;
    }
}
/********************************* END OF FILE *********************************/

