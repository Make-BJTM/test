
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "CANopen_Pub.h"
#include "FUNC_GlobalVariable.h"
#include "FUNC_InterfaceProcess.h"
#include "FUNC_FunCode.h"
#include "CANopen_DeviceCtrl.h"
#include "CANopen_PP.h"
#include "CANopen_Home.h"
#include "CANopen_PV.h"
#include "FUNC_ErrorCode.h" 
#include "FUNC_PosCtrl.h"          
#include "FUNC_SpdCtrl.h"
#include "PUB_Library_Function.h"

/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/


//pCanopenIntplt[0]指向PP
//pCanopenIntplt[1]指向HM
//pCanopenIntplt[2]指向暂停
STR_POSCTRLLINEINTERPLT* pCanopenIntplt[3] = {0, 0, 0};

// 重新规划减速段时用到以下三个变量
static int64 PPdiff = 0;     // 总差值
static int64 PPdiff2 = 0;    // 总差值的绝对值
static int64 PPcompsnt = 0;  // 分散到每次插补时的补偿值
/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
STR_ACC_FACTOR                   STR_Acc_Factor;
STR_VELOCITYENCODER_FACTOR       STR_VelEnc_Factor;
STR_VELOCITY_FACTOR              STR_Vel_Factor;
STR_POSITION_FACTOR              STR_Pos_Factor;
STR_POSITION_FACTOR_INVERSE      STR_Pos_Factor_Inverse;
STR_LIMITVARIBLES                STR_LmtVar;
STR_CANSYSCONTR                  STR_CanSyscontrol;
/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */

/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */
Static_Inline int64 PPAccDistCalc(int64 PPStartPulseQ16, int64 PPStopPulseQ16, int64 PPPulseRevQ16);
Static_Inline int64 PPMaxPulseRecalQ16(int64 PPStartPulseQ16, int64 PPStopPulseQ16, int64 RisePulseQ16,int64 DownPulseQ16, int64 LineDistQ16);

/*******************************************************************************
  函数名: void CANopenFactorUpdate(void)
  输入:   无 
  输出:   
  子函数: 无       
  描述:   转换因子实时更新
********************************************************************************/
void FactorUpdate(void)
{
    Uint64 Temp111 = 0;
    Uint64 Temp222 = 0;

    //6093只更新值，不进行计算，在计算驱动器位置指令处，要进行余数运算，以保证位置指令精度  //由于历史原因，名称写的是6093，实际是6091
	//6093h：position factor，指令单位/s-->p
    STR_Pos_Factor.PosFactor6093_Numerator           = ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev;
    STR_Pos_Factor.PosFactor6093_Denominator         = ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;

    STR_Pos_Factor_Inverse.PosFactor6093_Numerator   = ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev;
    STR_Pos_Factor_Inverse.PosFactor6093_Denominator = ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;

    //Vel_VelEncoder6094Q12：Velocity encoder factor，指令单位/s-->p/s 转化成驱动器速度指令---速度控制 10000*rpm
    STR_VelEnc_Factor.VelEncoder6094_Numerator = ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev;
    STR_VelEnc_Factor.VelEncoder6094_Denominator = ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;

    //6095h：Velocity factor 1，rpm-->user velocity unit
    //6095_Inverse:6095倒数，user velocity unit-->rpm
    STR_Vel_Factor.VelFactor6095_Numerator = ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;
    STR_Vel_Factor.VelFactor6095_Denominator =ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev;


    //Vel_AccFactor6097Q14：Acceleration factor，user acceleration unit--p/s2 转化成驱动器每个控制周期的速度指令增量--速度控制 10000*rpm  
    STR_Acc_Factor.AccFactor6097_Numerator = ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev;
    STR_Acc_Factor.AccFactor6097_Denominator = ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;

    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus != RUN)return;

	if(ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev==0)PostErrMsg(GEALSETERR3);

    if(ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev >= ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev)
    {
        Temp111 = (Uint64)ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev * (Uint64)10L;
        Temp222 = (Uint64)4L * (Uint64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev * (Uint64)ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev;
        if(Temp111>Temp222)PostErrMsg(GEALSETERR3);
    }
    else
    {
        if(ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev !=0)
        {
            Temp111 = (Uint64)ObjectDictionaryStandard.FactorGroup.GearRatio.ShaftRev * (Uint64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev;
            Temp222 = (Uint64)ObjectDictionaryStandard.FactorGroup.GearRatio.MotorRev * (Uint64)1000L *(Uint64)10000L;
            if(Temp111>Temp222)PostErrMsg(GEALSETERR3);
        }
        else
        {
            PostErrMsg(GEALSETERR3);
        }
    }
} 
/*******************************************************************************
  函数名:  int64 UserPosUnit2IncpUnit(STR_POSITION_FACTOR *posFactor,int32 posInput)
  输入:    STR_Pos_Factor.InPut;                    //位置因子前输入量
           STR_Pos_Factor.PosRemainder;            //位置因子计算余数
           STR_Pos_Factor.PosFactor6093_Numerator;
           STR_Pos_Factor.PosFactor6093_Denominator;
  参数：       
  输出:    Result       
  描述:    将用户位置转化成驱动器目标位置     6093     
********************************************************************************/ 
int64 UserPosUnit2IncpUnit(STR_POSITION_FACTOR *posFactor,int32 posInput)
{
    int64  Sum    = 0; //位置指令计算中间变量 
    int64  Result = 0;     
    int64  Temp   = 0;
    
    if (posFactor->PosFactor6093_Denominator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }

	//接收输入指令
    posFactor->InPut = posInput;    

    /*计算放大后的位置指令加上上次计算的余数*/	
    Sum = ((int64)posFactor->InPut) * ((int64)posFactor->PosFactor6093_Numerator); 
    Sum += (int64)posFactor->PosRemainder;  
    
    /*计算乘以位置转换因子后的位置指令*/  
    Result = (Sum / posFactor->PosFactor6093_Denominator);

    /*计算余数*/         
    Temp = Result * posFactor->PosFactor6093_Denominator; 
    posFactor->PosRemainder = Sum - Temp;     

    return Result;    
}

/*******************************************************************************
  函数名:  int32 IncpUnit2UserPosUnit(STR_POSITION_FACTOR_INVERSE *posFactor,int32 posInput)
  输入:    STR_Pos_Factor_Inverse.InPut;                    //位置因子前输入量
           STR_Pos_Factor_Inverse.PosRemainder;            //位置因子计算余数
           STR_Pos_Factor_Inverse.PosFactor6093_Numerator;
           STR_Pos_Factor_Inverse.PosFactor6093_Denominator;
  参数：       
  输出:    Result       
  描述:    将驱动器位置转化为用户位置，仅在位置显示用    1/6093
********************************************************************************/ 
int32 IncpUnit2UserPosUnit(STR_POSITION_FACTOR_INVERSE *posFactorinverse,int64 posInput)
{
    int32  Result    = 0;     
	int64 Temp111 = 0;

    if (posFactorinverse->PosFactor6093_Numerator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }


	//接收输入指令
    posFactorinverse->InPut = posInput;    

	Temp111 = (int64)posFactorinverse->InPut * posFactorinverse->PosFactor6093_Denominator;

	Temp111 = Temp111/posFactorinverse->PosFactor6093_Numerator;

	Result  = (int32)Temp111;

    return Result;    
}
/*******************************************************************************
  函数名:   int64 UserVelUnit2IncpsUnit(STR_VELOCITYENCODER_FACTOR *velFactor,Uint32 velInput)
  输入:     STR_Vel_Factor_Pos.InPut;
            STR_Vel_Factor_Pos.VelEncoder6094_Numerator
            STR_Vel_Factor_Pos.VelEncoder6094_Denominator

  参数：       
  输出:    Result       
  描述:    将用户速度指令单位转化成驱动器位置指令单位:p/位置环控制周期  6094      
********************************************************************************/ 
int64 UserVelUnit2IncpsUnit(STR_VELOCITYENCODER_FACTOR *velFactor,Uint32 velInput)
{
    int64  ResultQ16 = 0;
//    Uint64 Temp111 = 0;
    
    if (velFactor->VelEncoder6094_Denominator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }
    velFactor->InPut = velInput;  

    //为保证精度，必须调整计算顺序
    ResultQ16 = (((Uint64)velFactor->InPut<<12)/(Uint64)STR_FUNC_Gvar.System.PosFreq) 
                * (Uint64)velFactor->VelEncoder6094_Numerator
                /(Uint64)velFactor->VelEncoder6094_Denominator;

    ResultQ16 = ResultQ16<<4;
    
    return ResultQ16;    
}
/*******************************************************************************
  函数名:  Uint64 UserAccUnit2Incps2Unit(STR_ACC_FACTOR *accFactor,Uint32 accInput)
  输入:    STR_Factors.Pos_AccFactor6097Q14;
  参数：       
  输出:    Result       
  描述:    将用户加速度单位转化成驱动器单位:p/位置环控制周期  Q16      6097
********************************************************************************/ 
Uint64 UserAccUnit2Incps2Unit(STR_ACC_FACTOR *accFactor,Uint32 accInput)
{
    Uint64  ResultQ16 = 0;
    Uint64  Temp = 0;
    Uint64 Temp111 = 0;

    if (accFactor->AccFactor6097_Denominator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }
    accFactor->InPut = accInput;  

    //为保证精度，必须调整计算顺序
	// 计算电机最大速度对应的每s插补周期脉冲个数 Q16
    if(FunCodeUnion.code.FC_FeedbackMode == 1)
    {
        Temp = (Uint64)FunCodeUnion.code.MT_MaxSpd * STR_InnerGvarPosCtrl.ExRPM2PPPtCoefQ16;
    }
    else
    {
        Temp = (Uint64)FunCodeUnion.code.MT_MaxSpd * STR_InnerGvarPosCtrl.RPM2PPPtCoefQ16;
    }
    
    Temp = Temp>> PPAMPBIT;
    Temp = Temp *(Uint64)STR_FUNC_Gvar.System.PosFreq * STR_FUNC_Gvar.System.PosFreq; 
    Temp111 = (Uint64)accFactor->InPut * (Uint64)accFactor->AccFactor6097_Numerator/(Uint64)accFactor->AccFactor6097_Denominator;
    if(Temp111>Temp)Temp111 = Temp;
    
    Temp = (Uint64)STR_FUNC_Gvar.System.PosFreq * STR_FUNC_Gvar.System.PosFreq;
    ResultQ16 = (Temp111<<10)/Temp;
    ResultQ16 = ResultQ16<<6;
    if(ResultQ16==0)ResultQ16 =1;//一个位置环至少一个脉冲，因此齿轮比为1时，对应的6083最小值为16000000                

    return ResultQ16;    
}

/*******************************************************************************
  函数名:   int32 UserVelUnit2RpmUnit(STR_VELOCITYENCODER_FACTOR *velFactor,int64 velInput)
  输入:    
            STR_Vel_Factor_Vel.InPut;
            STR_Vel_Factor_Vel.VelEncoder6094_Numerator
            STR_Vel_Factor_Vel.VelEncoder6094_Denominator
  参数：       
  输出:    Result       
  描述:    将用户速度单位转化成驱动器速度单位:rpm*10000         6094
********************************************************************************/ 
int32 UserVelUnit2RpmUnit(STR_VELOCITYENCODER_FACTOR *velFactor,int64 velInput)
{
    int64  Result = 0; 
    int32 Temp111 = 0;

    if (velFactor->VelEncoder6094_Denominator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }

    velFactor->InPut = velInput;

    velFactor->VelEncoder6094Q20 =(int64)(((((Uint64)60 * 10000L << 20)/UNI_FUNC_MTRToFUNC_InitList.List.EncRev) *
		                                    (int64)velFactor->VelEncoder6094_Numerator) / 
		                                    (int64)velFactor->VelEncoder6094_Denominator);

    Result = velFactor->InPut * velFactor->VelEncoder6094Q20 >> 20;

    Temp111 = (int16)FunCodeUnion.code.MT_MaxSpd * 10000L;

    if(Result>Temp111)Result = Temp111;
    else
    {
        Temp111 = - Temp111;
        if(Result < Temp111)Result = Temp111;
    }

    return (int32)Result;    
}
/*******************************************************************************
  函数名:  int32 Rpm2UserVel(STR_VELOCITY_FACTOR *velFactor,int32 velInput)
  输入:    STR_Pos_Factor.InPut;                    //位置因子前输入量
           STR_Pos_Factor.PosRemainder;            //位置因子计算余数
           STR_Pos_Factor.PosFactor6093_Numerator;
           STR_Pos_Factor.PosFactor6093_Denominator;
  参数：       
  输出:    Result       
  描述:    将驱动器速度单位(rpm)转化成用户速度单位,只在反馈的时候用   6095    
********************************************************************************/ 
int32 Rpm2UserVel(STR_VELOCITY_FACTOR *velFactor,int32 velInput)
{
    int32  Result    = 0; 
    int32 Temp111 = 0;

    
    if (velFactor->VelFactor6095_Denominator == 0)	  //当分母为零时退出该程序，返回的速度为零
    {
        return 0;
    }

    //将 rpm  转成  指令单位/s
    if(FunCodeUnion.code.FC_FeedbackMode == 1)
    {
        Temp111 = (int32)A_SHIFT16_PLUS_B(FunCodeUnion.code.FC_ExCoderPulse_H,FunCodeUnion.code.FC_ExCoderPulse_L);
        velFactor->VelFactor6095Q14 =(Uint64)velFactor->VelFactor6095_Numerator * (((Uint64)Temp111 << 14)/(Uint64)60L) 
     								 /((Uint64)velFactor->VelFactor6095_Denominator);
    }
    else
    {
        velFactor->VelFactor6095Q14 =(Uint64)velFactor->VelFactor6095_Numerator * (((Uint64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev << 14)/(Uint64)60L) 
     								 /((Uint64)velFactor->VelFactor6095_Denominator);
    }
	//接收输入指令
    velFactor->InPut = velInput;    

    //计算用户速度反馈	
    Result =(int32)((int64)velFactor->InPut * velFactor->VelFactor6095Q14 >> 14); 
    
    return Result;    
}

/*******************************************************************************
  函数名:  Uint64 UserAccUnit2Rpmps2Unit(STR_FACTORS *accFactor,Uint32 accInput)
  输入:    STR_Factors.Vel_AccFactor6097Q14;
  参数：       
  输出:    ResultQ10       
  描述:    将用户加速度单位转化成驱动器速度控制中的加速度单位:10000*rpm/速度环周期 Q10 6097    
********************************************************************************/ 
Uint64 UserAccUnit2Rpmps2Unit(STR_ACC_FACTOR *accFactor,Uint32 accInput)
{
    Uint64  ResultQ10 = 0;
    Uint64  Temp = 0;
//    Uint64 Temp111 = 0;
    Uint64 MaxSpdDelta = 0;
    
    if (accFactor->AccFactor6097_Denominator == 0)	  //当分母为零时退出该程序，返回的位置指令为零
    {
        return 0;
    }
    accFactor->InPut = accInput; 

    if(accFactor->InPut==0)accFactor->InPut = 1;
    
    MaxSpdDelta = (Uint64)FunCodeUnion.code.MT_MaxSpd * 10000L * UNI_FUNC_MTRToFUNC_InitList.List.EncRev<<1;

    Temp = (60L * 10000L<<1)/STR_FUNC_Gvar.System.SpdFreq;
    Temp = Temp * (Uint64)accFactor->AccFactor6097_Numerator/(Uint64)accFactor->AccFactor6097_Denominator;//31457280~7

    Temp = Temp * (Uint64)accFactor->InPut;
    //为了保证精度，必须扩大到2^16，以兼容23位编码器
    
    if(Temp>MaxSpdDelta)Temp = MaxSpdDelta;
    Temp = Temp<<15;
    ResultQ10 = Temp/(Uint64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev;
    if(ResultQ10==0)ResultQ10=1;

    return ResultQ10;    
}

/*********************************************************************
  函数名: int32 CANopenVelLmt (int32 Vel); 
  输入:   参数1：	Vel		      用户速度给定
          PV/CSV:   60FF
 描述:   根据速度限定607F,电机最大速度H0015,得出速度给定值
          对于速度控制----得出速度指令10000*rpm
********************************************************************************/
int32 CANopenVelLmt(int32 Vel)
{    
    int32 MaxUserVel = 0;
    int32 MaxMotorVel = 0;
    int32 MaxUserVelLmt = 0;
    int32  temp = 0;
    int32  temp111 = 0;

    MaxMotorVel = 10000L * (int16)FunCodeUnion.code.MT_MaxSpd;//H0015

	if(((FunCodeUnion.code.CM_ECATHost==2)||(FunCodeUnion.code.CM_ECATHost==1))
		&&(STR_CanSyscontrol.Mode == ECTCSVMOD))
	{
		MaxUserVel = MaxMotorVel;	
	}
	else
	{
	    MaxUserVel = UserVelUnit2RpmUnit(&STR_VelEnc_Factor,(Uint32)ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity); 
	}

	MaxUserVelLmt = ((MaxUserVel <= MaxMotorVel) ? MaxUserVel : MaxMotorVel);

    //最大速度限制，正向限制
    temp = MIN(Vel,MaxUserVelLmt);

    //最大速度限制，负向限制
    MaxUserVelLmt = -MaxUserVelLmt;
    temp111 = MAX(temp,MaxUserVelLmt);
    
  	return temp111;
}
/*********************************************************************
  函数名: int64 CanopenPosVelLmt(Uint32 Vel,Uint32 Max1,Uint32 Max2)
  输入:   参数1：	Vel		      用户速度给定
          PP:   6081
          PV:   60FF
          HM:   6099
********************************************************************************/
int64 CanopenPosVelLmt(Uint32 Vel,Uint32 MaxProfileVel)
{    
    int64 MaxPulseQ16 = 0;
    int64 temp = 0;
    Uint32 temp111 = 0;

	//取小者
	temp111 = ((Vel <= MaxProfileVel) ? Vel : MaxProfileVel);

    //限幅计算
    temp = UserVelUnit2IncpsUnit(&STR_VelEnc_Factor,temp111);

    if(FunCodeUnion.code.FC_FeedbackMode == 1)//非全闭环
    {
        MaxPulseQ16 = ((int64)(Uint16)FunCodeUnion.code.MT_MaxSpd) * STR_InnerGvarPosCtrl.ExRPM2PPPtCoefQ16;
    }
    else
    {
        MaxPulseQ16 = ((int64)(Uint16)FunCodeUnion.code.MT_MaxSpd) * STR_InnerGvarPosCtrl.RPM2PPPtCoefQ16;
    }

    if(temp>MaxPulseQ16)temp = MaxPulseQ16;

    return temp;
}
/*******************************************************************************
  函数名: void CanopenAccLmt (Uint32 Acc,Uint32 Dec) 
  输入:   参数1：	Acc		      标准加速度---6083(PP,PV,IP)  609A(HM)
          参数2：	Dec 	      标准减速度---6084(PP,PV,IP)  609A(HM) 
  输出:   无
  子函数: 无       
  描述:   根据加速度限定60C5,减速度限定60C6,得出标准加速度、标准减速度、急停减速度限定值
          位置模式下，段切换时，每周期位置指令增量更新
********************************************************************************/ 
void CanopenPosAccLmt (Uint32 Acc,Uint32 Dec,Uint32 QStopDec)
{

    Uint64 MaxUserAccQ16 = 0;
    Uint64 MaxUserDecQ16 = 0;
    Uint64 MaxUserQuick_Stop_DecQ16 = 0;
    Uint64 AccQ16 = 0;
	Uint64 DecQ16 = 0;
    Uint64 Quick_Stop_DecQ16 = 0;
    Uint64 MaxPulseQ16 = 0;

	if(Acc      == 0) Acc      = 1;
	if(Dec      == 0) Dec      = 1;
	if(QStopDec == 0) QStopDec = 1;
    
	// 计算电机最大速度对应的每个插补周期脉冲个数 Q16
    if(FunCodeUnion.code.FC_FeedbackMode == 1)//非全闭环
    {
        MaxPulseQ16 = ((Uint64)(Uint16)FunCodeUnion.code.MT_MaxSpd) * STR_InnerGvarPosCtrl.ExRPM2PPPtCoefQ16;
    }
    else
    {
        MaxPulseQ16 = ((Uint64)(Uint16)FunCodeUnion.code.MT_MaxSpd) * STR_InnerGvarPosCtrl.RPM2PPPtCoefQ16;
    }
    
    AccQ16 = UserAccUnit2Incps2Unit(&STR_Acc_Factor,Acc);
    DecQ16 = UserAccUnit2Incps2Unit(&STR_Acc_Factor,Dec);
    Quick_Stop_DecQ16 = UserAccUnit2Incps2Unit(&STR_Acc_Factor,QStopDec);

    MaxUserAccQ16      = ((AccQ16 <= MaxPulseQ16) ? AccQ16 : MaxPulseQ16);
    
    MaxUserDecQ16      = ((DecQ16 <= MaxPulseQ16) ? DecQ16 : MaxPulseQ16);
    
    MaxUserQuick_Stop_DecQ16  = ((Quick_Stop_DecQ16 <= MaxPulseQ16) ? Quick_Stop_DecQ16 : MaxPulseQ16);

    STR_LmtVar.Pos_ProfAccQ16 = MaxUserAccQ16;
    
    STR_LmtVar.Pos_ProfDecQ16 = MaxUserDecQ16;

    STR_LmtVar.Pos_QuickStopDecQ16 = MaxUserQuick_Stop_DecQ16;

}


/*******************************************************************************
  函数名: void CanopenPosLmt (int32 Pos); 
  输入:   PP---6064和607A(需转化成绝对位置)
          HM
          IP
  输出:   无
  子函数: 无       
  描述:   位置控制类，负载绝对位置限制，对position demand value和position actual value
********************************************************************************/ 
int32 CanopenPosLmt(int32 Pos)
{    
    if((STR_PosCtrlVar.AbsPosActSet==1)
        ||((STR_PosCtrlVar.AbsPosActSet==2)&&(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.HomeFind == 1)))
    {
        //software position limit 判断
        if(Pos >=STR_LmtVar.MaxPositionLimit)
        {
            STR_PosCtrlVar.PosCmdReachLimit = 1;
            Pos = STR_LmtVar.MaxPositionLimit;
        }
        else if(Pos <=STR_LmtVar.MinPositionLimit)
        {
            STR_PosCtrlVar.PosCmdReachLimit = 1;
            Pos = STR_LmtVar.MinPositionLimit;
        }
        else
        {
            STR_PosCtrlVar.PosCmdReachLimit = 0;
        }
    }
    else
    {
        STR_PosCtrlVar.PosCmdReachLimit = 0;
    }
    return Pos;
}
/*******************************************************************************
  函数名: void AccDistCalc()
  输  入: StartPulseQ16 - 起点处每个插补周期的插补脉冲数 
          StopPulseQ16  - 终点处每个插补周期的插补脉冲数
		  PulseRev   - 每个插补周期脉冲递增(减)量(加/减速度)
  输  出: 加(减)速段的长度(以脉冲为单位)   
  子函数:                                       
  描  述: 根据起点处每周期插补脉冲数StartPulse, 终点处每周期插补脉冲数EndPulse
          以及每个插补周期脉冲递增量, 计算加(减)速段的长度
		  如果增(减)量 PulseRev等于0, 则认为加(减)速段距离也为0
********************************************************************************/ 
Static_Inline int64 PPAccDistCalc(int64 PPStartPulseQ16, int64 PPStopPulseQ16, int64 PPPulseRevQ16)
{
	int64 DistQ16 = 0;
	Uint32 RunCount = 0; 

	// 计算从起点速度到终点速度的插补周期数
	// 注意: 插补开始时起步速度是 StartPulseQ16 + PulseRev
    if(PPPulseRevQ16)    
    {
		RunCount = (PPStopPulseQ16 - PPStartPulseQ16) / PPPulseRevQ16 + 1;
	}
	else
	{
	    RunCount = 0; // 此时认为速度无需改变, 相应加速段距离也为0
	}
    // 计算加(减)速段长度
	DistQ16 = ((int64)RunCount * (PPStartPulseQ16 + ((int64)PPPulseRevQ16 * (RunCount-1)>>1)));	

    return DistQ16 ;
}

/*******************************************************************************
  函数名: void AccDistCalc()
  输  入: StartPulseQ16 - 起点处每个插补周期的插补脉冲数 
          StopPulseQ16  - 终点处每个插补周期的插补脉冲数
		  RisePulseQ16  - 每个插补周期脉冲递增(减)量(加/减速度)
		  DownPulseQ16  - 每个插补周期脉冲递增(减)量(加/减速度)
		  LineDistQ16   -加(减)速段的长度(以脉冲为单位) 
  输  出: 加速段最大插补脉冲数  
  子函数:                                       
  描  述: 根据起点处每周期插补脉冲数StartPulse, 终点处每周期插补脉冲数EndPulse
          以及每个插补周期脉冲递增量, 计算能够加速到的最大速度
		  S1=(Vt^2 - V01^2)/2a1 + (Vt + V01)/2
		  S2=(Vt^2 - V02^2)/2a2 + (Vt + V02)/2
		  S1+S2=S
		  ax^2+bx-c=0
		  a=1
		  b=2*a1*a2/(a1+a2)
		  c=(a1*a2*(S*2-V01-V02)+a2*V01^2+a1*V02^2)/(a1+a2)
		  x=(sqrt(b^2 - 4*a*c)-b)/(2*a)
********************************************************************************/ 
Static_Inline int64 PPMaxPulseRecalQ16(int64 PPStartPulseQ16, int64 PPStopPulseQ16, int64 RisePulseQ16,
                                               int64 DownPulseQ16, int64 LineDistQ16)
{
	Uint64 Temp1 = 0;
	Uint64 Temp2 = 0;
	int64 Temp3 = 0;
	Uint64 Temp4 = 0;
//    Uint64 MaxPulseSquareQ16 = 0;
    Uint64 MaxPulseQ16 = 0;
    int64 Factor_c = 0;
    int64 Factor_b = 0;
    
    if(RisePulseQ16<0)RisePulseQ16 = 0- RisePulseQ16;
    if(DownPulseQ16<0)DownPulseQ16 = 0- DownPulseQ16;

    Temp2 = RisePulseQ16 + DownPulseQ16;//2^15
    
    //计算方案4
    LineDistQ16 = LineDistQ16<<1;
    LineDistQ16 = LineDistQ16 - PPStartPulseQ16 - PPStopPulseQ16;		//by huangxin 如果是负的，后续过程分析？？

    Temp1 = RisePulseQ16 * DownPulseQ16;
    Temp1 = Temp1/Temp2;
    
    Factor_b = Temp1;

    Temp4 = (Uint64)LineDistQ16>>4;
	Temp4 = Temp4 * Temp1;

	if((Temp4 & 0xf000000000000000) == 0 )//右移4位后相乘结果高4bit是0，说明相乘不溢出，可以直接计算	  //by huangxin201804 _5 修改笔误
	{
	 	Temp1 = (Uint64)LineDistQ16*Temp1;	// (2*s - v1 -v2)*a1*a2/(a1+a2)	   //这一步溢出了 !!!!!!  
	    Factor_c = Temp1;	// (2*s - v1 -v2)*a1*a2/(a1+a2) 
    	Temp1 = (Uint64)PPStartPulseQ16 * PPStartPulseQ16 *DownPulseQ16 ; //v1 * v1 *a2
    	Temp1 = Temp1/Temp2;  // v1 * v1 * a2 /(a1 + a2)
   	 	Factor_c = Factor_c + Temp1; //(2*s - v1 -v2)*a1*a2/(a1+a2) + ( v1^2 * a2/(a1 + a2) )
   	 	Temp1 = (Uint64)PPStopPulseQ16 * PPStopPulseQ16 * RisePulseQ16 ;  //v2^2 * a1
   	 	Temp1 = Temp1/Temp2;											  // v2^2 * a1/(a1+a2)
   	 	Factor_c = Factor_c + Temp1;  //(2*s - v1 -v2)*a1*a2/(a1+a2) +  v1^2 * a2/(a1 + a2) +  v2^2 * a1/(a1+a2)	 
   	 	Temp3 = Factor_b * Factor_b + Factor_c;	 
		Temp3 = qsqrt64(Temp3);
	}
	else   //相乘溢出，需要每个变量右移4bit计算
	{
		LineDistQ16= LineDistQ16>>2;
		Temp1 = Temp1 >>2;
		Temp2 = Temp2 >>2;
		PPStartPulseQ16 =PPStartPulseQ16>>2;
		DownPulseQ16 = DownPulseQ16 >>2;
		PPStopPulseQ16 =PPStopPulseQ16 >>2;
		RisePulseQ16 = RisePulseQ16>>2;
		Factor_b = Factor_b >>2;
			
		Temp1 = (Uint64)LineDistQ16*Temp1;	// (2*s - v1 -v2)*a1*a2/(a1+a2)	    
	    Factor_c = Temp1;	// (2*s - v1 -v2)*a1*a2/(a1+a2) 
    	Temp1 = (Uint64)PPStartPulseQ16 * PPStartPulseQ16 *DownPulseQ16 ; //v1 * v1 *a2
    	Temp1 = Temp1/Temp2;  // v1 * v1 * a2 /(a1 + a2)
   	 	Factor_c = Factor_c + Temp1; //(2*s - v1 -v2)*a1*a2/(a1+a2) + ( v1^2 * a2/(a1 + a2) )
   	 	Temp1 = (Uint64)PPStopPulseQ16 * PPStopPulseQ16 * RisePulseQ16 ;  //v2^2 * a1
   	 	Temp1 = Temp1/Temp2;											  // v2^2 * a1/(a1+a2)
   	 	Factor_c = Factor_c + Temp1;  //(2*s - v1 -v2)*a1*a2/(a1+a2) +  v1^2 * a2/(a1 + a2) +  v2^2 * a1/(a1+a2)	 
   	 	Temp3 = Factor_b * Factor_b + Factor_c;	 
		Temp3 = qsqrt64(Temp3);
		Temp3 = Temp3<<2;//开根号后右移2bit放大回来 
	}
    Temp3 = Temp3 - Factor_b;
    MaxPulseQ16 = Temp3; 
    //计算方案4

    
    
    //计算方案3
    /*LineDistQ16 = (Uint64)LineDistQ16<<1;
    Temp1 = RisePulseQ16 * DownPulseQ16;
    Temp1 = Temp1/Temp2;
    Temp1 = (Uint64)LineDistQ16*Temp1;
    Temp1 = qsqrt64(Temp1);
    MaxPulseQ16 =Temp1; 

    Temp1 = (Uint64)DownPulseQ16<<20;
    Temp1 = Temp1/Temp2;
    Temp1 = qsqrt64(Temp1);
    Temp1 = PPStartPulseQ16 * Temp1>>10;
    MaxPulseQ16 +=Temp1; 
    
    
    Temp1 = (Uint64)RisePulseQ16<<20;
    Temp1 = Temp1/Temp2;
    Temp1 = qsqrt64(Temp1);
    Temp1 = PPStopPulseQ16 * Temp1>>10;
    MaxPulseQ16 +=Temp1; */
    //计算方案3

    
    //计算方案2---精度损失最大
    /*Temp2 = qsqrt64(Temp2);
    
    PPStartPulseQ16 = qsqrt64(PPStartPulseQ16);
    PPStopPulseQ16 = qsqrt64(PPStopPulseQ16);
    RisePulseQ16 = qsqrt64(RisePulseQ16);
    DownPulseQ16 = qsqrt64(DownPulseQ16);
    LineDistQ16 =  qsqrt64(LineDistQ16<<1);
    

    Temp1 = (Uint64)LineDistQ16 * RisePulseQ16 * DownPulseQ16;
    MaxPulseQ16 = Temp1/Temp2;

    Temp1 =  (PPStartPulseQ16 * PPStartPulseQ16) * DownPulseQ16;
    MaxPulseQ16 += Temp1/Temp2;
    
    Temp1 = (PPStopPulseQ16 * PPStopPulseQ16) * RisePulseQ16;
    MaxPulseQ16 += Temp1/Temp2;*/
    //计算方案2

    //计算方案1---精度损失最小，但极易溢出
    /*Temp1 = ((Uint64)LineDistQ16>>15) * RisePulseQ16 * DownPulseQ16;
    MaxPulseSquareQ16 = (Temp1/Temp2)<<16;

    Temp1 =  (PPStartPulseQ16 * PPStartPulseQ16) * DownPulseQ16;
    MaxPulseSquareQ16 += Temp1/Temp2;
    
    Temp1 = (PPStopPulseQ16 * PPStopPulseQ16) * RisePulseQ16;
    MaxPulseSquareQ16 += Temp1/Temp2;
    //计算方案1

    
    MaxPulseQ16 = qsqrt64(MaxPulseSquareQ16);*/
    return MaxPulseQ16 ;
}


/*******************************************************************************
  函数名: CanopenLineIntpltInit
  输  入: StartSpeed - 起步速度, 0或者当前电机实际插补脉冲数转化成的速度指令
          RunSpeed - 运行速度, 6081
		  LineDist - 总位移长度，607A
		  参数 UpTime, DownTime 都以插补周期为单位
  输  出:    
  子函数: PPAccDistCalc()计算加(减)速段的长度
  描  述: 定长直线插补初始化 
********************************************************************************/ 
void CanopenLineIntpltInit(int64 StartPulseQ16, int64 RunPulseQ16, int64 StopPulseQ16,
                    int64 RisePulseQ16, int64 DownPulseQ16, int64 LineDistQ16,
					STR_POSCTRLLINEINTERPLT *pAtb)
{
	int32 DecMinDist = 0;        // 计算开始速度大于终止速度时计算最小减速距离
	Uint16 i = 0;                // 循环计数变量
	int64 StartRealQ16 = 0;

	pAtb->PPStatus = 0;  // 插补状态置0, 表示当前并未插补
	
	// 计算匀速段每个插补周期的脉冲个数 Q16	
	pAtb->PPAvergePulseQ16 = RunPulseQ16;
	 
	// 计算起动速度对应的每个插补周期脉冲个数 Q16
    if(pAtb->ZeroPassHandle!=0)//存在过零减速段
    {
        pAtb->PPStartPulseQ16 = STR_PosCtrlVar.LPStartSPD;
        StartRealQ16 = StartPulseQ16;
    }
    else
    {
	    pAtb->PPStartPulseQ16 = StartPulseQ16;
        StartRealQ16 = STR_PosCtrlVar.LPStartSPD;
    }
    
	// 计算停止速度对应的每个插补周期脉冲个数 Q16
	pAtb->PPStopPulseQ16 = StopPulseQ16;

    if(STR_ServoMonitor.StopCtrlVar.CanHaltStopMode == 4)//斜坡停车
    {
        pAtb->HaltDownPulseQ16 = -STR_LmtVar.Pos_ProfDecQ16;
    }
    else if(STR_ServoMonitor.StopCtrlVar.CanHaltStopMode == 5)
    {
        pAtb->HaltDownPulseQ16 = -STR_LmtVar.Pos_QuickStopDecQ16;
    }

	
    //测试部要求无斜坡起动与停机
//    RisePulseQ16 = 0;
//    DownPulseQ16 = 0;
    //测试添加over
						
	if (RisePulseQ16) 
	{
        if (pAtb->PPStartPulseQ16 < pAtb->PPAvergePulseQ16) 
		{   // 目标速度比起步速度大, 递增量为正
			pAtb->PPUpPulseRevQ16 = RisePulseQ16;
			if (pAtb->PPUpPulseRevQ16  == 0)
			{
				pAtb->PPUpPulseRevQ16 = 1;
			}
		}
		else if (pAtb->PPStartPulseQ16 > pAtb->PPAvergePulseQ16)
		{   // 目标速度比起步速度小, 递增量为负数
			pAtb->PPUpPulseRevQ16 = 0 - RisePulseQ16;		   
			if (pAtb->PPUpPulseRevQ16 == 0)
			{
				pAtb->PPUpPulseRevQ16 = -1;
			}
		}
        else
        {
            pAtb->PPUpPulseRevQ16 = 0;
        }
	}
	else
	{
		pAtb->PPUpPulseRevQ16 = 0;
	}

	if (DownPulseQ16)
	{
		if (pAtb->PPAvergePulseQ16 < pAtb->PPStopPulseQ16) 
		{   // 目标速度比起步速度大, 递减量为正
			pAtb->PPDownPulseRevQ16 = DownPulseQ16;
			if (pAtb->PPDownPulseRevQ16 == 0)
			{
				pAtb->PPDownPulseRevQ16 = 1;
			}
		}
		else if (pAtb->PPAvergePulseQ16 > pAtb->PPStopPulseQ16)
		{   // 目标速度比起步速度小, 递减量为负
			pAtb->PPDownPulseRevQ16 = 0 - DownPulseQ16;
			if (pAtb->PPDownPulseRevQ16==0)
			{
				pAtb->PPDownPulseRevQ16 = -1;
			}
		}
        else
        {
            pAtb->PPDownPulseRevQ16 = 0;
        }
	}
	else
	{
		pAtb->PPDownPulseRevQ16 = 0;
	}


    if(pAtb->ZeroPassHandle != 0) 
    {
        //不管是正向减速还是反向减速，起步速度小于目标速度(0),因此递减量始终为负
        pAtb->ZeroPassDownPulseRevQ16 = - DownPulseQ16;
        if (pAtb->ZeroPassDownPulseRevQ16 == 0)
		{
			pAtb->ZeroPassDownPulseRevQ16 = -1;
		}
    	pAtb->ZeroPassLengthQ16 = PPAccDistCalc(StartRealQ16, STR_PosCtrlVar.LPStopSPD, pAtb->ZeroPassDownPulseRevQ16);
        pAtb->ZeroPassOver = 1;//过零未完成
    }
    else
    {
        pAtb->ZeroPassDownPulseRevQ16 = 0;
        pAtb->ZeroPassLengthQ16 = 0;
        pAtb->ZeroPassOver = 0;//过零完成
    }

    //位移为正，正向运行时，需要补上反向减速的距离
    if((LineDistQ16>0)&&(pAtb->ZeroPassHandle==(-1)))LineDistQ16 += pAtb->ZeroPassLengthQ16;
    //位移为负，反向运行时，需要补上正向减速的距离
    else if((LineDistQ16<0)&&(pAtb->ZeroPassHandle==1))LineDistQ16 -= pAtb->ZeroPassLengthQ16;

        
    // 插补长度用绝对值表示这里保存长度值的方向(正负号), 待每个插补周期的脉冲数
	// 计算出来后, 再将方向补上
	if(LineDistQ16 > 0)
	{
		pAtb->PPLineDir = 1;
		pAtb->PPLineLengthQ16 = LineDistQ16;
	}
	else if(LineDistQ16 < 0)
	{
		pAtb->PPLineDir = -1;
		pAtb->PPLineLengthQ16 = 0 - LineDistQ16;
	}
	else
	{
		pAtb->PPLineLengthQ16 = 0;
	}
    
	//计算加速段长度
	pAtb->PPUpLengthQ16 = PPAccDistCalc(pAtb->PPStartPulseQ16, pAtb->PPAvergePulseQ16, pAtb->PPUpPulseRevQ16);

	// 计算减速段长度
	pAtb->PPDownLengthQ16 = PPAccDistCalc(pAtb->PPAvergePulseQ16, pAtb->PPStopPulseQ16, pAtb->PPDownPulseRevQ16);



    if((pAtb->PPUpLengthQ16 + pAtb->PPDownLengthQ16) > pAtb->PPLineLengthQ16)
	{   
	    if(pAtb->PPDownLengthQ16 == 0)
		{
		    pAtb->PPUpLengthQ16 = pAtb->PPLineLengthQ16;
		}
		else
		{
		    // 加速段和减速段长度之和大于总长度, 此时先计算减速段最小长度
			if(pAtb->PPStartPulseQ16 > pAtb->PPStopPulseQ16)
			{
			    DecMinDist = PPAccDistCalc(pAtb->PPStartPulseQ16, pAtb->PPStopPulseQ16, pAtb->PPDownPulseRevQ16);
			}
	        else
			    DecMinDist = 0;
	
			if(pAtb->PPLineLengthQ16 < DecMinDist)
			{   // 如果减速段最小长度仍然大于总长度
			    pAtb->PPUpLengthQ16 = 0;                  // 将加速段置0
			    pAtb->PPAvergeLengthQ16 = 0;              // 将匀速段置0
				pAtb->PPDownLengthQ16 = pAtb->PPLineLengthQ16; // 在整段一直减速
			}
			else // 总长度不小于最小减速段长度
			{   // 重置加速段减速段长度值, 将匀速段置0
            
			    /*pAtb->PPUpLengthQ16 = (pAtb->PPLineLengthQ16 - DecMinDist) >> 1;
			    pAtb->PPDownLengthQ16 = pAtb->PPLineLengthQ16 - pAtb->PPUpLengthQ16;
                pAtb->PPAvergeLengthQ16 = 0;*/
                pAtb->PPAvergePulseQ16 = PPMaxPulseRecalQ16(pAtb->PPStartPulseQ16,pAtb->PPStopPulseQ16,pAtb->PPUpPulseRevQ16,
                                                            pAtb->PPDownPulseRevQ16,pAtb->PPLineLengthQ16);

                //计算加速段长度
            	pAtb->PPUpLengthQ16 = PPAccDistCalc(pAtb->PPStartPulseQ16, pAtb->PPAvergePulseQ16, pAtb->PPUpPulseRevQ16);

            	// 计算减速段长度
			    pAtb->PPDownLengthQ16 = pAtb->PPLineLengthQ16 - pAtb->PPUpLengthQ16;
			    pAtb->PPAvergeLengthQ16 = 0;
                
			}
		}
	}
	else // 总长度不小于加速段和减速段长度之和
	{   // 此时存在匀速段
		pAtb->PPAvergeLengthQ16 = pAtb->PPLineLengthQ16 - pAtb->PPUpLengthQ16 - pAtb->PPDownLengthQ16;
	}
    
	// 与620P插补的区别，设置剩余插补长度为总长度 由4段位移组成
	pAtb->PPLineRemainLengthQ16 = pAtb->PPLineLengthQ16 + pAtb->ZeroPassLengthQ16;

    if(pAtb->ZeroPassHandle!=0)
    {
    	pAtb->PPPlanValQ16 = StartRealQ16;    // 起步时每插补周期脉冲个数,Q6
    }
    else
    {
    	pAtb->PPPlanValQ16 = pAtb->PPStartPulseQ16;    // 起步时每插补周期脉冲个数,Q6
    }
    
	pAtb->PPPlanValueRemainQ16 = 0;                 // 当前插补周期脉冲剩余个数
	pAtb->PPRealVal = pAtb->PPPlanValQ16>>PPAMPBIT; // 本插补周期脉冲个数

	if(pAtb->PPLineRemainLengthQ16 > 0)
	{
	    pAtb->PPStatus = 1;   // 置插补状态值为1, 起动插补
	    pAtb->PPPlanDecAgain = 1;
	}
    
    
    // 目前有三处地方调用了插补功能, 当某处抢占使用时, 其余的插补结构需要复位
	for (i = 0; i < 3; i++)
	{
	    if (0 != pCanopenIntplt[i])
		{
	        if (pAtb != pCanopenIntplt[i])
			{
				pCanopenIntplt[i]->PPStatus = 0;
				pCanopenIntplt[i]->PPLineRemainLengthQ16 = 0;
			}
		}
	}

}


/*******************************************************************************
  函数名: int32 CanopenLineIntpItCal(STR_POSCTRLLINEINTERPLT *pAtb) 
  输  入: pAtb-指向直线插补寄存器的指针          
  输  出: 当前插补周期的插补脉冲数   
  子函数:                                       
  描  述: 计算每个插补周期的插补脉冲值  
********************************************************************************/ 
int32 CanopenLineIntpItCal(STR_POSCTRLLINEINTERPLT *pAtb)
{
	int64 tmpCnt = 0, sum = 0, sm1 = 0, df1 = 0, twice = 0, appPls = 0;

    static Uint8 FirFlag = 1;
	static Uint16 OEMStatusLast = 0;	//by huangxin201711_2 从暂停取消后，立即零位固定，与小多传一致

    if ((1 == pAtb->PPStatus) && (pAtb->PPLineRemainLengthQ16 > 0)&&
        (DeviceControlVar.OEMStatus != STATUS_HALT)&& OEMStatusLast != STATUS_HALT)
	{   // 只有在插补状态标志等于1 并且 插补剩余长度大于0时才进行插补
		//加速,当前剩余长度大于恒速长度＋减速长度
        if(pAtb->PPLineRemainLengthQ16 > (pAtb->PPUpLengthQ16 + pAtb->PPAvergeLengthQ16 + pAtb->PPDownLengthQ16))//过零段
        {
            if(FirFlag == 1)//都则将差一次脉冲，导致减速拖尾
            {
                FirFlag = 0;
            }
            else
            {
                pAtb->PPPlanValQ16 +=  pAtb->ZeroPassDownPulseRevQ16; 
            }
			if(pAtb->ZeroPassDownPulseRevQ16 > 0)//递增量为负
			{
                if(pAtb->PPPlanValQ16 > STR_PosCtrlVar.LPStopSPD)//过零前最后一次插补
				{   // 加速段每插补周期的最大脉冲数不大于恒速运行时的脉冲数
					pAtb->PPPlanValQ16 = STR_PosCtrlVar.LPStopSPD;
                    pAtb->ZeroPassOver = 0;//过零完成
				} 
			}
            else
            {
                if(pAtb->PPPlanValQ16 < STR_PosCtrlVar.LPStopSPD)//过零前最后一次插补
				{   // 加速段每插补周期的最大脉冲数不大于恒速运行时的脉冲数
					pAtb->PPPlanValQ16 = STR_PosCtrlVar.LPStopSPD;
                    pAtb->ZeroPassOver = 0;//过零完成
				} 
            }
        }
        else if (pAtb->PPLineRemainLengthQ16 >= (pAtb->PPAvergeLengthQ16 + pAtb->PPDownLengthQ16))   
		{
            pAtb->ZeroPassOver = 0;//过零完成
            pAtb->ZeroPassHandle = 0;
            pAtb->PPPlanValQ16 +=  pAtb->PPUpPulseRevQ16; 
			if(pAtb->PPUpPulseRevQ16 > 0)
			{
				if(pAtb->PPPlanValQ16 > pAtb->PPAvergePulseQ16)
				{   // 加速段每插补周期的最大脉冲数不大于恒速运行时的脉冲数
					pAtb->PPPlanValQ16 = pAtb->PPAvergePulseQ16;
				} 
			}
			else
			{
				if(pAtb->PPPlanValQ16 < pAtb->PPAvergePulseQ16)
				{   // 加速段每插补周期的最大脉冲数不大于恒速运行时的脉冲数
					pAtb->PPPlanValQ16 = pAtb->PPAvergePulseQ16;
				} 
			}
		}
		else if(pAtb->PPLineRemainLengthQ16 <= pAtb->PPDownLengthQ16)
		{   
            pAtb->ZeroPassOver = 0;//过零完成
            pAtb->ZeroPassHandle = 0;
			if (1 == pAtb->PPPlanDecAgain)
			{   // 根据剩余的脉冲数重新规划减速段
				pAtb->PPDownLengthQ16 = pAtb->PPLineRemainLengthQ16;				
				sm1 = pAtb->PPStopPulseQ16 + pAtb->PPPlanValQ16;
				df1 = pAtb->PPStopPulseQ16 - pAtb->PPPlanValQ16;
				twice = pAtb->PPDownLengthQ16 << 1;
				
				// 差值及每次的补偿值存储变量清零
				PPdiff = 0;
		        PPdiff2 = 0;
		        PPcompsnt = 0;
				
                // 计算减速时间(插补次数)
				tmpCnt =  twice / sm1;
				
				// 减速段插补次数大于4 才处理, tmpCntd等于1时会导致指令总数不对
				if (tmpCnt > 4)
				{
					// 计算减速时每插补周期脉冲递减量(减速度)
					// 以防止出现除数为0的情况
					twice += (twice == sm1)? 1 : 0;
					pAtb->PPDownPulseRevQ16 = (((sm1>>3) * (df1>>3)) / (twice - sm1))<<6;
					
					// 根据规划的插补次数和每插补周期脉冲递减量及起停速度计算总脉冲数
					sum = tmpCnt * (pAtb->PPPlanValQ16 + (pAtb->PPDownPulseRevQ16
															* (tmpCnt - 1) >> 1));	
			        // 计算实际总脉冲数与规划所得总脉冲数的差值
					PPdiff = pAtb->PPDownLengthQ16 - sum;
					
					// 差值大于零说明可能会拖尾, 小于0则可能会跌落
					if (PPdiff > 0)
				    {
						PPdiff2 = PPdiff;
					}
					else
					{
						PPdiff2 = (0-PPdiff);
					}
					
					// 将差值分 tmpCnt/2 次补偿到减速过程, 以使规划的递减过程得以实现
					// 以防止出现除数为0的情况
					tmpCnt += (0 == tmpCnt)? 1 : 0;
					PPcompsnt = (PPdiff2<<1) / tmpCnt;
				}
							
				pAtb->PPPlanDecAgain = 0;
			}
			else
			{
			    pAtb->PPPlanValQ16 += pAtb->PPDownPulseRevQ16; 
			}
			
			// 补偿差值, 若拖尾则将多余脉冲在减速的前半段发出去
			// 若跌落, 则将使其不跌落时所差的脉冲数在减速的前半段中少发一点补回来
			appPls = 0;
			if (PPdiff2 > 0)
			{
				if (PPdiff2 > PPcompsnt)
				{
					appPls = PPcompsnt;
				}
				else
				{
					appPls = PPdiff2;
				}
				PPdiff2 -= appPls;
				appPls = (PPdiff > 0)? appPls : (0-appPls);
			}
			 
			if(pAtb->PPDownPulseRevQ16 > 0 )
			{
			    if(pAtb->PPPlanValQ16 > pAtb->PPStopPulseQ16)
			    {   // 减速段每插补周期的最小脉冲数不小于预期的停止脉冲数
			        pAtb->PPPlanValQ16 = pAtb->PPStopPulseQ16;
			    }
			}
			else
			{
			    if(pAtb->PPPlanValQ16 < pAtb->PPStopPulseQ16)
		        {   // 减速段每插补周期的最小脉冲数不小于预期的停止脉冲数
			        pAtb->PPPlanValQ16 = pAtb->PPStopPulseQ16;
			    }
			}
		}
		else //恒速
		{
            pAtb->ZeroPassOver = 0;//过零完成
            pAtb->ZeroPassHandle = 0;
            pAtb->PPPlanValQ16 = pAtb->PPAvergePulseQ16;
		}
		
		//插补结束判断
		if(pAtb->PPLineRemainLengthQ16 <= pAtb->PPPlanValQ16)
		{
			pAtb->PPPlanValQ16 = pAtb->PPLineRemainLengthQ16;
			pAtb->PPStatus = 0;//插补结束
            pAtb->ZeroPassOver = 0;//过零完成
		}
		
		// 计算实际插补脉冲数	
		pAtb->PPRealVal = (pAtb->PPPlanValQ16 + appPls + pAtb->PPPlanValueRemainQ16)>>PPAMPBIT;	
		// 计算右移LINEAMPBIT位产生的余数
		pAtb->PPPlanValueRemainQ16 = pAtb->PPPlanValQ16  + appPls + pAtb->PPPlanValueRemainQ16
		                           - (pAtb->PPRealVal<<PPAMPBIT);
		// 计算剩余脉冲数
		pAtb->PPLineRemainLengthQ16 -= (pAtb->PPPlanValQ16+ appPls);
        //计算过零剩余脉冲数
        if(pAtb->ZeroPassHandle!=0)pAtb->ZeroPassRealLengthQ16 += pAtb->PPPlanValQ16;
	}
    else if((1 == pAtb->PPStatus)&&(pAtb->PPLineRemainLengthQ16 > 0)
            &&(DeviceControlVar.OEMStatus == STATUS_HALT))
    {
        pAtb->PPPlanValQ16 += pAtb->HaltDownPulseQ16;//减速

        if(pAtb->PPPlanValQ16 <= STR_PosCtrlVar.LPStopSPD)//暂停最后一次插补
        {
            pAtb->PPStatus = 0;
            if(pAtb->PPLineRemainLengthQ16 < STR_PosCtrlVar.LPStopSPD)
            {
                pAtb->PPPlanValQ16 = pAtb->PPLineRemainLengthQ16;
    			STR_PosHaltVar.HaltPosRefOk = 1;//暂停结束时位置指令刚好执行完毕
            }
            else
            {
                pAtb->PPPlanValQ16 = STR_PosCtrlVar.LPStopSPD;
    			STR_PosHaltVar.HaltPosRefOk = 0;//暂停结束时位置指令未执行完毕
    			//锁存剩余位置变量，为恢复运行初始化用
    			STR_PosHaltVar.PPAvergePulseQ16 = pAtb->PPAvergePulseQ16;
    			STR_PosHaltVar.PPUpPulseRevQ16 =  ABS(pAtb->PPUpPulseRevQ16);
    			STR_PosHaltVar.PPDownPulseRevQ16 = ABS(pAtb->PPDownPulseRevQ16);
            }
        }
        else//速度未到0,但指令已结束
        {
            if(pAtb->PPLineRemainLengthQ16 <= pAtb->PPPlanValQ16)
            {
                pAtb->PPStatus = 0;
                pAtb->PPPlanValQ16 = pAtb->PPLineRemainLengthQ16;
    			STR_PosHaltVar.HaltPosRefOk = 1;//暂停结束时位置指令刚好执行完毕
            }
            else
            {
    			STR_PosHaltVar.HaltPosRefOk = 2;//暂停进行中
            }

        }
		
		// 计算实际插补脉冲数	
		pAtb->PPRealVal = (pAtb->PPPlanValQ16 + pAtb->PPPlanValueRemainQ16)>>PPAMPBIT;	
		// 计算右移LINEAMPBIT位产生的余数
		pAtb->PPPlanValueRemainQ16 = pAtb->PPPlanValQ16  + pAtb->PPPlanValueRemainQ16
		                           - (pAtb->PPRealVal<<PPAMPBIT);
		// 计算剩余脉冲数
		pAtb->PPLineRemainLengthQ16 -= pAtb->PPPlanValQ16;
        pAtb->HaltDownLengthQ16 += pAtb->PPPlanValQ16;
        
        if(STR_PosHaltVar.HaltPosRefOk == 0)
        {
            //如果暂停刚好发生在过零减速段，暂停完成，也即是完成过零
            //暂停加速度与减速加速度不一致时，会导致暂停位移与过零位移不一致，暂停加速度大时，剩余总位移应减去暂停停少走的位移
            //暂停加速度小时，剩余总位移应加上暂停多走的位移
            if(pAtb->ZeroPassHandle!=0)
            {
                pAtb->ZeroPassRealLengthQ16 += pAtb->HaltDownLengthQ16;
                pAtb->PPLineRemainLengthQ16 = pAtb->PPLineLengthQ16 - pAtb->ZeroPassLengthQ16 + pAtb->ZeroPassRealLengthQ16 ;
                pAtb->ZeroPassOver = 0;//过零完成
                pAtb->ZeroPassRealLengthQ16 = 0;
                pAtb->HaltDownLengthQ16 = 0;
            }
            else{}
            
            
            STR_PosHaltVar.PPLineLengthQ16 = pAtb->PPLineRemainLengthQ16;
        	if(pAtb->PPLineDir != 1)
            {
                STR_PosHaltVar.PPLineLengthQ16 = 0 - STR_PosHaltVar.PPLineLengthQ16;
            }   
        }

    }
	else if ((DeviceControlVar.OEMStatus != STATUS_HALT) && (OEMStatusLast == STATUS_HALT) )  //by huangxin201711_2 从暂停取消后，立即零位固定，与小多传一致
	{
		CanopenPPReset();			//by huangxin201711_2 从暂停取消后，立即零位固定，与小多传一致
	}
    else
    {
        pAtb->PPStatus  = 0;//插补结束
	    pAtb->PPRealVal = 0;
        pAtb->ZeroPassOver = 0;//过零完成
        FirFlag = 1;
        return 0;	
	}

	OEMStatusLast =DeviceControlVar.OEMStatus;//by huangxin201711_2 从暂停取消后，立即零位固定，与小多传一致
    
	// 根据记录的方向标志, 恢复插补脉冲的方向
    if(pAtb->ZeroPassHandle == 0)
    {
    	if(pAtb->PPLineDir != 1)
        {
            pAtb->PPRealVal=0 - pAtb->PPRealVal;
        }   
    	else
        {}
    }
    else if(pAtb->ZeroPassHandle==(-1))//反向过零
    {
        pAtb->PPRealVal=0 - pAtb->PPRealVal;
    }
    else
    {}
    
    if(pAtb->ZeroPassOver==0)pAtb->ZeroPassHandle = 0;//过零处理结束


	
    return  pAtb->PPRealVal;
    
}

/*******************************************************************************
  函数名:  CanopenLineIntpltReset
  输  入:  STR_POSCTRLLINEINTERPLT 
  输  出:    
  子函数:                                       
  描  述: 插补状态复位 
********************************************************************************/
void CanopenLineIntpltReset(STR_POSCTRLLINEINTERPLT *pAtb)
{
	pAtb->PPStatus = 0;
	pAtb->PPLineRemainLengthQ16 = 0;
    pAtb->PPRealVal = 0;
    pAtb->ZeroPassHandle = 0;
    pAtb->ZeroPassOver = 0;
}
/*******************************************************************************
  函数名: void RecordCanopenIntpltStruct(STR_POSCTRLLINEINTERPLT *pAtb, Uint16 index)
  输  入: pAtb - 指向插补结构的指针
          index - 存储下标
          pCanopenIntplt[0]指向PP
          pCanopenIntplt[1]指向HM
          pCanopenIntplt[2]指向IP
  输  出:   
  子函数:                                       
  描  述: 记录插补结构变量的地址
********************************************************************************/ 				
void RecordCanopenIntpltStruct(STR_POSCTRLLINEINTERPLT *pAtb, Uint16 index)
{
    Uint16 u16Size = 0;
    u16Size = sizeof(pCanopenIntplt) / sizeof(pCanopenIntplt[0]);
    if (index > (u16Size - 1))
	{
	    index = u16Size - 1;
	}
	
    if (0 == pCanopenIntplt[index])
	{
	    pCanopenIntplt[index] = pAtb;
	}
}
/*******************************************************************************
  函数名: void ResetCanopenIntpltDownLength()
  输  入: PPLineLengthQ16 - 总脉冲数 
          PPLineRemainLengthQ16  - 剩余脉冲数
  输  出: 重置减速段的长度(脉冲数)  
  子函数:                                       
  描  述: 提前结束插补过程时重置减速段长度
********************************************************************************/ 
void ResetCanopenIntpltDownLength(STR_POSCTRLLINEINTERPLT *pAtb)
{
    if(pAtb->PPLineRemainLengthQ16 > (pAtb->PPUpLengthQ16 + pAtb->PPAvergeLengthQ16 + pAtb->PPDownLengthQ16))
    {
        //当前正处于过零段，以过零段剩余的位移作为减速段长度
        pAtb->PPDownLengthQ16 = pAtb->ZeroPassLengthQ16 - pAtb->ZeroPassRealLengthQ16;
        pAtb->PPDownPulseRevQ16 = pAtb->ZeroPassDownPulseRevQ16;
        pAtb->ZeroPassOver = 1;
        pAtb->ZeroPassHandle = Sign_NP(pAtb->PPRealVal);
    }
    else if (pAtb->PPLineRemainLengthQ16 > (pAtb->PPAvergeLengthQ16 + pAtb->PPDownLengthQ16))
	{   // 当前正处于加速段, 以已经走过的长度作为减速段长度
	    pAtb->PPDownLengthQ16 = pAtb->PPLineLengthQ16 - pAtb->PPLineRemainLengthQ16;
	}
	else if(pAtb->PPLineRemainLengthQ16 <= pAtb->PPDownLengthQ16)
	{   // 当前正处于减速段, 以剩余的作为减速段长度
	    pAtb->PPDownLengthQ16 = pAtb->PPLineRemainLengthQ16;
	}
	else
	{   // 当前正处于匀速段, 仍为预先规划的减速段长度
	    // pAtb->DownLengthQ16;
	}
}

/*******************************************************************************
  函数名: void CanopenPosArrive(int32 PosAmplifErrTemp)
  输入:    PosAmplifErrTemp //位置偏差
  输出:
  子函数:  无 
  描述:    根据位置到达输出条件，置6041 bit12
********************************************************************************/ 
void CanopenPosArrive(int32 PosAmplifErrTemp)//要改成用户单位位置偏差判断
{
    static Uint32  TimeCnt = 0;
	int8  CoinTemp = 0;
    Uint64 PosWin = 0;
	int32 AbsPosErr =0;

    PosWin = (Uint64)STR_PosCtrlVar.PositionWindow;

    if(STR_PosCtrlVar.PosWinUnitSet==0)//0-编码器单位 1-指令单位
    {
        PosWin = PosWin * (Uint64)STR_Pos_Factor.PosFactor6093_Denominator
                 /(Uint64)STR_Pos_Factor.PosFactor6093_Numerator;
    }
    
//    if(FunCodeUnion.code.FC_FeedbackMode == 0)
//    {
//	    STR_PosCtrlVar.CurrentAbsPosUserUnit = IncpUnit2UserPosUnit(&STR_Pos_Factor_Inverse,STR_InnerGvarPosCtrl.CurrentAbsPos);
//    }
//    else if(FunCodeUnion.code.FC_FeedbackMode == 1)//外部位置反馈
//    {
//	    STR_PosCtrlVar.CurrentAbsPosUserUnit = IncpUnit2UserPosUnit(&STR_Pos_Factor_Inverse,STR_FUNC_Gvar.PosCtrl.ExCurrentAbsPos);
//    }
//    else
//    {
//        PostErrMsg(MULTPOSCLASHFULCLOP);    //参数设置错误
//    }

    AbsPosErr = IncpUnit2UserPosUnit(&STR_Pos_Factor_Inverse,PosAmplifErrTemp);
    
    CoinTemp = (ABS(AbsPosErr) <= PosWin) ? 1 : 0; 

		
	if((CoinTemp==1)&&(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN))//编码器单位
    {
        if(TimeCnt >= STR_PosCtrlVar.PositionWindowTime)
        {
            TimeCnt = STR_PosCtrlVar.PositionWindowTime;
			STR_FUNC_Gvar.PosCtrl.DovarReg_Coin = 1;
        	STR_FUNC_Gvar.PosCtrl.DovarReg_Near = 1;
        }
        else
        {
            TimeCnt ++;
			STR_FUNC_Gvar.PosCtrl.DovarReg_Coin = 0;
        	STR_FUNC_Gvar.PosCtrl.DovarReg_Near = 0;
        }
    }
    else
    {
        TimeCnt = 0;
		STR_FUNC_Gvar.PosCtrl.DovarReg_Coin = 0;
    	STR_FUNC_Gvar.PosCtrl.DovarReg_Near = 0; //DO输出无效
    }

    if(STR_CanSyscontrol.Mode == ECTPOSMOD)
    {
        if(STR_FUNC_Gvar.PosCtrl.DovarReg_Cmdok==1)
        {
            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = STR_FUNC_Gvar.PosCtrl.DovarReg_Coin;
        }
        else
        {
            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = 0;
        }
    }
    else
    {
        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = STR_FUNC_Gvar.PosCtrl.DovarReg_Coin;
    }
}
/*******************************************************************************
  函数名:  PosMonitor()
  输入:    STR_PosCtrl.PosErrCnt                        
  输出:    PERRWARN; PSTNOERR  (警告或故障)
  子函数:  PostErrMsg() (警告或故障处理函数)      
  描述:    位置偏差监控函数
********************************************************************************/ 
void CanopenPosMonitor(int32 PosAmplifErrTemp)
{
    
    int32  PosAmplifErrMax;
	int32  PosAmplifErr_Encoder;
	
	PosAmplifErr_Encoder = PosAmplifErrTemp;
	
	PosAmplifErrTemp = (int32)IncpUnit2UserPosUnit(&STR_Pos_Factor_Inverse,(int64)PosAmplifErrTemp);
	PosAmplifErrMax  = ((int64)1<<31)- UNI_FUNC_MTRToFUNC_InitList.List.EncRev;
	
	
    if(ABS(PosAmplifErr_Encoder)>PosAmplifErrMax)  
    {
        //following error
        PostErrMsg(PSTNOERR);    //偏差位置过大故障
    }	
	
	
	if(STR_PosCtrlVar.FollowingErrorWindow != 0xFFFFFFFF)//指令单位
    {
        if(ABS(PosAmplifErrTemp) >= STR_PosCtrlVar.FollowingErrorWindow)  
        {
            STR_PosCtrlVar.FollowingErrorCnt++;
			if(STR_PosCtrlVar.FollowingErrorCnt>=STR_PosCtrlVar.FollowingErrorTimeOut)
			{
			    STR_PosCtrlVar.FollowingErrorCnt = STR_PosCtrlVar.FollowingErrorTimeOut;
				PostErrMsg(PSTNOERR);    //偏差位置过大故障
			}
        }
        else
        {
			STR_PosCtrlVar.FollowingErrorCnt = 0;
        }
		
        
        if(   (STR_CanSyscontrol.Mode == ECTPOSMOD)   //位置模式下才进行位置控制相关处理244/72us=3.38us
	        ||(STR_CanSyscontrol.Mode == ECTHOMMOD)    //回零模式
	        ||(STR_CanSyscontrol.Mode == ECTCSPMOD))   //周期同步位置模式
		{
			if((STR_FUNC_Gvar.Monitor.HighLevelErrCode & 0x0FFF)==0x0B00)//位置类模式下，位置偏差过大故障，将bit13置为1
	        {
	            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = 1;
	        }
			else
			{
	            if(STR_CanSyscontrol.Mode != ECTHOMMOD)
				{
					ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = 0;
				}
			}
        }
        
    }
    else//不进行位置偏差监控
    {
        if(STR_CanSyscontrol.Mode != ECTHOMMOD)
		{
			ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = 0;
		}
		STR_PosCtrlVar.FollowingErrorCnt = 0;
    }

	STR_FUNC_Gvar.OscTarget.PosAmpErr = PosAmplifErrTemp; 
}

