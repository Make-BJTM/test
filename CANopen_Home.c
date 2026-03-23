
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "CANopen_Home.h"
#include "FUNC_FunCode.h"
#include "FUNC_GlobalVariable.h"
#include "FUNC_InterfaceProcess.h"
#include "FUNC_PosCtrl.h"
#include "FUNC_ErrorCode.h"
#include "CANopen_Pub.h"
#include "CANopen_PP.h"
#include "CANopen_DeviceCtrl.h"
#include "FUNC_FullCloseLoop.h"
#include "FUNC_OperEeprom.h"
#include "stm32f4xx.h"
#include "ECT_CSP.h"
/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/
#define  ENABLE  1
#define  DISABLE 0
/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
/*static*/ STR_HOMINGVARIBLES STR_CanopenHome;//= CanopenHomeDflts;
//static STR_POSCTRLLINEINTERPLT STR_CanopenHomeLnIntplt = CanopenHomeDflts;
static Uint8 KeyLevelLastbit=0;
static Uint8 LimtKeylastbit =0;

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */


/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */
Static_Inline void CanopenHomingOverTimeCheck(void);
Static_Inline void HomingStartSpd(void);
Static_Inline void CanopenHomeLimtSwtich(void);

Static_Inline int32  HomingSpeedHandle(void);
Static_Inline void  HomingSpeedInit(Uint8 Enable,int8 SpeedLeve,Uint8 Dir);
void CanopenHomingModeHalt(void);



/*******************************************************************************
  函数名: void CanopenHomingVarClear(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   标准回零模式下,初始化与停机变量清零
********************************************************************************/ 
void CanopenHomingInit(void)
{
    //清除回零相关变量
	STR_CanopenHome.HomingStep = HIGHSPPENULL;
    STR_CanopenHome.MoveStatus = START0;
    STR_CanopenHome.HomingStatus.bit.HomingAttain = 0;
    STR_CanopenHome.HomingStatus.bit.HomingError  = 0;
	STR_CanopenHome.HomingStatus.bit.ZeroSpeed    = 0;
	STR_CanopenHome.HomingSpeedOutputQ16 = 0;


    //6099-2h //高速转换成每个位置控制周期的位置指令
	STR_CanopenHome.PulseDuringSearchForSwitchQ16 = CanopenPosVelLmt(ObjectDictionaryStandard.HomingMode.HomingSpeeds.SpeedDuringSearchForSwitch,ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity);

    //6099-1h	//低速转换成每个位置控制周期的位置指令
	STR_CanopenHome.PulseDuringSearchForZeroQ16 = CanopenPosVelLmt(ObjectDictionaryStandard.HomingMode.HomingSpeeds.SpeedDuringSearchForZero,ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity);


    //加速度限制--驱动器每个位置周期的位置指令增量
    CanopenPosAccLmt(ObjectDictionaryStandard.HomingMode.HomingAcceleration ,ObjectDictionaryStandard.HomingMode.HomingAcceleration ,ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration);
    STR_CanopenHome.RiseDownPulseQ16 = STR_LmtVar.Pos_ProfAccQ16;
    STR_CanopenHome.HomingAccQ16         = STR_CanopenHome.RiseDownPulseQ16;
	HomingSpeedInit(DISABLE,HIGH,STR_CanopenHome.HomingStatus.bit.HMoveDir);
    
    KeyLevelLastbit = 0;
    LimtKeylastbit  = 0;
	STR_CanopenHome.HomingStatus.bit.ZIndex = 0;	
    STR_CanopenHome.TimeCount = 0;
	STR_FUNC_Gvar.PosCtrl.PosReguOut = 0;   // 位置调节器输出的速度指令
	STR_FUNC_Gvar.PosCtrl.PosAmplifErr = 0; // 随动偏差
	STR_InnerGvarPosCtrl.PosErrLast = 0;    // 前馈速度值及AO输出		
	STR_InnerGvarPosCtrl.FdFwdOut = 0;      // 前馈速度值及AO输出
	STR_FUNC_Gvar.PosCtrl.DovarReg_Coin = 0;    // 位置到达信号清零
	STR_FUNC_Gvar.PosCtrl.DovarReg_Near = 0;    // 位置接近清零
	STR_CanopenHome.ReverencetoZero=INVALID;
	

	STR_CanopenHome.HomeOffsetInc =(int64)STR_Pos_Factor.PosFactor6093_Numerator * ((int64)(int32)ObjectDictionaryStandard.ProPosMode.HomeOffset)
                                    /(int64)STR_Pos_Factor.PosFactor6093_Denominator;

	if(FunCodeUnion.code.FC_FeedbackMode == 1)
	{
		STR_CanopenHome.HomeOffsetInc = ((int64)STR_CanopenHome.HomeOffsetInc 
										* (int64)STR_FUNC_Gvar.PosCtrl.ExPosCoefQ7)>>7;	
	}
   

	STR_CanopenHome.HomingStatus.bit.HomingClrFlag=0; 
    //扩展，加入回原点时间设置，以报出601错误，后续要测试标杆是如何报出这个错误的
    STR_CanopenHome.SearchTime =(Uint32)FunCodeUnion.code.PL_OriginSearchTime * 10;  //原点查找时间 H0535
    
    // 清除回零超时警告
    WarnAutoClr(ORIGINOVERTIME);
}
/*******************************************************************************
  函数名: void CanopenHomingModeFuncVarStop(void)

  输入:   无 
  输出:   无
      
  描述:   标准回零模式下,初始化与停机更新
********************************************************************************/ 
Static_Inline void CanopenHomeLimtSwtich(void)
{
	
	if(   (ObjectDictionaryStandard.HomingMode.HomingMethod == 15)
        ||(ObjectDictionaryStandard.HomingMode.HomingMethod == 16)
        ||(ObjectDictionaryStandard.HomingMode.HomingMethod == 31)
        ||(ObjectDictionaryStandard.HomingMode.HomingMethod == 32))
    {
        PostErrMsg(ODVALUEERR);
	    STR_FUNC_Gvar.PosCtrl.HomStats = 0; 
		STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
        ObjectDictionaryStandard.HomingMode.HomingMethod = STR_CanopenHome.HomingMethod;
    }
    else
    {
        if(ObjectDictionaryStandard.HomingMode.HomingMethod < 17)
        {
            STR_CanopenHome.FindIndexFlag = VALID;
            STR_CanopenHome.HomingMethod = (int8)ObjectDictionaryStandard.HomingMode.HomingMethod; 
        }
        else if(ObjectDictionaryStandard.HomingMode.HomingMethod < 33)
        {
            STR_CanopenHome.FindIndexFlag = INVALID;
            STR_CanopenHome.HomingMethod = (int8)ObjectDictionaryStandard.HomingMode.HomingMethod - 16; 
        }
        else if(ObjectDictionaryStandard.HomingMode.HomingMethod < 35)
        {
            STR_CanopenHome.FindIndexFlag = VALID;
            STR_CanopenHome.HomingMethod = (int8)ObjectDictionaryStandard.HomingMode.HomingMethod;  
        }
        else
        {
            STR_CanopenHome.FindIndexFlag = INVALID;
            STR_CanopenHome.HomingMethod = (int8)ObjectDictionaryStandard.HomingMode.HomingMethod; 
        }

        if(STR_CanSyscontrol.Mode==CANOPENHOMMOD)
        {
			if((STR_CanopenHome.HomingMethod == 1)||
               (STR_CanopenHome.HomingMethod == 11)||
               (STR_CanopenHome.HomingMethod == 12)||
               (STR_CanopenHome.HomingMethod == 13)||
               (STR_CanopenHome.HomingMethod == 14))
            {
                STR_FUNC_Gvar.PosCtrl.HomStats = 2;//反向回零
            }
            else if((STR_CanopenHome.HomingMethod == 2)||
                   (STR_CanopenHome.HomingMethod == 7)||
                   (STR_CanopenHome.HomingMethod == 8)||
                   (STR_CanopenHome.HomingMethod == 9)||
                   (STR_CanopenHome.HomingMethod == 10))
            {
                STR_FUNC_Gvar.PosCtrl.HomStats = 1;//正向回零
            }
        }
    	else
    	{
    	    STR_FUNC_Gvar.PosCtrl.HomStats = 0; 
    		STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
    	}
    }
	
}
/*******************************************************************************
  函数名: void CanopenHomeRunUpdata(void)

  输入:   无 
  输出:   无
  子函数: 无
  描述:   处理限位开关，运行时调用
********************************************************************************/ 
void CanopenHomeRunUpdata(void)
{
	if(STR_CanopenHome.HomingStatus.bit.HomingEn==1)return;

	CanopenHomeLimtSwtich();
}

/*******************************************************************************
  函数名: void CanopenHomingModeFuncVarStop(void)
  输入:   无 
  输出:   无
  子函数:
      
  描述:   标准回零模式下,初始化与停机更新
********************************************************************************/ 
void CanopenHomingModeFuncVarStop(void)
{

	STR_FUNC_Gvar.PosCtrl.HomStats = 0; 
    STR_CanopenHome.HomingStatus.bit.HomingEn    = 0;
	STR_CanopenHome.HomingStatus.bit.HomingError = 0;
	STR_CanopenHome.HomingStatus.bit.ZeroSpeed   = 0;
    
    STR_InnerGvarPosCtrl.MutexBit.bit.HomeWork   = 0; //  当前不处于回零过程
    STR_InnerGvarPosCtrl.MutexBit.bit.CanopenHomeWork = 0;
    STR_CanopenHome.TimeCount                    = 0;

    CanopenPosAccLmt(ObjectDictionaryStandard.HomingMode.HomingAcceleration ,ObjectDictionaryStandard.HomingMode.HomingAcceleration ,ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration);
    STR_CanopenHome.RiseDownPulseQ16 = STR_LmtVar.Pos_ProfAccQ16;
    STR_CanopenHome.HomingAccQ16     = STR_CanopenHome.RiseDownPulseQ16;    
    CanopenHomeLimtSwtich();
    
    // 清除回零超时警告
    WarnAutoClr(ORIGINOVERTIME);
	
	HomingSpeedInit(DISABLE,HIGH,STR_CanopenHome.HomingStatus.bit.HMoveDir);

}

/*******************************************************************************
  函数名: void CanopenHomingModeFuncVarStop(void)
  输入:   无 
  输出:   无
  子函数: 
      
  描述:   标准回零模式下,初始化与停机更新
********************************************************************************/ 
void CanopenHomingModeHalt(void)
{

	STR_FUNC_Gvar.PosCtrl.HomStats = 0; //暂停时超程开关必须起作用
	STR_CanopenHome.HomingStatus.bit.HomingError = 0;
    STR_InnerGvarPosCtrl.MutexBit.bit.HomeWork   = 0; //  当前不处于回零过程
    STR_InnerGvarPosCtrl.MutexBit.bit.CanopenHomeWork = 0;
    STR_CanopenHome.TimeCount = 0;
    STR_CanopenHome.MoveStatus = START0;
	STR_CanopenHome.HomingStatus.bit.ZIndex = 0;
	
    		  
    // 清除回零超时警告
    WarnAutoClr(ORIGINOVERTIME);

}

/*******************************************************************************
  函数名: void CanopenZeroIndexISR(void) 
  输  入:           
  输  出:   
  子函数:                                       
  描  述: z信号中断服务程序
********************************************************************************/
void CanopenZeroIndexISR(void)
{
    if((STR_CanopenHome.MoveStatus == FINDINDESTEP2)&&(STR_CanopenHome.FindIndexFlag == VALID))
    {
        STR_CanopenHome.HomingStatus.bit.ZIndex=1;
		STR_CanopenHome.HomingStatus.bit.HomingClrFlag=1;
    }
}

/*******************************************************************************
  函数名: void CanopenHomingModeFunc(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 回零主函数
********************************************************************************/
void CanopenHomingModeFunc(void)
{
    //6040 bit4 0-->1 :启动回零  1:回零中 0:回零无效
    int8 CtrwdBit4=0; 
    static Uint8 OEMStatus = 0;
    if(STR_CanSyscontrol.Mode==CANOPENHOMMOD)
    {
        CtrwdBit4 = ControlWord.bit.OperationModeSpecific & 0x1;  //0x6040bit4
        //第一次触发回零，必须是在运行状态下，bit4给出上升沿
        if((DeviceControlVar.OEMStatus == STATUS_OPERENABLE)&&
            ((OEMStatus == STATUS_OPERENABLE)||(OEMStatus == STATUS_HALT)||(OEMStatus == STATUS_QUICKSTOPACTIVE)))
        {
            if((CtrwdBit4 ==1)&&(STR_CanopenHome.HomingStatus.bit.OpModeSpebit==0))//bit4上升沿
            {
                STR_CanopenHome.HomingStatus.bit.HomingEn = 1;
                ObjectDictionaryStandard.DeviceControl.StatusWord.bit.HomeFind = 0;//参考点已找到
				ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = 0; //找0过程中bit10为0，bit12为0
                STR_CanopenHome.PosCalMethod = ObjectDictionaryStandard.HomandPosCalMethod.ActualPosCalMethod;
                //回零一旦使能后，禁止更改回零方式，否则将发生错误
                CanopenHomeLimtSwtich();
                CanopenHomingInit();
            }
            else if(CtrwdBit4==0)
            {
                STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
				STR_CanopenHome.HomingStatus.bit.HomingAttain = 0; //by huangxin201711_11由于bit10和bit12要借用HomingAttain，回零启动前就必须清标志位，否则会造成回零启动就结束的bug。
				ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1 = 0;  //未启动时bit12为0，bit10为1	,
				ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached = 1;		  //未启动时bit12为0，bit10为1
            }
        }
        //回零过程中发生暂停,对于任何开关信号均不作处理，不理会控制字的bit4是否为1
        else if(((DeviceControlVar.OEMStatus == STATUS_HALT)&&(OEMStatus == STATUS_OPERENABLE)
                  &&(1 == STR_CanopenHome.HomingStatus.bit.HomingEn)))
        {
            //不切断插补信号
            STR_CanopenHome.HomingStatus.bit.HomingEn = 2;
            CanopenHomingModeHalt();
            if(STR_ServoMonitor.StopCtrlVar.CanHaltStopMode==5)STR_CanopenHome.HomingAccQ16 = STR_LmtVar.Pos_QuickStopDecQ16;
			HomingSpeedInit(ENABLE,ZERO,ZERO);
        }
        
		STR_CanopenHome.HomingStatus.bit.OpModeSpebit = CtrwdBit4;
        
        if(1 == STR_CanopenHome.HomingStatus.bit.HomingEn)
        {
            STR_InnerGvarPosCtrl.MutexBit.bit.CanopenHomeWork = 1;
            
            //处理回零相关信号
            HomingSwtichHandle();

            if((STR_FUNC_Gvar.DivarRegLw.bit.Pot & STR_FUNC_Gvar.DivarRegLw.bit.Not)
                ||(STR_FUNC_Gvar.DivarRegLw.bit.Pot & STR_FUNC_Gvar.DivarRegLw.bit.OrgNear)
                ||(STR_FUNC_Gvar.DivarRegLw.bit.Not & STR_FUNC_Gvar.DivarRegLw.bit.OrgNear))//同时有效
            {
                STR_CanopenHome.HomingStatus.bit.HomingError = 1;
                STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
                PostErrMsg(ORIGINOVERTIME);            
            }
            
            HomingHandle();
            
        }
        else if(STR_CanopenHome.HomingStatus.bit.HomingEn == 2)
        {
            if(STR_CanopenHome.HomingStatus.bit.ZeroSpeed ==1)//暂停结束后，将回零过程结束
            {
                STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
            }
        }
        else
        {
            STR_InnerGvarPosCtrl.MutexBit.bit.CanopenHomeWork = 0;
            HomingSpeedInit(DISABLE,ZERO,STR_CanopenHome.HomingStatus.bit.HMoveDir);
        }
        //参考三洋，homing atttain 只在原点回零完成且控制字bit4为1、伺服使能的情况下有效，否则在原点回零中无效
        if((DeviceControlVar.OEMStatus == STATUS_OPERENABLE)&&(CtrwdBit4==1)
            &&(STR_CanopenHome.HomingStatus.bit.HomingAttain==1))//防止启动的时候，home attain信号即变为有效
        {
            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1 
                = ObjectDictionaryStandard.DeviceControl.StatusWord.bit.TargetReached;

            //没有定位完成前，不应该给出回零完成标志位，且应该继续计时,回零完成后，清除回零计时器
            if(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1==0)
			{
				CanopenHomingOverTimeCheck();
			}
			else
			{
	            STR_CanopenHome.TimeCount = 0;
			}
        }
        else
        {
            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1 = 0; 
        }

        //ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault;
		ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Warning; // by huangxin201803 _1 回零bit13与警告一致																		   

        OEMStatus = DeviceControlVar.OEMStatus;
    }
    else
    {
        OEMStatus  = 0;
    }
    
}


int32 CanopenHomingPosCmd(void)
{
	return (HomingSpeedHandle());  
}
/*******************************************************************************
  函数名: void HomingOverTimeCheck() 
  输  入:           
  输  出:   
  子函数: PostErrMsg()                                      
  描  述: 控制寻找原点的时间
********************************************************************************/
Static_Inline void CanopenHomingOverTimeCheck(void)
{
    STR_HOMINGVARIBLES *pHome = &STR_CanopenHome;
	
	pHome->TimeCount++;  // 搜索原点过程计时累加
 
	if(pHome->TimeCount > ((Uint32)pHome->SearchTime))
	{   
        HomingSpeedInit(DISABLE,HIGH,STR_CanopenHome.HomingStatus.bit.HMoveDir);
		STR_InnerGvarPosCtrl.MutexBit.bit.HomeWork = 0;  // 告知系统当前不处于回零过程
	    STR_InnerGvarPosCtrl.MutexBit.bit.CanopenHomeWork = 0;
        STR_CanopenHome.HomingStatus.bit.HomingEn = 0;
		STR_CanopenHome.HomingStatus.bit.HomingError = 1;
	    PostErrMsg(ORIGINOVERTIME);//原点回零超时
	}
}
/*******************************************************************************
  函数名: void CanopenHomingReset(void) 
  输  入: STR_CanopenHomeLnIntplt          
  输  出:   
  子函数: CanopenLineIntpltReset                                      
  描  述: 复位插补结构
********************************************************************************/
void CanopenHomingReset(void)
{
    HomingSpeedInit(DISABLE,ZERO,0);
}
/*******************************************************************************
  函数名: void HomingHandle(void) 
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 回零过程处理
********************************************************************************/
void HomingHandle(void)
{  	
	

	switch(STR_CanopenHome.HomingStep)
    {
        case HIGHSPEED0://高速过程
        {
            STR_CanopenHome.MoveStatus = HIGHSPEED0;
            STR_CanopenHome.TimeCount = 0;

            HomingSpeedInit(ENABLE,HIGH,STR_CanopenHome.HomingStatus.bit.HMoveDir);
			
			STR_CanopenHome.HomingStep = LIMTSWSTEP1;
        }
        break;
        case LIMTSWSTEP1://遇限位开关检测
        {
            STR_CanopenHome.MoveStatus = LIMTSWSTEP1;
			
            if(STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd == INDETERMINATE)
            {
            }
            else if(STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd == VALID)//判断是否遇限位反向
            {
                STR_CanopenHome.HomingStep = LIMTSWSTEP2;
            }
            else
            {
                STR_CanopenHome.HomingStep = DECESWSTEP1;
            }
        }
        break;
        case LIMTSWSTEP2:
        {
            STR_CanopenHome.MoveStatus = LIMTSWSTEP2;
            STR_CanopenHome.TimeCount  = 0;
            HomingSpeedInit(ENABLE,STR_CanopenHome.LimtSwtich.bit.RunSpd,STR_CanopenHome.LimtSwtich.bit.RunDir);			
			STR_CanopenHome.HomingStep = DECESWSTEP1;
        }
        break;
       
        case DECESWSTEP1:
        {
            STR_CanopenHome.MoveStatus = DECESWSTEP1;

    		//减速点有效
    		if(STR_CanopenHome.HomingStatus.bit.HomeKey ==STR_CanopenHome.DeceSwtich.bit.Logic)
    		{
                
                STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;
				
                STR_CanopenHome.HomingStep = DECESWSTEP2;
				
    		}  		
        }
        break;
        case DECESWSTEP2:
        {
            STR_CanopenHome.MoveStatus = DECESWSTEP2;
					
			if(STR_CanopenHome.DeceleratetoZero==VALID)
			{
				HomingSpeedInit(ENABLE,0,STR_CanopenHome.LimtSwtich.bit.RunDir);
				
                if(ABS(STR_FUNC_Gvar.PosCtrl.PosAmplifErr)<=100)
				{
					STR_CanopenHome.HomingStep = DECESWSTEP3;	
				}

			}
			else
			{
				STR_CanopenHome.HomingStep = DECESWSTEP3;	
			}			
  		
        }
        break;		
		
        case DECESWSTEP3:
        {
            STR_CanopenHome.MoveStatus = DECESWSTEP3;

			//减速点和反向点重合
			if(STR_CanopenHome.DeceSwtich.bit.Logic==STR_CanopenHome.RevSwtich.bit.Logic)
			{
				STR_CanopenHome.HomingStep = REVSWSTEP1;//直接进入反向
					
			}
			else
			{
				STR_CanopenHome.HomingStep = DECESWSTEP4;//减速
			}
 		
        }
        break;		
		
        
        case DECESWSTEP4:
        {
            STR_CanopenHome.MoveStatus = DECESWSTEP4;

            STR_CanopenHome.TimeCount = 0;

            HomingSpeedInit(ENABLE,STR_CanopenHome.DeceSwtich.bit.RunSpd,STR_CanopenHome.DeceSwtich.bit.RunDir);
			STR_CanopenHome.HomingStep = REVSWSTEP0; 	               

        }
        break;
      
		case REVSWSTEP0:

       	STR_CanopenHome.MoveStatus = REVSWSTEP0;
		if(STR_CanopenHome.HomingStatus.bit.HomeKey ==STR_CanopenHome.RevSwtich.bit.Logic)
		{		    
            STR_CanopenHome.HomingStep = REVSWSTEP1;
		}
		
		break;
		
		case REVSWSTEP1:

		    STR_CanopenHome.MoveStatus = REVSWSTEP1;
		
		    if(STR_CanopenHome.ReverencetoZero==VALID)
			{
				HomingSpeedInit(ENABLE,0,STR_CanopenHome.DeceSwtich.bit.RunDir);
				
				if(ABS(STR_FUNC_Gvar.PosCtrl.PosAmplifErr)<=100)
				{
					STR_CanopenHome.HomingStep = REVSWSTEP2;	
				}				
			}
			else
			{
				 STR_CanopenHome.HomingStep = REVSWSTEP2;
			}
		
		break;		
		
        case REVSWSTEP2:
        {
			STR_CanopenHome.MoveStatus = REVSWSTEP2;
            STR_CanopenHome.TimeCount = 0;

            HomingSpeedInit(ENABLE,STR_CanopenHome.RevSwtich.bit.RunSpd,STR_CanopenHome.RevSwtich.bit.RunDir);
			
			STR_CanopenHome.HomingStep = FINDINDESTEP1;

        }
        break;
		case FINDINDESTEP1:
            STR_CanopenHome.MoveStatus = FINDINDESTEP1;
			if(STR_CanopenHome.HomingStatus.bit.HomeKey==STR_CanopenHome.FindIndex.bit.Logic)
			{               
                if(STR_CanopenHome.FindIndexFlag == VALID)
				{
				    STR_CanopenHome.HomingStep = FINDINDESTEP2;
				}
				else
				{
                    STR_CanopenHome.HomingStep = HOMEOK;
					STR_CanopenHome.HomingStatus.bit.HomingClrFlag=1;
				}
			}
        break;		
        case FINDINDESTEP2:
        {
            STR_CanopenHome.MoveStatus = FINDINDESTEP2;

			if(STR_CanopenHome.HomingStatus.bit.ZIndex==VALID)
			{
                STR_CanopenHome.HomingStep = HOMEOK;
			}
        }
        break;
        case HOMEOK:
        {
			STR_CanopenHome.MoveStatus = HOMEOK;

            STR_CanopenHome.HomingStep = HOMINSUCCESS;

            STR_CanopenHome.TimeCount = 0;
            //回零使能清除
            
            STR_CanopenHome.HomingStatus.bit.ZIndex=0;

        }
        break;
        case HOMINSUCCESS://回零成功
        {
            STR_CanopenHome.MoveStatus = START0;
            
            STR_CanopenHome.HomingStep = HIGHSPPENULL;
            
            
            //置位回零成功标志
            STR_CanopenHome.HomingStatus.bit.HomingAttain = 1;
            ObjectDictionaryStandard.DeviceControl.StatusWord.bit.HomeFind = 1;//参考点已找到
            STR_CanopenHome.HomingStatus.bit.HomingEn=0;
        }
        break;
        default:
        break;
    }
    CanopenHomingOverTimeCheck();
}
/*******************************************************************************
  函数名: HomingStartSpd
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 根据不同回零模式处理回零相关信号
********************************************************************************/
Static_Inline void HomingStartSpd(void)
{
    switch(STR_CanopenHome.MoveStatus)
    {
        case START0://根据当前状态决定回零步骤
        { 
            STR_CanopenHome.MoveStatus = START1;
        }
        break;
        case START1://根据当前状态决定回零步骤
        { 
            if(STR_CanopenHome.HomingStatus.bit.HomeKey==STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd)//减速点有效
            {
                STR_CanopenHome.HomingStep = REVSWSTEP2;    
            }
            else
            {
                STR_CanopenHome.HomingStep = HIGHSPEED0;     
            }
        }
        break;       
        default:
        break;
    }

}

/*******************************************************************************
  函数名: HomingSwtichHandle(void) 
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 根据不同回零模式收集设定回零信息
********************************************************************************/
void HomingSwtichHandle(void)
{
    Uint8  NegLimitSwitch;
	Uint8  PosLimitSwitch;
	Uint8  HomeSwitch;
    
	NegLimitSwitch = ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.NegLimitSwitch;
	PosLimitSwitch = ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.PosLimitSwitch;
	HomeSwitch     = ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.HomeSwitch;

	
	switch(STR_CanopenHome.HomingMethod)
    {
        case 1:
        case 2:
        {
            if(STR_CanopenHome.HomingMethod==1)
            {
                //高低速运行方向
			    STR_CanopenHome.HomingStatus.bit.HMoveDir=NEG;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=POS;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_RISE;//上升沿反向
                
				//减速点与反向点为同一信号
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
				
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				//寻零开始点有效逻辑
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_DOWN;
                //实际信号获取
			    STR_CanopenHome.HomingStatus.bit.HomeKey = (NegLimitSwitch)
														   |(KeyLevelLastbit<<1);
                                                           	
            }
            else if(STR_CanopenHome.HomingMethod==2)
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir=POS;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_RISE;//上升沿反向
                //减速点与反向点为同一信号
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				//寻零开始点有效逻辑
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_DOWN;
                //实际信号获取
				STR_CanopenHome.HomingStatus.bit.HomeKey = (PosLimitSwitch)
                                                           |(KeyLevelLastbit<<1);
                                                 
            }

			
            STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;//碰限位开关不反转高速运行
            
            STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd  = LOGIC_POS ;//初始化进入低速减速点逻辑								 

			HomingStartSpd();

            KeyLevelLastbit = STR_CanopenHome.HomingStatus.bit.HomeKey & 0x1;

        }
        break;
        case 3:
        case 4:
        case 5:
        case 6:
        {
            if(STR_CanopenHome.HomingMethod==3)
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir=POS;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_RISE;
                //减速点与反向点为同一信号
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_DOWN;      //寻零开始点有效逻辑
				STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_POS ;//初始化进入低速减速点逻辑
			}
            else if(STR_CanopenHome.HomingMethod==4)
            {
                 //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir=NEG;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=POS;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;//反向减速点在原点开关下降沿
                //减速点与反向点为同一信号
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				//寻零开始点有效逻辑
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_RISE;
                STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_NEG ;//初始化进入低速减速点逻辑
			}
            else if(STR_CanopenHome.HomingMethod==5)
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir=NEG;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=POS;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_RISE;
                //减速点与反向点为同一信号
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_DOWN;
                STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_POS;//初始化进入低速减速点逻辑
            }
            else if(STR_CanopenHome.HomingMethod==6)
            {
                 //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir=POS;
                //反向点初始化
                STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
				STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
				STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;
                //减速点初始化
                STR_CanopenHome.DeceSwtich.all = STR_CanopenHome.RevSwtich.all;
                STR_CanopenHome.ReverencetoZero=VALID;
				STR_CanopenHome.DeceleratetoZero=VALID;
				//减速点与反向点为同一信号
				STR_CanopenHome.FindIndex.bit.Logic = LOGIC_RISE;
                STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_NEG;//初始化进入低速减速点逻辑
            }
            //实际信号获取
		    STR_CanopenHome.HomingStatus.bit.HomeKey = (HomeSwitch)
                                                       |(KeyLevelLastbit<<1);
                                                       
            STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;

			HomingStartSpd();

            KeyLevelLastbit = STR_CanopenHome.HomingStatus.bit.HomeKey & 0x1;
        }
        break;
        
        case 7:
        case 8:
        case 9:
        case 10://初始化高速过程
        {            
            if((STR_CanopenHome.HomingMethod==7)||(STR_CanopenHome.HomingMethod==10))
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir = POS;
				//找零开始点初始化
				STR_CanopenHome.FindIndex.bit.Logic       = LOGIC_DOWN;
				STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_POS ;//初始化进入低速减速点逻辑            
            }
			else if((STR_CanopenHome.HomingMethod==8)||(STR_CanopenHome.HomingMethod==9))
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir= POS;
				//寻零开始点有效逻辑
				STR_CanopenHome.FindIndex.bit.Logic      = LOGIC_RISE;
				STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd= LOGIC_POS ;//初始化进入低速减速点逻辑                        
            }  
            //实际信号获取
		    STR_CanopenHome.HomingStatus.bit.HomeKey = (HomeSwitch)
                                                       |(KeyLevelLastbit<<1);
		    STR_CanopenHome.HomingStatus.bit.HomeLimt= (PosLimitSwitch)
                                                       |(LimtKeylastbit<<1);
			
            if(STR_CanopenHome.MoveStatus==START0)
            {
                STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INDETERMINATE;
				STR_CanopenHome.MoveStatus = START1;
				STR_CanopenHome.ReverencetoZero=VALID;
            }
            else if(STR_CanopenHome.MoveStatus==START1)
            {
                //直接低速运行
				if(STR_CanopenHome.HomingStatus.bit.HomeKey==STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd)//减速点有效
                {
                    
					STR_CanopenHome.ReverencetoZero=VALID;
					STR_CanopenHome.DeceleratetoZero=VALID;
					
					if(STR_CanopenHome.HomingMethod==8)//低速回零反向
					{
                        STR_CanopenHome.HomingStep = DECESWSTEP4;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;		    
					}
					else if(STR_CanopenHome.HomingMethod==9)//低速回零反向
					{
                        STR_CanopenHome.HomingStep = DECESWSTEP4;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					else if(STR_CanopenHome.HomingMethod==7)//低速回零不反向
					{
                        STR_CanopenHome.HomingStep = REVSWSTEP2;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					else if(STR_CanopenHome.HomingMethod==10)//低速回零不反向
					{
                        STR_CanopenHome.HomingStep = REVSWSTEP2;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;
                }
                else
                {
                    STR_CanopenHome.HomingStep = HIGHSPEED0;     
                }
            }
            else if(STR_CanopenHome.MoveStatus==LIMTSWSTEP1)
            { 
				if(STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd == INDETERMINATE)
                {
                    //正常回零
					if(STR_CanopenHome.HomingStatus.bit.HomeKey == LOGIC_RISE)
                    {
						
						if(STR_CanopenHome.HomingMethod==7)
						{																					
							STR_CanopenHome.HomingStep = REVSWSTEP1;
							
							STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
							
							STR_CanopenHome.ReverencetoZero=VALID;
							STR_CanopenHome.DeceleratetoZero=VALID;
							
						}
						else if(STR_CanopenHome.HomingMethod==8)
						{
                            STR_CanopenHome.HomingStep = DECESWSTEP1;
							
							STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
							STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;

							STR_CanopenHome.RevSwtich.bit.RunDir=POS;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
							STR_CanopenHome.ReverencetoZero=VALID;
							STR_CanopenHome.DeceleratetoZero=VALID;
						    						    
					   }
						else if(STR_CanopenHome.HomingMethod==9)
						{
                            							
							STR_CanopenHome.HomingStep = DECESWSTEP1;
							
							STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
							STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;

							STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;  
                            
							STR_CanopenHome.ReverencetoZero=VALID; 
 						    STR_CanopenHome.DeceleratetoZero=INVALID;
					   }
					   else if(STR_CanopenHome.HomingMethod==10)
					   {						    
						    STR_CanopenHome.HomingStep = DECESWSTEP1;
						   
						    STR_CanopenHome.RevSwtich.bit.RunDir=POS;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
						    STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;

						    STR_CanopenHome.ReverencetoZero=INVALID;
						    STR_CanopenHome.DeceleratetoZero=INVALID;
					   }
                        //减速过程中，遇到限位开关，需要立刻反向
					   STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;
						
                    }
					//遇限位开关回零
                    else if((STR_CanopenHome.HomingStatus.bit.HomeLimt==LOGIC_RISE)||
                            (STR_CanopenHome.HomingStatus.bit.HomeLimt==LOGIC_POS)) 
                    {
                        STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = VALID;
                        STR_CanopenHome.LimtSwtich.bit.RunDir=NEG;
						STR_CanopenHome.LimtSwtich.bit.RunSpd=HIGH;	

						if(STR_CanopenHome.HomingMethod==7)
                        {
						   STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
						   STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;
							
						   STR_CanopenHome.ReverencetoZero=INVALID;
						   STR_CanopenHome.DeceleratetoZero=INVALID;
                        }
						else if(STR_CanopenHome.HomingMethod==8)
						{
                           STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;
						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
						   STR_CanopenHome.ReverencetoZero=VALID;
						   STR_CanopenHome.DeceleratetoZero=INVALID;
						   
						}
						else if(STR_CanopenHome.HomingMethod==9)
						{
                           STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						   STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;
						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;
						   
						   STR_CanopenHome.ReverencetoZero=VALID;
						   STR_CanopenHome.DeceleratetoZero=VALID;
						}
					    else if(STR_CanopenHome.HomingMethod==10)
						{						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;

						   STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;
							
						   STR_CanopenHome.ReverencetoZero=VALID; 
						   STR_CanopenHome.DeceleratetoZero=VALID;
						}
                    }
                }
            }

			KeyLevelLastbit = STR_CanopenHome.HomingStatus.bit.HomeKey & 0x1;
			LimtKeylastbit  = STR_CanopenHome.HomingStatus.bit.HomeLimt& 0x1;
        }
        break;
        case 11:
        case 12:
        case 13:
        case 14://初始化高速过程
        {
            if((STR_CanopenHome.HomingMethod==11)||(STR_CanopenHome.HomingMethod==14))
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir = NEG;
				//找零开始点初始化
				STR_CanopenHome.FindIndex.bit.Logic       = LOGIC_DOWN;
				STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd = LOGIC_POS ;//初始化进入低速减速点逻辑            
            }
			else if((STR_CanopenHome.HomingMethod==12)||(STR_CanopenHome.HomingMethod==13))
            {
                //高低速运行方向
				STR_CanopenHome.HomingStatus.bit.HMoveDir= NEG;
				//找零开始点初始化
				STR_CanopenHome.FindIndex.bit.Logic      = LOGIC_RISE;
				STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd= LOGIC_POS ;//初始化进入低速减速点逻辑                        
            }  
            //实际信号获取
		    STR_CanopenHome.HomingStatus.bit.HomeKey = (HomeSwitch)
                                                       |(KeyLevelLastbit<<1);                                        
		    STR_CanopenHome.HomingStatus.bit.HomeLimt= (NegLimitSwitch)
                                                       |(LimtKeylastbit<<1);                                          
            if(STR_CanopenHome.MoveStatus==START0)
            {
                STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INDETERMINATE;
				STR_CanopenHome.MoveStatus = START1;
            }
            else if(STR_CanopenHome.MoveStatus==START1)
            {
                //直接低速运行
				if(STR_CanopenHome.HomingStatus.bit.HomeKey==STR_CanopenHome.HomingFlag.bit.StartLogic_LowSpd)//减速点有效
                {
                    STR_CanopenHome.ReverencetoZero=VALID;
					STR_CanopenHome.DeceleratetoZero=VALID;
					//低速回零反向
					if(STR_CanopenHome.HomingMethod==12)
					{
                        STR_CanopenHome.HomingStep = DECESWSTEP4;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;		    
					}
					//低速回零反向
					else if(STR_CanopenHome.HomingMethod==13)
					{
                        STR_CanopenHome.HomingStep = DECESWSTEP4;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					else if(STR_CanopenHome.HomingMethod==11)//低速回零不反向
					{
                        STR_CanopenHome.HomingStep = REVSWSTEP2;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					else if(STR_CanopenHome.HomingMethod==14)//低速回零不反向
					{
                        STR_CanopenHome.HomingStep = REVSWSTEP2;

                        STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						
						STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						STR_CanopenHome.RevSwtich.bit.Logic = LOGIC_DOWN;					   
					}
					STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;
                }
                else
                {
                    STR_CanopenHome.HomingStep = HIGHSPEED0;     
                }
            }
            else if(STR_CanopenHome.MoveStatus==LIMTSWSTEP1)
            { 
                
				if(STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd== INDETERMINATE)
                {
                    //正常回零
					if(STR_CanopenHome.HomingStatus.bit.HomeKey == LOGIC_RISE)
                    {
						if(STR_CanopenHome.HomingMethod==11)
						{							
							STR_CanopenHome.HomingStep = REVSWSTEP1;
							
							STR_CanopenHome.RevSwtich.bit.RunDir=POS;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
							
							STR_CanopenHome.ReverencetoZero=VALID;
							STR_CanopenHome.DeceleratetoZero=VALID;
						    
						}
						else if(STR_CanopenHome.HomingMethod==12)
						{
                            						
							STR_CanopenHome.HomingStep = DECESWSTEP1;
							
							STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
							STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;

							STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
							STR_CanopenHome.ReverencetoZero=VALID;
							STR_CanopenHome.DeceleratetoZero=VALID;
						    						    
					   }
						else if(STR_CanopenHome.HomingMethod==13)
						{
                            
							STR_CanopenHome.HomingStep = DECESWSTEP1;
							
							STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
							STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;

							STR_CanopenHome.RevSwtich.bit.RunDir=POS;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
							STR_CanopenHome.ReverencetoZero=VALID;
							STR_CanopenHome.DeceleratetoZero=INVALID;
					   }
					   else if(STR_CanopenHome.HomingMethod==14)
					   {
							STR_CanopenHome.HomingStep = DECESWSTEP1;
						    
						    STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
							STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
							STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
						    STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;

						    STR_CanopenHome.ReverencetoZero=INVALID;
						    STR_CanopenHome.DeceleratetoZero=INVALID;
					   }

					   STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID;
						
                    }
					//遇限位开关回零
                    else if((STR_CanopenHome.HomingStatus.bit.HomeLimt==LOGIC_RISE)||
                            (STR_CanopenHome.HomingStatus.bit.HomeLimt==LOGIC_POS)) 
                    {
                        STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = VALID;
                        STR_CanopenHome.LimtSwtich.bit.RunDir=POS;
						STR_CanopenHome.LimtSwtich.bit.RunSpd=HIGH;

						if(STR_CanopenHome.HomingMethod==11)
                        {						 
							
						   STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;
							
						   STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;
							
						   STR_CanopenHome.ReverencetoZero=INVALID;
						   STR_CanopenHome.DeceleratetoZero=INVALID;
						   				   
                        }
						else if(STR_CanopenHome.HomingMethod==12)
						{
                           STR_CanopenHome.DeceSwtich.bit.RunDir=POS;
						   STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;
						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
						   STR_CanopenHome.ReverencetoZero=VALID; 
						   STR_CanopenHome.DeceleratetoZero=INVALID;
						}
						else if(STR_CanopenHome.HomingMethod==13)
						{
                           STR_CanopenHome.DeceSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.DeceSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.DeceSwtich.bit.Logic =LOGIC_RISE;
						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=POS;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_DOWN;
							
						   STR_CanopenHome.ReverencetoZero=VALID;
						   STR_CanopenHome.DeceleratetoZero=VALID;
						}
					    else if(STR_CanopenHome.HomingMethod==14)
						{						   
						   STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
						   STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
						   STR_CanopenHome.RevSwtich.bit.Logic =LOGIC_RISE;

						   STR_CanopenHome.DeceSwtich.all=STR_CanopenHome.RevSwtich.all;
							
						   STR_CanopenHome.ReverencetoZero=VALID;
						   STR_CanopenHome.DeceleratetoZero=VALID;
						   
						}

                    }
                }
            }
            
			KeyLevelLastbit = STR_CanopenHome.HomingStatus.bit.HomeKey & 0x1;
			LimtKeylastbit  = STR_CanopenHome.HomingStatus.bit.HomeLimt& 0x1;
        }
        break;
        case 33:
        case 34:
        {
            STR_CanopenHome.RevSwtich.bit.RunSpd=LOW;
			STR_CanopenHome.FindIndex.bit.Logic =LOGIC_RISE;
			STR_CanopenHome.HomingStatus.bit.HomeKey=STR_CanopenHome.FindIndex.bit.Logic;
            //各检测点逻辑
            if(STR_CanopenHome.HomingMethod==33)
            {
                 STR_CanopenHome.RevSwtich.bit.RunDir=NEG;
            }
            else if(STR_CanopenHome.HomingMethod==34)
            {
                 STR_CanopenHome.RevSwtich.bit.RunDir=POS;
            }
            STR_CanopenHome.HomingFlag.bit.LimtSwtich_RevHSpd = INVALID; 
                                    
            if(STR_CanopenHome.MoveStatus==START0)
            {
                STR_CanopenHome.HomingStep = REVSWSTEP2;
            }
        }
        break;


        case 35:
        {
            if(STR_CanopenHome.MoveStatus == START0)
            {
                STR_CanopenHome.HomingStep = HOMEOK;
				STR_CanopenHome.HomingStatus.bit.HomingClrFlag=1;

            }
            else
            {
                //由上次运行状态决定
            }
        }
        break;

        default:
        break;
    }

}


/*******************************************************************************
  函数名: HomingSpeedInit(Uint8 SpeedLeve,Uint8 Dir) 
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 初始化回零速度
********************************************************************************/
Static_Inline void  HomingSpeedInit(Uint8 Enable,int8 SpeedLeve,Uint8 Dir)
{
    int64 HomingSpeedQ16=0;


    if(SpeedLeve == HIGH)
    {
        HomingSpeedQ16 = STR_CanopenHome.PulseDuringSearchForSwitchQ16;
    }
    else if(SpeedLeve == LOW)
    {
        HomingSpeedQ16 = STR_CanopenHome.PulseDuringSearchForZeroQ16;
    }
	else
	{
	    HomingSpeedQ16 = 0; 
	}
    
    if(Dir ==  POS)
    {
        HomingSpeedQ16 = HomingSpeedQ16;
    }
    else
    {
        HomingSpeedQ16 = -HomingSpeedQ16;
    }
	
    STR_CanopenHome.HomingFlag.bit.RunEn = Enable;
	STR_CanopenHome.HomingSpeedRefQ16    = HomingSpeedQ16;
	//STR_CanopenHome.HomingAccQ16         = STR_CanopenHome.RiseDownPulseQ16;		
}


/*******************************************************************************
  函数名: Static_Inline int32  HomingSpeedHandle()
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 初始化回零速度
********************************************************************************/
Static_Inline int32  HomingSpeedHandle()
{
    int64 temp111;
	int32 output;
		
	
	if(STR_CanopenHome.HomingFlag.bit.RunEn==0) 
	{
	    STR_CanopenHome.HomingSpeedRem = 0;
		return 0;
	}

    if (STR_CanopenHome.HomingSpeedOutputQ16 <= STR_CanopenHome.HomingSpeedRefQ16)
    {
        
		STR_CanopenHome.HomingSpeedOutputQ16 += STR_CanopenHome.HomingAccQ16;
		
		if(STR_CanopenHome.HomingSpeedOutputQ16 > STR_CanopenHome.HomingSpeedRefQ16)
        {
            STR_CanopenHome.HomingSpeedOutputQ16 = STR_CanopenHome.HomingSpeedRefQ16;
        }
		
    }
    else
    {
		STR_CanopenHome.HomingSpeedOutputQ16 -= STR_CanopenHome.HomingAccQ16;
		
		if(STR_CanopenHome.HomingSpeedOutputQ16 < STR_CanopenHome.HomingSpeedRefQ16)
        {
            STR_CanopenHome.HomingSpeedOutputQ16 = STR_CanopenHome.HomingSpeedRefQ16;
        }
    }

	temp111 =  STR_CanopenHome.HomingSpeedOutputQ16 + STR_CanopenHome.HomingSpeedRem;		
	output  = temp111>>16;	 
	STR_CanopenHome.HomingSpeedRem    = temp111 - ((int64)output<<16); 

	if((output==0)&&(STR_CanopenHome.HomingSpeedOutputQ16==0)&&
	   (STR_CanopenHome.HomingSpeedRefQ16==0))
    {
		 STR_CanopenHome.HomingStatus.bit.ZeroSpeed = 1;
	}

	return (output);
		
}

/*******************************************************************************
  函数名: HomingClearPos(void) 
  输  入:         
  输  出:   
  子函数:                                     
  描  述: 本函数用来在找到原点后清除位置反馈、位置偏差、位置指令等 by huangxin201711_17
********************************************************************************/
void HomingClearPos(void)
{
	HomingSpeedInit(DISABLE,ZERO,STR_CanopenHome.LimtSwtich.bit.RunDir);
	STR_CanopenHome.HomingSpeedOutputQ16 =0;
   	STR_CanopenHome.HomingAccQ16 =0;

	STR_InnerGvarPosCtrl.InputPulseCnt = 0;
    STR_InnerGvarPosCtrl.PulseCalcRemainder = 0; 
	STR_ECTCSPVar.OTLatchPosCmd = 0;
	STR_FUNC_Gvar.PosCtrl.PosFdb = 0;  //!!!!! by huangxin201711_15 回零一定要清除位置反馈，否则造成位置偏差无法清除
    
	if(STR_CanopenHome.PosCalMethod == 0)//绝对回零
    {
        STR_InnerGvarPosCtrl.CurrentAbsPos = STR_CanopenHome.HomeOffsetInc;
		
    }
    else
    {
        STR_InnerGvarPosCtrl.CurrentAbsPos += STR_CanopenHome.HomeOffsetInc;
    }
    STR_InnerGvarPosCtrl.CurrentAbsPos_ToqInt = STR_InnerGvarPosCtrl.CurrentAbsPos;
    
    STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos = STR_InnerGvarPosCtrl.CurrentAbsPos;
    if((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos < 0)
    {
        STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos = 0 - STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos;
        STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos = STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos % STR_InnerGvarPosCtrl.AbsMod2PosUpLmt;
        STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos = STR_InnerGvarPosCtrl.AbsMod2PosUpLmt - STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos;
    }

    if((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos >= (int64)STR_InnerGvarPosCtrl.AbsMod2PosUpLmt)
    {
        STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos = STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos % STR_InnerGvarPosCtrl.AbsMod2PosUpLmt;
    }
    
	STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = STR_InnerGvarPosCtrl.CurrentAbsPos;
    if((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt < 0)
    {
        if(((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt + (int64)STR_InnerGvarPosCtrl.AbsMod2PosUpLmt)<=0)
        {
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = 0 - STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt;
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt % STR_InnerGvarPosCtrl.AbsMod2PosUpLmt;
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = 0 - STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt;
        }

        if(((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt + (int64)STR_InnerGvarPosCtrl.AbsMod2PosUpLmt) <= 100)
        {
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = -(STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt + STR_InnerGvarPosCtrl.AbsMod2PosUpLmt);
        }
    }
    else 
    {
        if((int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt >= (int64)STR_InnerGvarPosCtrl.AbsMod2PosUpLmt)
        {
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt % STR_InnerGvarPosCtrl.AbsMod2PosUpLmt;
        }

        if(((int64)STR_InnerGvarPosCtrl.AbsMod2PosUpLmt - (int64)STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt) <= 100)
        {
            STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt = STR_InnerGvarPosCtrl.AbsMod2PosUpLmt - STR_InnerGvarPosCtrl.AbsMod2MechSingleAbsPos_ToqInt;
        }
    }
    

	STR_FUNC_Gvar.PosCtrl.PosReguOut = 0;   // 位置调节器输出的速度指令
	STR_FUNC_Gvar.PosCtrl.PosAmplifErr = 0; // 随动偏差
	STR_InnerGvarPosCtrl.PosErrLast = 0;    // 前馈速度值及AO输出		
	STR_InnerGvarPosCtrl.FdFwdOut = 0;      // 前馈速度值及AO输出
    if((1 == UNI_FUNC_MTRToFUNC_InitList.List.AbsPosDetection)||(3 == UNI_FUNC_MTRToFUNC_InitList.List.AbsPosDetection))
    {
        AbsMod1_MultiTurnOffset();
        STR_InnerGvarPosCtrl.MutexBit.bit.AbsMod1CalcPosOffset = 1;
    }
    else if(2 == UNI_FUNC_MTRToFUNC_InitList.List.AbsPosDetection)
    {
        STR_InnerGvarPosCtrl.MutexBit.bit.AbsMod2RestRemSum = 1;
    }
	 
	FullCloseParaRst(); 
    CanopenHomeReset();
	
}




/********************************* END OF FILE *********************************/
