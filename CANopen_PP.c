/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "CANopen_PP.h"
#include "FUNC_GlobalVariable.h"
#include "FUNC_PosCtrl.h"
#include "CANopen_PV.h"
#include "FUNC_InterfaceProcess.h"
#include "FUNC_FunCode.h"
#include "FUNC_PosCmdFilter.h"
#include "FUNC_ErrorCode.h"
#include "FUNC_ServoError.h" //G2_LH_12.31 主要是提供PstErrMsg()函数
#include "FUNC_FullCloseLoop.h"
#include "CANopen_Pub.h"
#include "CANopen_DeviceCtrl.h"
#include "FUNC_FullCloseLoop.h"


/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/
// 每段起步脉冲数 1rpm--262

/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
STR_POSITIONCONTROLVARIBLES STR_PosCtrlVar;
STR_POSHALTVARIBLES STR_PosHaltVar;
STR_SETPOINTBUFFER  STR_SetPointBuffer;

static STR_POSCTRLLINEINTERPLT STR_ProPos = PosCtrlLineInterpltDflts;

extern Uint8 PV2PPSwitchFlag;
/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */


/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */
void BufferedSetPoit(void);
void BuffModeIntpltInit(void);
void ImmedModeIntpltInit(void);

/*******************************************************************************
  函数名: extern  void CanopenPPModeUpdate(void)
  输入:   无 
  输出:   
  子函数: 无       
  描述:   标准位置模式运行更新
********************************************************************************/ 
void CanopenPPModeUpdate(void)
{ 

    //6065 FollowingErrorWindow--立刻更新
    STR_PosCtrlVar.FollowingErrorWindow =(Uint32) ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorWindow;
	STR_PosCtrlVar.FollowingErrorTimeOut = (Uint32)ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorTimeOut
	                                       * STR_FUNC_Gvar.System.PosFreq /(Uint32)1000L;
    
	//6067 PositionWindow--立刻更新
    STR_PosCtrlVar.PositionWindow =(Uint32)ObjectDictionaryStandard.PosCtrlFunc1.PositionWindow;

    //6068 PositionWindowTime:ms-->位置环调度周期数--立刻更新
    STR_PosCtrlVar.PositionWindowTime   = ObjectDictionaryStandard.PosCtrlFunc1.PositionWindowTime 
                                          * STR_FUNC_Gvar.System.PosFreq /(Uint32)1000L;
}

/*******************************************************************************
  函数名: extern  void InitCanopenPPMode(void)
  输入:   无 
  输出:   
  子函数: 无       
  描述:   标准位置模式初始化
********************************************************************************/ 
void InitCanopenPPMode(void)
{ 
    RecordCanopenIntpltStruct(&STR_ProPos,0);
    FactorUpdate();

	STR_PosCtrlVar.LPStartSPD = ((int64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev<<16)/((int64)60L *(int64)STR_FUNC_Gvar.System.PosFreq);
	STR_PosCtrlVar.LPStopSPD  = ((int64)UNI_FUNC_MTRToFUNC_InitList.List.EncRev<<16)/((int64)60L *(int64)STR_FUNC_Gvar.System.PosFreq);

    CanopenPPModeUpdate();

	STR_PosCtrlVar.FollowingErrorCnt = 0;
    //增量型位置指令，位置指令锁存值为0
    PosLatchUpdate();

    //暂停结束时位置指令刚好执行完毕
    HaltVarReset();    
    //各计算的余数清零
    STR_Pos_Factor.PosRemainder = 0;
    
    //标准位置模式、回零模式、插补模式初始化
    CanopenPPReset();
    ObjectDictionaryStandard.DeviceControl.StatusWord.bit.HomeFind = 0;//参考点已找到

    //绝对位置生效条件检测
    STR_PosCtrlVar.AbsPosActSet = FunCodeUnion.code.AbsPosActSet;
    STR_PosCtrlVar.PosWinUnitSet = FunCodeUnion.code.PL_PosWinUnitSet;
	CanopenPPPosBuffReset();
}

/*******************************************************************************
  函数名: void CanopenProPosStopUpdata(void)
  输入:   无 
  输出:   无
  子函数: 无       
  描述:   标准位置模式下,PP模式运行模式更新,停机更新
  施耐德扩展了6040的bit9，具有3种运行模式
  bit9   bit5  bit4
  0      0     0--1    当前段运行时接收新的位移信息，但会将该段位置指令运行结束，才会执行新的指令
  1      0     0--1    当前段运行时接收新的位移信息，且会运行到新的目标位置，以当前速度
  x      1     0--1    当前段运行时接收新的位移信息，且立刻执行
********************************************************************************/ 
void CanopenPPModeStopUpdata(void)
{
	CanopenPPModeUpdate();

    //增量型位置指令，位置指令锁存值为当前绝对位置反馈
    PosLatchUpdate();

    //暂停结束时位置指令刚好执行完毕
    HaltVarReset();    

    //各计算的余数清零

    //标准位置模式、回零模式、插补模式初始化
    CanopenPPReset();
	CanopenPPPosBuffReset();

    //绝对位置生效条件检测
    STR_PosCtrlVar.AbsPosActSet = FunCodeUnion.code.AbsPosActSet;
    STR_PosCtrlVar.PosWinUnitSet = FunCodeUnion.code.PL_PosWinUnitSet;

    //607D SoftwarePositionLimit
    //经评审后，不与607C耦合，但仅在原点回零完成后才生效
    STR_LmtVar.MaxPositionLimit = (int32)ObjectDictionaryStandard.ProPosMode.SoftwarePositionLimit.MaxPositionLimit;
    STR_LmtVar.MinPositionLimit = (int32)ObjectDictionaryStandard.ProPosMode.SoftwarePositionLimit.MinPositionLimit;
    
     
    if(STR_LmtVar.MaxPositionLimit < STR_LmtVar.MinPositionLimit)  PostErrMsg(POSLIMITERR);

    if((((int32)ObjectDictionaryStandard.ProPosMode.HomeOffset)<STR_LmtVar.MinPositionLimit)||
        (((int32)ObjectDictionaryStandard.ProPosMode.HomeOffset)>STR_LmtVar.MaxPositionLimit))PostErrMsg(HOMEOFFSETERR);

}


/*******************************************************************************
  函数名: void CanopenClrPosReg(void)
  输入:   无 
  输出:   
  子函数: 
  描述:   仅在运行模式切换时，超程监控和超程处理中，清除位置模式下的中间变量信息
********************************************************************************/ 
void CanopenClrPosReg(void)
{
    //following error
    ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific2 = 0;

    //PP模式可接收位置指令位、回零完成、IP激活位
    STR_PosCtrlVar.CmdBusy = 0;

    //各计算的余数清零
    STR_Pos_Factor.PosRemainder = 0;

    //触发信号上升沿
    STR_PosCtrlVar.New_SetPoint = (Uint8)(((Uint16)ControlWord.all& 0x0010)>>4);
 
    STR_PosCtrlVar.New_SetPointLatch = STR_PosCtrlVar.New_SetPoint;
    
    //暂停结束时位置指令刚好执行完毕
    HaltVarReset();  

    //增量型位置指令，位置指令锁存值为当前绝对位置反馈
    PosLatchUpdate();

    STR_FUNC_Gvar.PosCtrl.DovarReg_Cmdok = 1;
}
/*******************************************************************************
  函数名: int32 CanopenPPModePosRefCal (void)
  输入:   无 
  输出:   每周期实际插补脉冲数
  子函数: CanopenUserVelLmt()
          CanopenPVVelLmt()
          CanopenUserAccLmt()
          CanopenMaxProfileCelerationLimit()
  描述:   标准位置模式下,接收新的位置指令、速度、加速度
********************************************************************************/ 
int32 CanopenPPModePosRefCal (void)
{
	Uint8  SetPointUpEdge = 0;

    static Uint8 StateLatch = 0;
    STR_POSCTRLLINEINTERPLT *pProPos = &STR_ProPos;
    
    
    STR_PosCtrlVar.CtrlWord = (Uint16)ControlWord.all;
    
    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)
    {
        //6040 bit4 new set point
        STR_PosCtrlVar.New_SetPoint         = (Uint8)((STR_PosCtrlVar.CtrlWord & 0x0010)>>4);
		STR_PosCtrlVar.ChangeSetImmediately = (Uint8)((STR_PosCtrlVar.CtrlWord & 0x0020)>>5);

        if(((DeviceControlVar.OEMStatus == STATUS_OPERENABLE)&&(StateLatch==STATUS_OPERENABLE))
		||((DeviceControlVar.OEMStatus == STATUS_OPERENABLE)&&(StateLatch==STATUS_HALT)))//运行状态
        {
            if((0== STR_PosCtrlVar.New_SetPointLatch) &&(STR_PosCtrlVar.New_SetPoint == 1))
			{
			    SetPointUpEdge = 1;
			}
            if((SetPointUpEdge == 1)&&(STR_PosCtrlVar.CmdBusy==0))//非暂停状态下，有新的位置指令产生，丛机可接收
            {
			    //将6041 bit12置1  set point acknowlwdge
                STR_PosCtrlVar.CmdBusy = 1;
			}
			if((STR_PosCtrlVar.CmdBusy==1)&&(STR_PosCtrlVar.PosBuffSt==0)&&(STR_PosCtrlVar.New_SetPoint==0))
			{
			    STR_PosCtrlVar.CmdBusy = 0;
			}
			
			if((SetPointUpEdge==1)&&(STR_PosCtrlVar.ChangeSetImmediately==1))
			{
			    ImmedModeIntpltInit();
			}
			else  if((SetPointUpEdge==1)&&(STR_PosCtrlVar.PosBuffSt==0))
			{
			    BufferedSetPoit();
				//return 0 ;
			}
			else if((pProPos->PPStatus==0)&&(STR_PosCtrlVar.PosBuffSt==1))
			{
			   BuffModeIntpltInit();
			}
        }
		else 
		{
			PosLatchUpdate();     
		}
	    
		//6040 bit5 change set immediately--立刻更新
		//STR_PosCtrlVar.PPOperModLatch = (Uint8)((STR_PosCtrlVar.CtrlWord & 0x0020)>>5);

        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1 = STR_PosCtrlVar.CmdBusy;
        STR_PosCtrlVar.New_SetPointLatch = STR_PosCtrlVar.New_SetPoint;
        StateLatch        = DeviceControlVar.OEMStatus;

        if(pProPos->PPStatus == 0)STR_FUNC_Gvar.PosCtrl.DovarReg_Cmdok = 1;
        else STR_FUNC_Gvar.PosCtrl.DovarReg_Cmdok = 0;

        return CanopenLineIntpItCal(pProPos);
    }
    else
    {
        STR_PosCtrlVar.CmdBusy = 0;        
        STR_PosHaltVar.HaltPosRefOk = 1;
        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.OperationModeSpecific1 = STR_PosCtrlVar.CmdBusy;
        return 0;
    }    
}


/*******************************************************************************
  函数名: void CanopenPPReset(void) 
  输  入: STR_ProPos          
  输  出:   
  子函数: CanopenLineIntpltReset                                      
  描  述: 复位插补结构
********************************************************************************/
void CanopenPPReset(void)
{
    // 直线插补器参数复位	
    if (1 == STR_ProPos.PPStatus)
	{
        CanopenLineIntpltReset(&STR_ProPos);
	}
}

/*******************************************************************************
  函数名: void CanopenPPPosBuffReset(void) 
  输  入: STR_ProPos          
  输  出:   
  子函数: CanopenLineIntpltReset                                      
  描  述: 复位插补结构
********************************************************************************/
void CanopenPPPosBuffReset(void)
{
	STR_PosCtrlVar.PosBuffSt = 0;
}

/*******************************************************************************
  函数名: void CanopenHomeReset(void) 
  输  入: STR_ProPos          
  输  出:   
  子函数:                                       
  描  述: 回零完成时，将与PP控制相关
********************************************************************************/
void CanopenHomeReset(void)
{
    //触发信号上升沿
    STR_PosCtrlVar.New_SetPoint = (Uint8)(((Uint16)ControlWord.all& 0x0010)>>4);
    STR_PosCtrlVar.New_SetPointLatch = STR_PosCtrlVar.New_SetPoint;
	
	STR_PosCtrlVar.PosBuffSt = 0;
    
    STR_PosCtrlVar.CmdBusy = 0;
    PosLatchUpdate();
    
    STR_ProPos.PPPlanValueRemainQ16 = 0;
    
    //暂停结束时位置指令刚好执行完毕
    HaltVarReset();    
}
/*******************************************************************************
  函数名: void HaltVarReset(void) 
  输  入: STR_PosHaltVar          
  输  出:   
  子函数:                                       
  描  述: 清除暂停变量
********************************************************************************/
void HaltVarReset(void)
{
    //暂停结束时位置指令刚好执行完毕
	STR_PosHaltVar.HaltPosRefOk      = 1;
    STR_PosHaltVar.PPAvergePulseQ16  = 0;
    STR_PosHaltVar.PPUpPulseRevQ16   = 0;
    STR_PosHaltVar.PPDownPulseRevQ16 = 0;
    STR_PosHaltVar.PPLineLengthQ16    =0;
    
}


/*******************************************************************************
  函数名: void PosLatchUpdate(void) 
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 更新位置指令锁存值
********************************************************************************/
void PosLatchUpdate(void)
{
    int64 PosTemp = 0;
    if(FunCodeUnion.code.BP_ModeSelet == 9)
	{
		if((FunCodeUnion.code.FC_FeedbackMode == 1)&&(FunCodeUnion.code.FC_ExInErrFilterTime!=0))//外部位置反馈
	    {
	        PosTemp = (int64)STR_FUNC_Gvar.PosCtrl.ExPosAmplifErr
					                        + STR_FUNC_Gvar.PosCtrl.ExCurrentAbsPos 
						                    - (int64)STR_FUNC_Gvar.PosCtrl.ExPosFdb;
	
	    }
	    else if((FunCodeUnion.code.FC_FeedbackMode == 1)&&(FunCodeUnion.code.FC_ExInErrFilterTime==0))//外部位置反馈
	    {
	        PosTemp = (int64)STR_FUNC_Gvar.PosCtrl.PosAmplifErr
					                        + STR_FUNC_Gvar.PosCtrl.ExCurrentAbsPos 
						                    - (int64)STR_FUNC_Gvar.PosCtrl.PosFdb;
	
	    }
	    else if(FunCodeUnion.code.FC_FeedbackMode == 0)
	    {
	        PosTemp = (int64)STR_FUNC_Gvar.PosCtrl.PosAmplifErr
					                        + STR_InnerGvarPosCtrl.CurrentAbsPos 
						                    - (int64)STR_FUNC_Gvar.PosCtrl.PosFdb;
	    }
	    else
	    {
	        PostErrMsg(MULTPOSCLASHFULCLOP);    //参数设置错误
	    }
        STR_PosCtrlVar.TargetPosLatch = IncpUnit2UserPosUnit(&STR_Pos_Factor_Inverse,PosTemp);

	}
}

/*******************************************************************************
  函数名: void HaltVarReset(void) 
  输  入: STR_PosHaltVar          
  输  出:   
  子函数:                                       
  描  述: 清除暂停变量
********************************************************************************/
void ImmedModeIntpltInit()
{
    Uint8 PPPosCmdType = 0;
    int32 UserTargetPos = 0;
    int64 OEMDeltaPos = 0;
	int32 DeltaPos=0;
    Uint32 UserProVel = 0;
    Uint32 UserProAcc = 0;
    Uint32 UserProDec = 0;
    Uint32 UserQStopDec = 0;

	STR_POSCTRLLINEINTERPLT *pProPos = &STR_ProPos;
    
    int64 StartPulseQ16 = 0;
    int64 RunPulseQ16 = 0;
    int64 RisePulseQ16 =0;
    int64 DownPulseQ16 =0;
	
	CanopenPPPosBuffReset();//by huangxin201711_1 根据协议，立即更新后，抛弃缓存。
	//更新位移指令 速度指令 加减速度指令
	UserTargetPos       =(int32) ObjectDictionaryStandard.ProPosMode.TargetPosition; //int32
	UserProVel          = ObjectDictionaryStandard.ProPosMode.ProfileVelocity; //Uint32
	UserProAcc          = ObjectDictionaryStandard.ProPosMode.ProfileAcceleration;  //Uint32
	UserProDec          = ObjectDictionaryStandard.ProPosMode.ProfileDeceleration;  //Uint32
	UserQStopDec        = ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration;
	
	//6040 bit6 abs/rel--位置指令类型
	PPPosCmdType   = (Uint8)((STR_PosCtrlVar.CtrlWord & 0x0040)>>6);
	
	if ((((Uint8)ObjectDictionaryStandard.ProPosMode.Polarity) & 0x80)== 0x80)//607E bit7=1位置指令反向
	{
	    UserTargetPos = 0 - UserTargetPos;
	}
	
	//计算驱动器位置指令
	if(PPPosCmdType == 0)//绝对指令
	{
	    if(PV2PPSwitchFlag==1) PV2PPSwitchFlag=0;
	}
	else
	{
	    //hy参考台达修改---立即更新模式，相对位置指令，并不抛弃当前段未走完的位置指令
	    if(PV2PPSwitchFlag==1)
	    {
	        PosLatchUpdate();
	        PV2PPSwitchFlag=0;
	    }
	    
	    UserTargetPos = UserTargetPos + STR_PosCtrlVar.TargetPosLatch;
	}
	//STR_PosCtrlVar.TargetPos = CanopenPosLmt(UserTargetPos);
	STR_PosCtrlVar.TargetPos = UserTargetPos;
	
	//方案2用位置反馈来算
	PosLatchUpdate();
	
	//不启用低通滤波与平均值滤波
	DeltaPos = STR_PosCtrlVar.TargetPos - STR_PosCtrlVar.TargetPosLatch;
	OEMDeltaPos = UserPosUnit2IncpUnit(&STR_Pos_Factor,DeltaPos);
	STR_PosCtrlVar.DeltaPosIncQ16 = OEMDeltaPos<< PPAMPBIT;  
	
	//正在执行的段位移指令锁存
	STR_PosCtrlVar.TargetPosLatch = STR_PosCtrlVar.TargetPos;
	//速度限制，转换成每个位置控制周期的位置指令
	RunPulseQ16 = CanopenPosVelLmt(UserProVel,ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity);
	
	//加速度限制--驱动器每个位置周期的位置指令增量
	CanopenPosAccLmt (UserProAcc,UserProDec,UserQStopDec);
	RisePulseQ16 = STR_LmtVar.Pos_ProfAccQ16;
	DownPulseQ16 = STR_LmtVar.Pos_ProfDecQ16;
	
	if(STR_PosCtrlVar.ChangeSetImmediately == 1)
	{
	    if((pProPos->PPRealVal>0)&&(STR_PosCtrlVar.DeltaPosIncQ16<0))pProPos->ZeroPassHandle = 1;//正向过零
	    else if((pProPos->PPRealVal<0)&&(STR_PosCtrlVar.DeltaPosIncQ16>0))pProPos->ZeroPassHandle = -1;//反向过零
	    else  pProPos->ZeroPassHandle = 0;
	    
		StartPulseQ16 = ((int64)(ABS(pProPos->PPRealVal)))<< PPAMPBIT;
	    if(StartPulseQ16==0)StartPulseQ16 = STR_PosCtrlVar.LPStartSPD; 
	
	}
	else
	{
	    StartPulseQ16= STR_PosCtrlVar.LPStartSPD;
	}
	//位移规划初始化
	CanopenLineIntpltInit(StartPulseQ16, RunPulseQ16, STR_PosCtrlVar.LPStopSPD, RisePulseQ16, DownPulseQ16,STR_PosCtrlVar.DeltaPosIncQ16, pProPos);       
}
/*******************************************************************************
  函数名: void BuffModeIntpltInit() 
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 清除暂停变量
********************************************************************************/
void BuffModeIntpltInit()
{
    Uint8 PPPosCmdType = 0;
    int32 UserTargetPos = 0;
    int64 OEMDeltaPos = 0;
	int32 DeltaPos=0;
    Uint32 UserProVel = 0;
    Uint32 UserProAcc = 0;
    Uint32 UserProDec = 0;
    Uint32 UserQStopDec = 0;
    
	STR_POSCTRLLINEINTERPLT *pProPos = &STR_ProPos;
   
    int64 StartPulseQ16 = 0;
    int64 RunPulseQ16 = 0;
    int64 RisePulseQ16 =0;
    int64 DownPulseQ16 =0;
	//更新位移指令 速度指令 加减速度指令
	UserTargetPos       = STR_SetPointBuffer.UserTargetPos; //int32
	UserProVel          = STR_SetPointBuffer.UserProVel; //Uint32
	UserProAcc          = STR_SetPointBuffer.UserProAcc;  //Uint32
	UserProDec          = STR_SetPointBuffer.UserProDec;  //Uint32
	UserQStopDec        = ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration;
	
	STR_PosCtrlVar.PosBuffSt=0;
	
	//6040 bit6 abs/rel--位置指令类型
	PPPosCmdType   = STR_SetPointBuffer.PPPosCmdType;
	
	//计算驱动器位置指令
	if(PPPosCmdType == 0)//绝对指令
	{
	    if(PV2PPSwitchFlag==1) PV2PPSwitchFlag=0;
	}
	else
	{
	    //hy参考台达修改---立即更新模式，相对位置指令，并不抛弃当前段未走完的位置指令
	    if(PV2PPSwitchFlag==1)
	    {
	        PosLatchUpdate();
	        PV2PPSwitchFlag=0;
	    }
	    
	    UserTargetPos = UserTargetPos + STR_PosCtrlVar.TargetPosLatch;
	}
	//STR_PosCtrlVar.TargetPos = CanopenPosLmt(UserTargetPos);
	STR_PosCtrlVar.TargetPos = UserTargetPos;
	
	//方案2用位置反馈来算
	// CurrentAbsPos是当前值, 所示需要减去最近一次的反馈
	//此处在使用全闭环，且外部闭环时，由于位置指令是相对于外部编码器的，必须与外部编码器的绝对位置做参考
	PosLatchUpdate();
	
	//不启用低通滤波与平均值滤波
	DeltaPos = STR_PosCtrlVar.TargetPos - STR_PosCtrlVar.TargetPosLatch;
	OEMDeltaPos = UserPosUnit2IncpUnit(&STR_Pos_Factor,DeltaPos);
	
	STR_PosCtrlVar.DeltaPosIncQ16 = OEMDeltaPos<< PPAMPBIT;  
	
	//正在执行的段位移指令锁存
	STR_PosCtrlVar.TargetPosLatch = STR_PosCtrlVar.TargetPos;
	//速度限制，转换成每个位置控制周期的位置指令
	RunPulseQ16 = CanopenPosVelLmt(UserProVel,ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity);
	
	//加速度限制--驱动器每个位置周期的位置指令增量
	CanopenPosAccLmt (UserProAcc,UserProDec,UserQStopDec);
	RisePulseQ16 = STR_LmtVar.Pos_ProfAccQ16;
	DownPulseQ16 = STR_LmtVar.Pos_ProfDecQ16;
	
//	if(STR_PosCtrlVar.ChangeSetImmediately == 1)
//	{
//	    if((pProPos->PPRealVal>0)&&(STR_PosCtrlVar.DeltaPosIncQ16<0))pProPos->ZeroPassHandle = 1;//正向过零
//	    else if((pProPos->PPRealVal<0)&&(STR_PosCtrlVar.DeltaPosIncQ16>0))pProPos->ZeroPassHandle = -1;//反向过零
//	    else
//	    {
//	        pProPos->ZeroPassHandle = 0;
//	    }
//	    StartPulseQ16 = ((int64)(ABS(pProPos->PPRealVal)))<< PPAMPBIT;
//	    if(StartPulseQ16==0)StartPulseQ16 = STR_PosCtrlVar.LPStartSPD; 
//	
//	}
//	else
//	{
	    StartPulseQ16= STR_PosCtrlVar.LPStartSPD;
//	}
	//位移规划初始化
	CanopenLineIntpltInit(StartPulseQ16, RunPulseQ16, STR_PosCtrlVar.LPStopSPD, RisePulseQ16, DownPulseQ16,STR_PosCtrlVar.DeltaPosIncQ16, pProPos);       
}
/*******************************************************************************
  函数名: void BufferedSetPoit() 
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 
********************************************************************************/
void BufferedSetPoit()
{
	STR_SetPointBuffer.UserTargetPos =(int32) ObjectDictionaryStandard.ProPosMode.TargetPosition; //int32
	
	if ((((Uint8)ObjectDictionaryStandard.ProPosMode.Polarity) & 0x80)== 0x80)//607E bit7=1位置指令反向
	{
	    STR_SetPointBuffer.UserTargetPos = 0 - STR_SetPointBuffer.UserTargetPos;
	}
	
	STR_SetPointBuffer.UserProVel    = ObjectDictionaryStandard.ProPosMode.ProfileVelocity; //Uint32
	STR_SetPointBuffer.UserProAcc    = ObjectDictionaryStandard.ProPosMode.ProfileAcceleration;  //Uint32
	STR_SetPointBuffer.UserProDec    = ObjectDictionaryStandard.ProPosMode.ProfileDeceleration;  //Uint32
	STR_SetPointBuffer.PPPosCmdType  = (Uint8)((STR_PosCtrlVar.CtrlWord & 0x0040)>>6);
	
	STR_PosCtrlVar.PosBuffSt = 1;

}

/********************************* END OF FILE *********************************/
