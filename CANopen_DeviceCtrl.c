/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "CANopen_DeviceCtrl.h"
#include "FUNC_GlobalVariable.h"
#include "FUNC_ServoMonitor.h"
#include "FUNC_FunCode.h"
#include "FUNC_ErrorCode.h"
#include "FUNC_ServoMonitor.h"
#include "PUB_Main.h"
#include "CANopen_InterFace.h"
#include "FUNC_InterfaceProcess.h"
#include "FUNC_ModeSelect.h"
#include "CANopen_Pub.h"
#include "CANopen_PP.h"
#include "CANopen_Home.h"
#include "FUNC_StopProcess.h"
#include "CANopen_Pub.h"
#include "ECT_InterFace.h"
#include "FUNC_DiDo.h"
#include "ECT_CSP.h"
#include "FUNC_PosCtrl.h"          

/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/
#define   GD_STOP_SPD     15000000      //GD停机处理速度
#define   ENGD           0  
#define   DISGD          1 
#define   QUICK_STOP_SPD  1000000       //快速停车处理速度 1rpm

#define   LIMITHYSTERESIS   3000

#if CAN_ENABLE_SWITCH
// NMT状态
#define kNode_Initialisation		0x00			// 初始化
#define kNode_Stopped				0x04			// 停止
#define kNode_Operational			0x05			// 运行
#define kNode_PreOperational		0x7F			// 预运行
#define EMCY_LIFEGUARD_ERROR_OR_HEARTBEAT_ERROR    0x8130//心跳超时，对应
#define EMCY_BUSOFF_OCCURED                        0x8141//总线脱离，对应
#define EMCY_PDO_NOT_PROCESSED_DUE_TO_LENGTH_ERROR 0x8210//PDO长度错误
#endif
/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
//Uint16		wServoStatus;
STR_DEVICECONTROLVAR DeviceControlVar = {0};
UNI_CONTROLWORD ControlWord;  
//static STR_POSCTRLLINEINTERPLT STR_PosHalt = PosCtrlLineInterpltDflts;
/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */


/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */
void UdcJundge(void);
//Static_Inline Uint8  ComControlBKEnable(void);

/*******************************************************************************
  函数名: void CanopenSupportedDriveModeSet(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   6502伺服模式规定
********************************************************************************/ 
void CanopenSupportedDriveModeSet(void)
{
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.all = 0;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.PP = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.PV = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.HM = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.PT = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.CSP = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.CSV = 1;
    ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.bit.CST = 1;
}


/*******************************************************************************
  函数名: void CanopenDIRefresh(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   canopen使用的DI信号状态更新
********************************************************************************/ 
void CanopenDIRefresh1k(void)
{
    ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.NegLimitSwitch = STR_FUNC_Gvar.DivarRegLw.bit.Not;
    ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.PosLimitSwitch = STR_FUNC_Gvar.DivarRegLw.bit.Pot;
    ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.HomeSwitch = STR_FUNC_Gvar.DivarRegLw.bit.OrgNear;

    switch(FunCodeUnion.code.CM_ECATHost)
    {
        case 2://OMRON
        union_OmronInside.bit.DI1Logic = 1 - AuxFunCodeUnion.code.DP_DIState&0x0001;
        union_OmronInside.bit.DI2Logic = 1 - ((AuxFunCodeUnion.code.DP_DIState&0x0002)>>1);
        union_OmronInside.bit.DI3Logic = 1 - ((AuxFunCodeUnion.code.DP_DIState&0x0004)>>2);
        union_OmronInside.bit.PCL = STR_FUNC_Gvar.DivarRegLw.bit.Pcl;
        union_OmronInside.bit.NCL = STR_FUNC_Gvar.DivarRegLw.bit.Ncl;
        union_OmronInside.bit.DIEmergStop = STR_FUNC_Gvar.DivarRegHi.bit.EmergencyStop;
        union_OmronInside.bit.BKOut = STR_FUNC_Gvar.Monitor.DovarReg_Blk;
        union_OmronInside.bit.STO1 = 1;
        union_OmronInside.bit.STO2 = 1;
        break;

        default:
        break;
    }

    if(FunCodeUnion.code.CM_ECATHost==2)
    {
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI5 = union_OmronInside.bit.DI1Logic;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI6 = union_OmronInside.bit.DI2Logic;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI7 = union_OmronInside.bit.DI3Logic;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI8 = union_OmronInside.bit.PCL;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI9 = union_OmronInside.bit.NCL;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.Rsvd25 = union_OmronInside.bit.DIEmergStop;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.Rsvd26 = union_OmronInside.bit.BKOut;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.Rsvd27 = union_OmronInside.bit.STO1;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.Rsvd28 = union_OmronInside.bit.STO2;
    }
    else
    {}
}
/*******************************************************************************
  函数名: void CanopenDIRefresh4k(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   canopen使用的DI信号状态更新
********************************************************************************/ 
void CanopenDIRefresh4k(void)
{
    switch(FunCodeUnion.code.CM_ECATHost)
    {
        case 2://OMRON
        union_OmronInside.bit.TouPro1 = 1-((AuxFunCodeUnion.code.DP_DIState&0x0080)>>7);
        union_OmronInside.bit.TouPro2 = 1-((AuxFunCodeUnion.code.DP_DIState&0x0100)>>8);
        union_OmronInside.bit.TouPro3 = 0;
        break;

        default:
        break;
    }

    if(FunCodeUnion.code.CM_ECATHost==2)
    {
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI2 = union_OmronInside.bit.TouPro1;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI3 = union_OmronInside.bit.TouPro2;
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI4 = union_OmronInside.bit.TouPro3;
    }
    else
    {}
}

void MCZindex(void)
{
    switch(FunCodeUnion.code.CM_ECATHost)
    {
        case 2://OMRON
        union_OmronInside.bit.ZIndex = 1;
        break;

        default:
        break;
    }

}


void MCZindexClear(void)
{
	static Uint8 ClearFlag = 0;
    static Uint32 ClearCNT = 0;
	Uint16 ZindexLatch = 0;


	ZindexLatch = union_OmronInside.bit.ZIndex;

    switch(FunCodeUnion.code.CM_ECATHost)
    {
        case 2://OMRON
		if(ZindexLatch==1)ClearFlag = 1;

        if(ClearFlag==1)
        {
            ClearCNT++;
            if(ClearCNT>(STR_ECTCSPVar.SYNCPeriodRatio<<1))
            {
                ClearCNT = STR_ECTCSPVar.SYNCPeriodRatio<<1;
                ClearFlag = 0;
                union_OmronInside.bit.ZIndex = 0;
            }
        }
        else
        {
            ClearCNT = 0;
        }
        break;

        default:
        break;
    }

	

    if(FunCodeUnion.code.CM_ECATHost==2)
    {
        ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.bit.DI1 = union_OmronInside.bit.ZIndex;
    }
    else
    {}
}

/*******************************************************************************
  函数名: void CanopenDORefresh(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   canopen使用的DO信号状态更新
********************************************************************************/ 
void ComControlBK(void)
{

    /*if(ComControlBKEnable())
    {
        STR_FUNC_Gvar.Monitor.DovarReg_Blk = ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.BitMask.bit.SetBrake;
    }*/
    ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.DOFunc.PhysicalOutput.bit.BrakeEnable = STR_FUNC_Gvar.Monitor.DovarReg_Blk;
}

/*******************************************************************************
  函数名: Uint8 ComControlBKEnable(void)

  输入:   无 
  输出:   无
  子函数:无
  描述:   通信控制抱闸使能
********************************************************************************/ 
/*Static_Inline Uint8  ComControlBKEnable(void)
{
    Uint8  EnableFlag = 0;


    if((STR_PUB_Gvar.AllInitDone == 0) ||
        (STR_FUNC_Gvar.Monitor.HighLevelErrCode !=0)||
        (STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus !=RDY)||
        (STR_FUNC_Gvar.MonitorFlag.bit.BrakeEn == 0)||
        (ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.PhysicalOutput.bit.BrakeEnable ==0) 
       )
    {
        ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.BitMask.bit.SetBrake = STR_FUNC_Gvar.Monitor.DovarReg_Blk;
        EnableFlag=0;
    }
    else
    {
        EnableFlag=1;
    }

    return EnableFlag;

}  */




/*******************************************************************************
  函数名: void ServoRunManage()
  输入:  
  输出:  
  子函数:    
  描述: 设备控制初始化，解决上电即有故障的情况下，6041无法显示伺服状态的问题
********************************************************************************/ 
void UdcJundge(void)
{

    //判断伺服母线电压是否准备好:电压检测ok 缺相检测ok
    if(STR_FUNC_Gvar.Monitor2Flag.bit.UdcOk == VALID)
    {
        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.VoltageEnabled = 1;
    }
    else
    {
        ObjectDictionaryStandard.DeviceControl.StatusWord.bit.VoltageEnabled = 0;
    }


}

/*******************************************************************************
  函数名: void CanopenDeviceControlFunc(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 设备控制主函数
********************************************************************************/
void CanopenDeviceControlFunc(void)
{
    static Uint16 ContrlWBit =0;
//	static Uint8  ServoRunStatusLast=0;
    static Uint8  AllInitDoneLast = 0;
    static Uint16  StatusStep = 0;
    static Uint8  FirFlag = 1;
	static Uint8  AlmRstFlagLast=0;
   Uint16 OEMStatus = 0;
    //static Uint8 EmergencyStop = 0;

    //伺服状态为NRD:电压 电流检测正常且无故障
    //否则，即使母线电压不正常，但伺服有故障，伺服依然显示ERR,但是状态机中必须显示伺服状态为母线电压未上，且有故障
    UdcJundge();

    ControlWord.all = (Uint16)ObjectDictionaryStandard.DeviceControl.ControlWord.all;
	STR_FUNC_Gvar.OscTarget.CtrlWord = ControlWord.all;

	if((AuxFunCodeUnion.code.FA_EmergencyStop == 1)||(STR_FUNC_Gvar.DivarRegHi.bit.EmergencyStop == 1))
	{
	    ControlWord.bit.QuickStop=0;
	}
	

    // Transition 0   初始化过程中，此时402状态机不用进行故障与否状态检测 
    if(STR_PUB_Gvar.AllInitDone == 0)
    {
        DeviceControlVar.StatusStep = STATUS_NOTRDYSWICHON;
    }
    // Transition 1   初始化完成之后  
    else if((STR_PUB_Gvar.AllInitDone == 1)&&(AllInitDoneLast==0))
    {
        //初始化完成之后，判断伺服母线电压是否准备好:电压检测ok 缺相检测ok 欠压时也认为母线电压未准备好
        if(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.VoltageEnabled == 1)
        {
            if(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault==1)
            {
                DeviceControlVar.StatusStep = STATUS_FAULT;
            }
            else    
            {
                DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
            }
        }
        else
        {
            if(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault==1)
            {
                DeviceControlVar.StatusStep = STATUS_NRD_FAULT;
            }
            else
            {
                DeviceControlVar.StatusStep = STATUS_NRD_SWITCHONDIS;
            }
        }
    }
    
    //进行ndy 与 rdy的转换判断
    if(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.VoltageEnabled == 1)
    {
        if(DeviceControlVar.StatusStep == STATUS_NRD_SWITCHONDIS)DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
        else if(DeviceControlVar.StatusStep == STATUS_NRD_RDYTOSWITCHON)DeviceControlVar.StatusStep = STATUS_RDYTOSWITCHON;
        else if(DeviceControlVar.StatusStep == STATUS_NRD_FAULT)DeviceControlVar.StatusStep = STATUS_FAULT;
		else if((DeviceControlVar.StatusStep ==STATUS_FAULTACTIVE)&&(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus != ERR)
				&&(DeviceControlVar.AlmRstFlag==0)&&(AlmRstFlagLast==1))
				{
					DeviceControlVar.StatusStep =STATUS_FAULT;
				}
    }
    else
    {
        if(DeviceControlVar.StatusStep == STATUS_SWITCHONDIS )DeviceControlVar.StatusStep = STATUS_NRD_SWITCHONDIS;
        else if(DeviceControlVar.StatusStep ==STATUS_RDYTOSWITCHON )DeviceControlVar.StatusStep =STATUS_NRD_RDYTOSWITCHON;
        else if(DeviceControlVar.StatusStep ==STATUS_FAULT )DeviceControlVar.StatusStep = STATUS_NRD_FAULT;
        
		//伺服在switch on及以上运行状态下，发生母线掉电，首先切换ready to switch on状态
        else if((DeviceControlVar.StatusStep ==STATUS_SWITCHON)||(DeviceControlVar.StatusStep ==STATUS_OPERENABLE)
            ||(DeviceControlVar.StatusStep ==STATUS_QUICKSTOPACTIVE)||(DeviceControlVar.StatusStep ==STATUS_FAULTACTIVE))
        {
            DeviceControlVar.StatusStep =STATUS_NRD_RDYTOSWITCHON;
        }
    }

    //获取控制字指令
    if((1 == ControlWord.bit.FaultReset)&&
        (DeviceControlVar.ControlWordBack.bit.FaultReset == 0))
    {
        ContrlWBit = FAULTESET;
    }
    else
	{
        ContrlWBit= (ControlWord.all & (0x0F));	    

	}
    
	switch(ContrlWBit)
	{
        case CMD_SHUTDOWN0:
        case CMD_SHUTDOWN1://STATUS_RDYTOSWITCHON
			{
                // Transition 2
                if(DeviceControlVar.StatusStep == STATUS_NRD_SWITCHONDIS)//hy nrd添加
                {
			        DeviceControlVar.StatusStep = STATUS_NRD_RDYTOSWITCHON;
                }
                else if(DeviceControlVar.StatusStep == STATUS_SWITCHONDIS)
			    {
			        DeviceControlVar.StatusStep = STATUS_RDYTOSWITCHON;
			    }
                // Transition 6
				else if(DeviceControlVar.StatusStep == STATUS_SWITCHON)
				{
				    DeviceControlVar.StatusStep = STATUS_RDYTOSWITCHON;   
				}
				// Transition 8
				else if(DeviceControlVar.StatusStep == STATUS_OPERENABLE)
				{
				    DeviceControlVar.StatusStep = STATUS_RDYTOSWITCHON;   
				}

			}			
			break;
            
        case CMD_SWITCHON0://CMD_DISABLEOPERATION
        		{
                 // Transition 3
				 if(DeviceControlVar.StatusStep == STATUS_RDYTOSWITCHON)
			     {
			         DeviceControlVar.StatusStep = STATUS_SWITCHON;
			     }
				 // Transition 4
				 /*else if(DeviceControlVar.StatusStep == STATUS_SWITCHON)
				 {
				     DeviceControlVar.StatusStep = STATUS_OPERENABLE;   
				 }*/
                 // Transition 5
				 else if(DeviceControlVar.StatusStep == STATUS_OPERENABLE)
				 {
				     DeviceControlVar.StatusStep = STATUS_SWITCHON;  
				 }
				 // Transition 16
//				 else if(DeviceControlVar.StatusStep == STATUS_QUICKSTOPACTIVE)
//				 {
//				     DeviceControlVar.StatusStep = STATUS_OPERENABLE;  
//				 }
				 
			 }
		     break;

        case CMD_ENABLEOPENRATION://CMD_ENABLEOPENRATION
			 {
                 // Transition 3 +4
				 if(DeviceControlVar.StatusStep == STATUS_RDYTOSWITCHON)
			     {
			         DeviceControlVar.StatusStep = STATUS_OPERENABLE;
			     }
				 // Transition 4
				 else if(DeviceControlVar.StatusStep == STATUS_SWITCHON)
				 {
				     DeviceControlVar.StatusStep = STATUS_OPERENABLE;   
				 }
                 
                 // Transition 5
//				 else if(DeviceControlVar.StatusStep == STATUS_OPERENABLE)
//				 {
//				     DeviceControlVar.StatusStep = STATUS_SWITCHON;  
//				 }
				 // Transition 16
				 else if((DeviceControlVar.StatusStep == STATUS_QUICKSTOPACTIVE)&&
                          (DeviceControlVar.QuickStopFinishFlag == 1))
				 {
                     if(STR_ServoMonitor.StopCtrlVar.CanQuickStopState==2)
                     {
                         DeviceControlVar.StatusStep = STATUS_OPERENABLE;
                         DeviceControlVar.QuickStopFinishFlag =0;
                         STR_ServoMonitor.StopCtrlFlag.bit.CanQuickClampFlg = 0;
                     }
				 }
			 }
		     break;
        case CMD_DISABLEVOLTAGE0:
        case CMD_DISABLEVOLTAGE1:
        case CMD_DISABLEVOLTAGE2:
        case CMD_DISABLEVOLTAGE3:
		case CMD_DISABLEVOLTAGE4:
		case CMD_DISABLEVOLTAGE5:
		case CMD_DISABLEVOLTAGE6:
		case CMD_DISABLEVOLTAGE7:
			{
                 // Transition 7
                if(DeviceControlVar.StatusStep == STATUS_NRD_RDYTOSWITCHON )//hy nrd添加
                {
			        DeviceControlVar.StatusStep = STATUS_NRD_SWITCHONDIS;
                }
				else if(DeviceControlVar.StatusStep == STATUS_RDYTOSWITCHON)
			    {
			        DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
			    }
				 // Transition 9
				else if(DeviceControlVar.StatusStep == STATUS_OPERENABLE)
				{
				    DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;   
				}
				 // Transition 10
				else if(DeviceControlVar.StatusStep == STATUS_SWITCHON)
				{
				    DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;   
				}
				 // Transition 12  本为自然过渡过程，防止上位机发送错误指令

				else if(DeviceControlVar.StatusStep == STATUS_QUICKSTOPACTIVE)
				{
                    DeviceControlVar.StatusStep = STATUS_SWITCHONDIS; 
                    DeviceControlVar.QuickStopFinishFlag =0;					    
				}


			}
			break;

		case CMD_QUICKSTOP0:
		case CMD_QUICKSTOP1:
		case CMD_QUICKSTOP2:
		case CMD_QUICKSTOP3:
			{
                 // Transition 7
                if(DeviceControlVar.StatusStep == STATUS_NRD_RDYTOSWITCHON )//hy nrd添加
                {
			        DeviceControlVar.StatusStep = STATUS_NRD_SWITCHONDIS;
                }
				else if(DeviceControlVar.StatusStep == STATUS_RDYTOSWITCHON)
			    {
			         DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
			    }
				// Transition 10
				else if(DeviceControlVar.StatusStep == STATUS_SWITCHON)
				{
				    DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;   
				}			 					 				
                 // Transition 11
				else if((DeviceControlVar.StatusStep == STATUS_OPERENABLE)
                         ||(DeviceControlVar.StatusStep == STATUS_HALT))
				{
					DeviceControlVar.StatusStep = STATUS_QUICKSTOPACTIVE;
                    STR_ServoMonitor.StopCtrlFlag.bit.CanQuickStopAck = VALID;
				}	
			}			 	
			break;
	    case FAULTESET:
			{
                 // Transition 15
				if((DeviceControlVar.StatusStep == STATUS_FAULT)
                    ||(DeviceControlVar.StatusStep == STATUS_NRD_FAULT)
                    ||(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Warning==1))
				{
                     //置位故障复位标志  
                    DeviceControlVar.AlmRstFlag = 1;    
				}
			}			 
			break;
		default:
			break;
	
	}

    OEMStatus   = DeviceControlVar.StatusStep;

	//报警复位状态转换
	if((DeviceControlVar.StatusStep==STATUS_FAULT)&&
       (DeviceControlVar.FaultFlagLast==1)&&(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault==0))
	{
		DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
        OEMStatus   = DeviceControlVar.StatusStep;

	}
	if((DeviceControlVar.StatusStep==STATUS_NRD_FAULT)&&
        (DeviceControlVar.FaultFlagLast==1)&&(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault==0))
	{
		DeviceControlVar.StatusStep = STATUS_NRD_SWITCHONDIS;
        OEMStatus   = DeviceControlVar.StatusStep;
	}
    //报警处理状态转换
	else if((DeviceControlVar.StatusStep==STATUS_FAULTACTIVE)&&
		    (STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == ERR))
	{
	    DeviceControlVar.StatusStep = STATUS_FAULT;
        OEMStatus   = DeviceControlVar.StatusStep;
	}
	//急停处理状态自然转换
	else if((DeviceControlVar.StatusStep==STATUS_QUICKSTOPACTIVE)&&
		    (DeviceControlVar.QuickStopFinishFlag == 1))
	{
        if(STR_ServoMonitor.StopCtrlVar.CanQuickStopState==0)
        {
            DeviceControlVar.StatusStep = STATUS_SWITCHONDIS; 
            DeviceControlVar.QuickStopFinishFlag =0;
            OEMStatus   = DeviceControlVar.StatusStep;
        }
        
	}
    //暂停状态保持
    //暂停命令，只在run状态下有效,避免暂停未完成，控制字halt即切换为无效，此处只进行一次判断
    else if((STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)&&
            (0x010F == (ControlWord.all&0x010F))&&(FirFlag==1))

    {
        FirFlag = 0;
        DeviceControlVar.StatusStep  = StatusStep;
        STR_ServoMonitor.StopCtrlFlag.bit.CanHaltStopAck = VALID;
        OEMStatus   = STATUS_HALT;
    }
    else if((FirFlag == 0)&&(0x010F ==(ControlWord.all&0x010F)))//暂停中指令仍为暂停

    {
        //暂停完成
        if(DeviceControlVar.HaltFinishFlag == 1)
        {
            
            if(STR_ServoMonitor.StopCtrlVar.CanHaltStopState==2)//保持位置锁定状态
            {
                DeviceControlVar.StatusStep  = StatusStep;
                OEMStatus   = STATUS_HALT;
            }
            else if(STR_ServoMonitor.StopCtrlVar.CanHaltStopState==0)
            {
                STR_ServoMonitor.StopCtrlFlag.bit.CanHaltClampFlg = 0;
                DeviceControlVar.StatusStep = STATUS_SWITCHONDIS;
                OEMStatus   = DeviceControlVar.StatusStep;
            }
            else
            {}
        }
        else//暂停未完成，内部保持暂停状态，完成后再切换
        {
            DeviceControlVar.StatusStep  = StatusStep;
            OEMStatus   = STATUS_HALT;
        }
    }
    else if((FirFlag == 0)&&((0 == ControlWord.bit.Halt)))//暂停中指令已切换
    {
        FirFlag = 1;
        STR_ServoMonitor.StopCtrlFlag.bit.CanHaltClampFlg = 0;
        DeviceControlVar.HaltFinishFlag = 0;
        OEMStatus = DeviceControlVar.StatusStep;
        STR_ServoMonitor.StopCtrlFlag.bit.CanHaltStopAck = INVALID;
    }
    
    DeviceControlVar.OEMStatus = OEMStatus;//必须用局部变量过渡，否则将存在中断被打断后，变量值为中间值的过程
	
	switch(DeviceControlVar.StatusStep)
	{
        case STATUS_NOTRDYSWICHON://初始化过程中
			 {
                 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+NOTREADYTOSWITCHON;	
			 }
			 break;
             
        case STATUS_NRD_SWITCHONDIS:
        case STATUS_SWITCHONDIS:
			 {
                 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
                 
                 if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus<=RDY)
                 {
    				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
    				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+SWITCHONDISABLEED;	
                 }
			 }
			 break;	

        case STATUS_NRD_RDYTOSWITCHON:
        case STATUS_RDYTOSWITCHON:
			 {
                 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
                 
                 if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus<=RDY)
                 {
    				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
    				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+READYTOSWITCHON;	
                 }
			 }
			 break;	
                          
        case STATUS_SWITCHON:
			 {
                 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
                 
                 if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus==RDY)
                 {
    				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
    				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+SWITCHON;	
                 }
			 }
			 break;		
        case STATUS_OPERENABLE:
			 {
                 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 1;
                 
                 if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus==RUN)
                 {
    				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
    				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+OPERATIONENABLE;	
                 }
			 	
			 }
			 break;					 
        case STATUS_QUICKSTOPACTIVE:
			 {
                 
				if(DeviceControlVar.QuickStopFinishFlag == 1)
            	{
                    if(STR_ServoMonitor.StopCtrlVar.CanQuickStopState==0)
                    {
                        DeviceControlVar.StatusStep = STATUS_SWITCHONDIS; 
                        DeviceControlVar.QuickStopFinishFlag =0;
                    }

            	}				 
				 
				 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 1;
			     ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
			     (ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+QICKSTOPACTIVE;					 				 

			 }
			 break;					 
        case STATUS_FAULTACTIVE:
			 {
				 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+FAULTREACTIONACTIVE;	
			 	
			 }
			 break;	

        case STATUS_NRD_FAULT:
        case STATUS_FAULT:
			 {
				 STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
				 ObjectDictionaryStandard.DeviceControl.StatusWord.all= 
				(ObjectDictionaryStandard.DeviceControl.StatusWord.all & 0xFFFFFF90)+FAULT;	
			 	
			 }
			 break;	
             
			 default:
			 break;
	
	}
    //锁存上次控制字的值
    DeviceControlVar.ControlWordBack.all = ControlWord.all;

    DeviceControlVar.FaultFlagLast = ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Fault;
    DeviceControlVar.WarningFlagLast = ObjectDictionaryStandard.DeviceControl.StatusWord.bit.Warning;
    StatusStep = DeviceControlVar.StatusStep;
//	ServoRunStatusLast = STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus;
    AllInitDoneLast = STR_PUB_Gvar.AllInitDone;


    GetWarningFlag();
	STR_FUNC_Gvar.OscTarget.StatusWord = (Uint16)ObjectDictionaryStandard.DeviceControl.StatusWord.all;
	AlmRstFlagLast = DeviceControlVar.AlmRstFlag;
}
/*******************************************************************************
  函数名: void CanopenQuickStopProcess(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 急停函数
********************************************************************************/
void CanopenQuickStopProcess(void)
{
    int16 QuickStopOptionCode = 0;
    
    QuickStopOptionCode =(int16)((int32)ObjectDictionaryStandard.DeviceControl.QuickStopOptionCode);

    //CSP模式--以最大转矩停车，保持位置锁定状态
    if(STR_CanSyscontrol.Mode ==ECTCSPMOD)
    {
        switch(QuickStopOptionCode)
        {
            case 0:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 0;  //停机方式: 0 自由停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 1:
            case 2:
            case 3:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 3;  //停机方式: 0 自由停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 5:
            case 6:
            case 7:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 3;  //停机方式: 3 以最大转矩停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 2 保持位置锁定状态  
                break;

            default:
                break;
        }
    }
    else if((STR_CanSyscontrol.Mode ==ECTTOQMOD)||(STR_CanSyscontrol.Mode ==ECTCSTMOD))
    {
        switch(QuickStopOptionCode)
        {
            case 0:
            case 3:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 0;  //停机方式: 0 自由停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 1:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 4;  //停机方式: 4 不同模式下的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 2:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 5;  //停机方式: 5 急停减速度的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 5: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 4;  //停机方式: 4 不同模式下的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;
                
            case 6: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 5;  //停机方式: 5 急停减速度的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;

            case 7: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 0;  //停机方式: 0 自由停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;

            default:
                break;
        }
    }
    else//即使未选择伺服模式，也选择该停车方式
    {
        switch(QuickStopOptionCode)
        {
            case 0:  
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 0;  //停机方式: 0 自由停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 1:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 4;  //停机方式: 1 不同模式下的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 2:
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 5;  //停机方式: 2 急停减速度的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态  
                break;

            case 3: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 3;  //停机方式: 3 以最大转矩停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 0;  //停机状态: 0 保持自由运行状态
                break;

            case 5: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 4;  //停机方式: 4 不同模式下的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;
                
            case 6: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 5;  //停机方式: 5 急停减速度的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;

            case 7: 
                STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  = 3;  //停机方式: 3 以最大转矩停车
                STR_ServoMonitor.StopCtrlVar.CanQuickStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;

            default:
                break;
        }
    }
}
/*******************************************************************************
  函数名: void CanopenQuickStopProcess(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 急停函数
********************************************************************************/
void CanopenQuickStopDeal(STR_SERVO_MONITOR *p)
{
    if(p->StopCtrlFlag.bit.FirCanQuickStop == 0)       //急停停机，ServoStopRun()执行一次,并置相应的标志位，在没有超程时清除
    {
        p->StopCtrlFlag.bit.FirCanQuickStop = 1;       //置ServoStopRun()执行一次标志位

         if((p->StopCtrlVar.CanQuickStopMode ==0)&&(STR_FUNC_Gvar.MonitorFlag.bit.BrakeEn == 0))
        {
            p->StopCtrlVar.CanQuickStopMode = 0;
            STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
        }
      else  if(p->StopCtrlVar.CanQuickStopMode <= 1)
        {
            p->StopCtrlVar.CanQuickStopMode = 5;
            STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
        }
        
        ServoStopRun(p->StopCtrlVar.CanQuickStopMode); //急停停机过程处理


        if(p->StopCtrlVar.CanQuickStopMode != 2) STR_FUNC_Gvar.MonitorFlag.bit.ZeroSpdStop = 0;
        else if(p->StopCtrlVar.CanQuickStopMode != 3) STR_FUNC_Gvar.MonitorFlag.bit.ToqStop = 0;
        else if((p->StopCtrlVar.CanQuickStopMode != 4)
                  &&(p->StopCtrlVar.CanQuickStopMode != 5))STR_FUNC_Gvar.MonitorFlag.bit.SlopeStop = 0;

    }
    //自由停车 以最大转矩停车
    if((STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  == 0)||
        (STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  == 3))
    {
        if((UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb < p->StopCtrlVar.StopModStateCutSpd) && 
           (UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb > -(p->StopCtrlVar.StopModStateCutSpd)))
         {
            if(p->StopCtrlVar.CanQuickStopState <= 1)
            {
                STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
            }
            else
            {
                ServoStopStatus( p->StopCtrlVar.CanQuickStopState);
            }

            STR_FUNC_Gvar.MonitorFlag.bit.ToqStop = 0;
            p->StopCtrlFlag.bit.CanQuickStopAck = INVALID;     //执行完急停响应，响应清零 
            p->StopCtrlFlag.bit.FirCanQuickStop = 0;           //急停执行一次的标志位清零 
            //置位紧急停机完成标志
            DeviceControlVar.QuickStopFinishFlag =1;

            if((STR_CanSyscontrol.Mode ==ECTPOSMOD)||(STR_CanSyscontrol.Mode ==ECTHOMMOD))
            {
                CanopenHomeReset();
	            CanopenPPReset();
				CanopenPPPosBuffReset();
                CanopenHomingReset();
            }
        }
    }
    else if((STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  == 4)||
        (STR_ServoMonitor.StopCtrlVar.CanQuickStopMode  == 5))
    {
        if((((STR_CanSyscontrol.Mode ==ECTPOSMOD) ||(STR_CanSyscontrol.Mode ==ECTHOMMOD)||(STR_CanSyscontrol.Mode ==ECTCSPMOD))
            //&&(ABS(STR_FUNC_Gvar.PosCtrl.PosAmplifErr) < STR_PosCtrlVar.PositionWindow))
            &&(ABS(UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb) <= STR_ServoMonitor.StopCtrlVar.StopModStateCutSpd))
            ||(((STR_CanSyscontrol.Mode ==ECTSPDMOD)||(STR_CanSyscontrol.Mode ==ECTCSVMOD)||(STR_CanSyscontrol.Mode ==ECTTOQMOD)||(STR_CanSyscontrol.Mode ==ECTCSTMOD))
            &&(ABS(UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb) <= STR_ServoMonitor.StopCtrlVar.StopModStateCutSpd))
           )
         {

            if(p->StopCtrlVar.CanQuickStopState <= 1)
            {
                STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
            }
            else
            {
                ServoStopStatus( p->StopCtrlVar.CanQuickStopState);
            }
            
            STR_FUNC_Gvar.MonitorFlag.bit.SlopeStop = 0; 
            p->StopCtrlFlag.bit.CanQuickStopAck = INVALID;     //执行完急停响应，响应清零 
            p->StopCtrlFlag.bit.FirCanQuickStop = 0;           //急停执行一次的标志位清零 
            //置位紧急停机完成标志
            DeviceControlVar.QuickStopFinishFlag =1;
        }
        else if(STR_CanSyscontrol.Mode==0)
        {
            if(p->StopCtrlVar.CanQuickStopState <= 1)
            {
                STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
            }
            else
            {
                ServoStopStatus( p->StopCtrlVar.CanQuickStopState);
            }
            STR_FUNC_Gvar.MonitorFlag.bit.SlopeStop = 0; 
            p->StopCtrlFlag.bit.CanQuickStopAck = INVALID;     //执行完急停响应，响应清零 
            p->StopCtrlFlag.bit.FirCanQuickStop = 0;           //急停执行一次的标志位清零 
            //置位紧急停机完成标志
            DeviceControlVar.QuickStopFinishFlag =1;
        }
    }
    else{}
}
/*******************************************************************************
  函数名: void CanopenHalt(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 急停函数
********************************************************************************/
void CanopenHalt(void)
{

    //CSP模式--以最大转矩停车，保持位置锁定状态
    if(STR_CanSyscontrol.Mode ==ECTCSPMOD)
    {
        STR_ServoMonitor.StopCtrlVar.CanHaltStopMode  = 3;  //停机方式: 3 以最大转矩停车
        STR_ServoMonitor.StopCtrlVar.CanHaltStopState = 2;  //停机状态: 0 保持位置锁定状态
    }
    else
    {
        switch(ObjectDictionaryStandard.DeviceControl.HaltOptionCode)
        {
            case 1: 
                STR_ServoMonitor.StopCtrlVar.CanHaltStopMode  = 4;  //停机方式: 3 不同模式下的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanHaltStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;
                
            case 2: 
                STR_ServoMonitor.StopCtrlVar.CanHaltStopMode  = 5;  //停机方式: 3 急停减速度的斜坡停车
                STR_ServoMonitor.StopCtrlVar.CanHaltStopState = 2;  //停机状态: 0 保持位置锁定状态
                break;

            case 3:
                if((STR_CanSyscontrol.Mode ==ECTTOQMOD)||(STR_CanSyscontrol.Mode ==ECTCSTMOD))
                {
                    STR_ServoMonitor.StopCtrlVar.CanHaltStopMode  = 0;  //停机方式: 0 自由停车
                }
                else
                {
                    STR_ServoMonitor.StopCtrlVar.CanHaltStopMode  = 3;  //停机方式: 3 以最大转矩停车
                }
                //三洋在位置模式下以最大转矩停车后，保持自由运行状态，不用确保由暂停恢复运行时的位置精度问题
                if((STR_CanSyscontrol.Mode ==ECTPOSMOD))
                {
                    STR_ServoMonitor.StopCtrlVar.CanHaltStopState  = 0;  //停机方式: 0 保持自由运行状态
                }
                else
                {
                    STR_ServoMonitor.StopCtrlVar.CanHaltStopState = 2;  //停机状态: 0 保持位置锁定状态
                }
                break;

            default:
                break;
        }
    }

}
/*******************************************************************************
  函数名: void CanopenQuickStopProcess(void)
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 急停函数
********************************************************************************/
void CanopenHaltDeal(STR_SERVO_MONITOR *p)
{
    CanopenPPPosBuffReset();
	
	if(p->StopCtrlFlag.bit.FirCanHalt == 0)       //急停停机，ServoStopRun()执行一次,并置相应的标志位，在没有超程时清除
    {
        p->StopCtrlFlag.bit.FirCanHalt = 1;       //置ServoStopRun()执行一次标志位

        if(p->StopCtrlVar.CanQuickStopMode <= 1)
        {
            p->StopCtrlVar.CanHaltStopMode = 5;
            STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
        }
        ServoStopRun(p->StopCtrlVar.CanHaltStopMode); //急停停机过程处理

        if(p->StopCtrlVar.CanHaltStopMode != 2) STR_FUNC_Gvar.MonitorFlag.bit.ZeroSpdStop = 0;
        else if(p->StopCtrlVar.CanHaltStopMode != 3) STR_FUNC_Gvar.MonitorFlag.bit.ToqStop = 0;
        else if((p->StopCtrlVar.CanHaltStopMode != 4)
                  &&(p->StopCtrlVar.CanHaltStopMode != 5))STR_FUNC_Gvar.MonitorFlag.bit.SlopeStop = 0;

    }

    if((UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb < p->StopCtrlVar.StopModStateCutSpd) && 
       (UNI_FUNC_MTRToFUNC_FastList_16kHz.List.SpdFdb > -(p->StopCtrlVar.StopModStateCutSpd)))
     {

        if(p->StopCtrlVar.CanHaltStopState <= 1)
        {
            STR_ServoMonitor.RunStateFlag.bit.CanopenServoSon = 0;
        }
        else
        {
            ServoStopStatus( p->StopCtrlVar.CanHaltStopState);
        }

        STR_FUNC_Gvar.MonitorFlag.bit.ToqStop = 0;
        STR_FUNC_Gvar.MonitorFlag.bit.SlopeStop = 0;
		 
        p->StopCtrlFlag.bit.CanHaltStopAck = INVALID;     //执行完急停响应，响应清零 
        p->StopCtrlFlag.bit.FirCanHalt = 0;           //急停执行一次的标志位清零 
        //置位暂停完成标志
        DeviceControlVar.HaltFinishFlag =1;
    }
}
/*******************************************************************************
  函数名: int32 CanopenPosModeSlopeStop(void) 
  输  入: STR_ProPos          
  输  出:   
  子函数:                                       
  描  述: Canopen位置类控制快速停车--stop on slow down ramp/slow down on quick stop ramp
********************************************************************************/
int32 CanopenPosModeSlopeStop (void)
{
    static int64 PosAccelTemp_Q16 = 0;      //加减速时速度指令中间变量
    static int64 SoftStartPosRefLatch_Q16=0;   //速度调节器输入的速度指令Q10格式
    int64   DownPulseQ16 = 0;
    int32   RealPulse = 0;
    
    if(STR_FUNC_Gvar.PosCtrl.PosRefLatch == 0)
    {
        PosAccelTemp_Q16 = 0;
        return 0;
    }
    //只用1次STR_FUNC_Gvar.PosCtrl.PosRefLatch，反复调用，会导致精度损失,提前减速到0
    if(DeviceControlVar.PosSlopeStopActiveFlag == 0)
    {
        SoftStartPosRefLatch_Q16 = (int64)STR_FUNC_Gvar.PosCtrl.PosRefLatch << 16;
        DeviceControlVar.PosSlopeStopActiveFlag = 1;
    }

    if((STR_CanSyscontrol.Mode==CANOPENHOMMOD)&&(STR_ServoMonitor.StopCtrlVar.CanQuickStopMode == 4))
    {
        CanopenPosAccLmt(ObjectDictionaryStandard.HomingMode.HomingAcceleration ,
                         ObjectDictionaryStandard.HomingMode.HomingAcceleration ,
                         ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration);
    }
    else
    {

        CanopenPosAccLmt(ObjectDictionaryStandard.ProPosMode.ProfileAcceleration ,
                         ObjectDictionaryStandard.ProPosMode.ProfileDeceleration ,
                         ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration);
    }
    
    if(STR_ServoMonitor.StopCtrlVar.CanQuickStopMode == 4)//斜坡停车
    {
        DownPulseQ16 = STR_LmtVar.Pos_ProfDecQ16;
    }
    else if(STR_ServoMonitor.StopCtrlVar.CanQuickStopMode == 5)
    {
        DownPulseQ16 = STR_LmtVar.Pos_QuickStopDecQ16;
    }

    if (STR_FUNC_Gvar.PosCtrl.PosRefLatch < 0)
    {
        PosAccelTemp_Q16 = SoftStartPosRefLatch_Q16 + DownPulseQ16;   //反向减速

        if(PosAccelTemp_Q16 < 0)
        {
            RealPulse = (int32)(PosAccelTemp_Q16 >> 16);
            SoftStartPosRefLatch_Q16 = PosAccelTemp_Q16;
        }
        else
        {
            PosAccelTemp_Q16 = 0;
		    RealPulse = (int32)(SoftStartPosRefLatch_Q16 >> 16);
            SoftStartPosRefLatch_Q16 = 0;
        }
    }
    else
    {
        PosAccelTemp_Q16 = SoftStartPosRefLatch_Q16 - DownPulseQ16;//正向减速   

        if(PosAccelTemp_Q16 > 0)
        {
            RealPulse = (int32)(PosAccelTemp_Q16 >> 16);
            SoftStartPosRefLatch_Q16 = PosAccelTemp_Q16;
        }
        else
        {
            PosAccelTemp_Q16 = 0;
			RealPulse = (int32)(SoftStartPosRefLatch_Q16 >> 16);
            SoftStartPosRefLatch_Q16 = 0;
        }
    }
    return RealPulse;               
}
/*******************************************************************************
  Uint32 NmtChangeError(Uint16 wErrType);
  输  入: wErrType，转换类型；
          wErrType = 1，情况1:           
  输  出: 0--操作成功，1--错误码 
  子函数:                                       
  描  述: 
        NMT状态切换到Initialising时，驱动器给出第二类可复位故障，
        执行故障停机，故障停机完成后6040等于0，按set键可使故障复位
        wErrType = 2，情况2；
        NMT状态切换到stopped时，驱动器给出第二类可复位故障，
        执行故障停机，故障停机完成后6040等于0，按set键可使故障复位
		否则，误操作，直接退出函数。
********************************************************************************/
#if CAN_ENABLE_SWITCH
Uint32 NmtChangeError(Uint16 wErrType)
{
    Uint32 ErrCode = 0;
	switch(wErrType)
	{
	    case 1:
	        PostErrMsg(COMMINITIALISE);
	        break;
	        
	    case 2:
	        PostErrMsg(COMMSTOP);
	        break;
	        
	    default:
	        break;
    }    
    return ErrCode;
}
#endif
/*******************************************************************************
  void CANopenTimeOut(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: 心跳超时检测
********************************************************************************/
#if CAN_ENABLE_SWITCH
void CANopenTimeOut(void)
{
	PostErrMsg(COMMCONNECTOVERTIME);
}
#endif
/*******************************************************************************
  void CANBusOffErr(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: 总线脱离错误
********************************************************************************/
#if CAN_ENABLE_SWITCH
void CANBusOffErr(void)
{
	PostErrMsg(COMBUSOFF);
}
#endif
/*******************************************************************************
  void CANRecovBusOffAlm(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: // 总线恢复警告
********************************************************************************/
#if CAN_ENABLE_SWITCH
void CANRecovBusOffAlm(void)
{
    PostErrMsg(CANRECOVBUSOFF);
}
#endif
/*******************************************************************************
  void CANPassiveErrAlm(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: // 总线被动错误警告
********************************************************************************/
#if CAN_ENABLE_SWITCH
void CANPassiveErrAlm(void)
{
    PostErrMsg(CANPASSIVEERR);
}
#endif
/*******************************************************************************
  void CANPdoLenErr(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: //总线PDO传输长度错误
********************************************************************************/
#if CAN_ENABLE_SWITCH
void CANPdoLenErr(void)
{
	PostErrMsg(COMPDOLENERR);
}
#endif
/*******************************************************************************
  void ClearCANCommErr(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: //总线PDO传输长度错误
********************************************************************************/
#if CAN_ENABLE_SWITCH
void ClearCANCommErr(void)
{
	//置位故障复位标志  
    DeviceControlVar.AlmRstFlag = 1;
}
#endif
/*******************************************************************************
  void CANopenPosLimt(void)
  输  入: 无           
  输  出: 无
  子函数: 无                                      
  描  述: 内部软件绝对位置检测
********************************************************************************/
void CANopenPosLimt(void)
{
    int64 TempMax = 0;
	int64 TempMin = 0;


	TempMax = (int64)STR_LmtVar.MaxPositionLimit - LIMITHYSTERESIS;
	TempMin = (int64)STR_LmtVar.MinPositionLimit + LIMITHYSTERESIS;
	    

	if((STR_PosCtrlVar.AbsPosActSet==1)
        ||((STR_PosCtrlVar.AbsPosActSet==2)&&(ObjectDictionaryStandard.DeviceControl.StatusWord.bit.HomeFind == 1)))
    {
        
		if(TempMax < TempMin)  
		{
		    TempMax = TempMin + LIMITHYSTERESIS;
			PostErrMsg(POSLIMITERR);
		}
		
		if(STR_InnerGvarPosCtrl.PosActualValInUser >= STR_LmtVar.MaxPositionLimit)
        {
            STR_PosCtrlVar.PosFedReachPosLimit = 1;
        }
		else if((STR_PosCtrlVar.PosFedReachPosLimit == 1)
				&&(STR_InnerGvarPosCtrl.PosActualValInUser <= TempMax))
		{
			STR_PosCtrlVar.PosFedReachPosLimit = 0;
		}
        
        if(STR_InnerGvarPosCtrl.PosActualValInUser <= STR_LmtVar.MinPositionLimit)
        {
            STR_PosCtrlVar.PosFedReachNegLimit = 1;
        }
		else if((STR_PosCtrlVar.PosFedReachNegLimit == 1)
				&&(STR_InnerGvarPosCtrl.PosActualValInUser >= TempMin))
		{
            STR_PosCtrlVar.PosFedReachNegLimit = 0;
		}
	}
	else
    {
        STR_PosCtrlVar.PosFedReachPosLimit = 0;
        STR_PosCtrlVar.PosFedReachNegLimit = 0;
		STR_PosCtrlVar.PosCmdReachLimit = 0;
    }

	//if((STR_PosCtrlVar.PosCmdReachLimit==1) ||(STR_PosCtrlVar.PosFedReachPosLimit==1)||(STR_PosCtrlVar.PosFedReachNegLimit==1))	   //by huangxin201711_6删除指令限位
	if((STR_PosCtrlVar.PosFedReachPosLimit==1)||(STR_PosCtrlVar.PosFedReachNegLimit==1)||(STR_FUNC_Gvar.DivarRegLw.bit.Pot == 1)||(STR_FUNC_Gvar.DivarRegLw.bit.Not == 1) )		  //by huangxin201711_6 删除指令限位	 //by huangxin201711_7加入硬限位也至bit11
	{
	    ObjectDictionaryStandard.DeviceControl.StatusWord.bit.InternalLimitActive = 1;
	}
	else
	{
	    ObjectDictionaryStandard.DeviceControl.StatusWord.bit.InternalLimitActive = 0;
	}



}
/*******************************************************************************
  void NodeStateCtrl(void)
  输  入: 无           
  输  出: 无
  子函数:extern Uint8 GetNodeState(void);                                     
  描  述: // 获得CANopen的NMT状态
********************************************************************************/
#if CAN_ENABLE_SWITCH
void NodeStateCtrl(void)
{
    static Uint8 NodeStateLatch = 0;
    Uint8 NodeState = 0;
    
    NodeState = GetNodeState();

    if (NodeState != NodeStateLatch)
    {
        if (STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN) // 驱动器处于运行状态
        {
            if (kNode_Initialisation == NodeState)PostErrMsg(COMMINITIALISE);
            else if (kNode_Stopped == NodeState)PostErrMsg(COMMSTOP);        
        }
        else
        {
            if(NodeState == kNode_Initialisation)DeviceControlVar.AlmRstFlag = 1;
        }

    }
    NodeStateLatch = NodeState;
} 
#endif


/*******************************************************************************
  Uint16 ComErrState(void)
  输  入: 无           
  输  出: 无
  子函数: extern Uint16 ComErrState(void);                                   
  描  述: // 获得CANopen的NMT状态
********************************************************************************/
#if CAN_ENABLE_SWITCH
void ComErrStateCtrl(void)
{
    Uint16 ErrorCode =0;
    Uint16 ErrFlag = 0;
    static Uint16 ErrFlagLatch = 0;
    
    ErrorCode = ComErrState();
    switch(ErrorCode)
    {
        case EMCY_LIFEGUARD_ERROR_OR_HEARTBEAT_ERROR:
            PostErrMsg(COMMCONNECTOVERTIME);
            ErrFlag = 1;
            break;

        case EMCY_BUSOFF_OCCURED:
            PostErrMsg(COMBUSOFF);
            ErrFlag = 1;
            break;

        case EMCY_PDO_NOT_PROCESSED_DUE_TO_LENGTH_ERROR:
            PostErrMsg(COMPDOLENERR);
            ErrFlag = 1;
            break;

        default:
            ErrFlag = 0;
            break;
    }
    if((ErrFlagLatch==1)&&(ErrFlag==0))DeviceControlVar.AlmRstFlag = 1;
    ErrFlagLatch = ErrFlag;
}
#endif
/********************************* END OF FILE *********************************/
