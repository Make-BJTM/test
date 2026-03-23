/*******************************************************************************
//深圳市汇川技术有限公司 版权所有（C）All rights reserved.
// 文件名: COMM_Modbus.c
// 创建人: 韦水平            创建日期：2011年10月 [V.001]
// 描述: Modbus协议C文件
// 修改历史:
		1.2012.3.26      韦水平
		    帧间隔时间计算通过调用宏GetSysTime_1MHzClk实现
		2.2012.3.26      韦水平
		    G_COMM_ModbusSchedual函数放到主循环中执行
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_Main.h"

#include "FUNC_FunCode.h"
#include "FUNC_COMMInterface.h"
#include "FUNC_GlobalVariable.h"
#include "COMM_System.h"
#include "COMM_ModbusDrv.h"
#include "COMM_Modbus.h"
#include "stm32f4xx.h"

/* Private_Constants ---------------------------------------------------------*/
/* 不带参数的宏定义 */

#define cTrue      1
#define cFalse     0
#define OK         1

/* Private_Macros ------------------------------------------------------------*/
/* 带参数的宏定义 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */  

/* Private_Variables ---------------------------------------------------------*/
/* 变量定义 */
COMM_RCV_MSG gaCommRcvBuff;           //接收帧缓存                          ？-调用位置较多 有风险
COMM_SEND_MSG gaCommSendBuff;         //发送帧缓存                          OK-仅在主循环中调用
UNI_MODBUS_ERR_REG gunCommErrBit;     //通信错误标志结构体                  OK-仅在主循环中调用

Uint16 gCommSendFlag = 0;             //发送标志，0禁止，1使能              OK-仅在主循环中调用
Uint16 gCommBrdcastFlag = 0;          //广播帧标志，0非广播帧，1广播帧      OK-电流环置1 主循环清0

Uint16 gRcvLenthMax = FUNCCMDLENTH;   //帧最大长度                          OK-电流环判断帧结束时设置 主循环调用
Uint16 gCommSendNum;                  //发送的字节个数                      OK-仅在主循环中调用
Uint16 gCommRcvLenth;                 //已接收字节数                        OK-电流环清0 串口中断设置
Uint16 RcvProcessRdy;                 //接收处理状态                        OK-调用位置较多
Uint16 SendProcessRdy;                //发送处理状态                        OK-调用位置较多
Uint16 gDataLen;                      //有效数据长度                        OK-仅在主循环中调用
Uint8 bContinueSending = 0;           //示波器连续采样开始标志              OK-仅在主循环中调用
Uint8 bResSending = 0;                //响应帧开始发送标志                  OK-仅在主循环中调用
Uint8 SectionNum = 0;                 //                                    OK-仅在主循环中调用

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */
void COMM_ModbusDeal_MainLoop(void);
void G_COMM_ModbusSchedual(void);

/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */ 
void ModbusDealRcvData(void);
void ModbusErrPacket(Uint8 type);
void ModbusOkPacket(Uint8 cmd);
void SetErrBit(Uint16 ret);
Uint16 ModbusCrcCalc(Uint8 *data, Uint16 length);
Uint16 ModbusErrCheck(void);
Uint16 FrameValidate(void);


/***************************************************************
  函数名:         // ModbusDealRcvData
  输入:           // 无
  输出:           // 无
  子函数:         // 无
  描述:           // 根据Modbus协议处理接收数据
***************************************************************/
void ModbusDealRcvData(void)
{
    int16 rtn = cTrue;
    Uint16 i = 0;
    Uint8 group = 0;    //起始功能码组地址
    Uint8 offset = 0;   //起始功能码偏移量
    Uint16 Ret = 0; //功能码读写函数返回结果,初始化为0,因为当命令字为0x7B时不调用功能码读写函数
    Uint8 dataTemp = 0; //临时变量，用来变量交换使用
    Uint16 type = 0;

    type = FunCodeUnion.code.CM_ModbusErrFrameType;

    rtn = ModbusErrCheck(); //帧有效性判断
    if (rtn == cFalse) //帧出错
    {
        ModbusErrPacket((Uint8)type); //错误响应帧打包
    }
    else
    {
        group = gaCommRcvBuff[GROUP]; //获取功能码组号
        offset = gaCommRcvBuff[OFFSET]; //获取功能码偏移

        if(gaCommRcvBuff[COMMCMD] == 0x10) //写多个连续功能码参数
        {
            gDataLen = (Uint16)gaCommRcvBuff[4]<<8 | gaCommRcvBuff[5]; //获取写入功能码个数
            for(i = 0; i < gDataLen; i++)
            {
                dataTemp = gaCommRcvBuff[7+i*2];
                DINT;
                gaCommRcvBuff[7+i*2] = gaCommRcvBuff[8+i*2];
                gaCommRcvBuff[8+i*2] = dataTemp;
                EINT;
            }

	        DINT;
			if(gDataLen > 1)
            {
                Ret = COMMWrRdFuncode(group,offset,gDataLen,(Uint16*)&gaCommRcvBuff[7],WRITESERIES,COMM_TYPE_MODBUS);//写入功能码      
            }
            else
            {
		        Ret = COMMWrRdFuncode(group,offset,gDataLen,(Uint16*)&gaCommRcvBuff[7],WRITEONE,COMM_TYPE_MODBUS);//写入功能码
            }
			EINT;
        }
        else if(gaCommRcvBuff[COMMCMD] == 0x03) //读多个连续功能码参数
        {
            gDataLen = (Uint16)gaCommRcvBuff[4]<<8 | gaCommRcvBuff[5]; //获取读入功能码个数
			DINT;
            Ret = COMMWrRdFuncode(group,offset,gDataLen,(Uint16*)&gaCommSendBuff[3],READSERIES,COMM_TYPE_MODBUS);
        	EINT;
		}
        else if(gaCommRcvBuff[COMMCMD] == 0x06) //写单个16位功能码
        {
            gDataLen = 1;

            dataTemp = gaCommRcvBuff[4];
            DINT;
            gaCommRcvBuff[4] = gaCommRcvBuff[5];
            gaCommRcvBuff[5] = dataTemp;            
            Ret = COMMWrRdFuncode(group,offset,gDataLen,(Uint16*)&gaCommRcvBuff[4],WRITEONE,COMM_TYPE_MODBUS);
        	EINT;
		}
        else if(gaCommRcvBuff[COMMCMD] == 0x45)  //读电子标签
        {
            gDataLen = (Uint16)gaCommRcvBuff[4]<<8 | gaCommRcvBuff[5]; //获取读入功能码个数

            if(group != 0) Ret = 2;
            else Ret = COMMWrRdEleLabel(offset,gDataLen,(Uint16*)&gaCommSendBuff[3],READSERIES);
        }
        else if(gaCommRcvBuff[COMMCMD] == 0x46)     //写电子标签
        {   
            gDataLen = (Uint16)gaCommRcvBuff[4]<<8 | gaCommRcvBuff[5]; //获取写入功能码个数

            for(i = 0; i < gDataLen; i++)
            {
                dataTemp = gaCommRcvBuff[7+i*2];
                DINT;
                gaCommRcvBuff[7+i*2] = gaCommRcvBuff[8+i*2];
                gaCommRcvBuff[8+i*2] = dataTemp;
                EINT;
            }

            if(group != 0) Ret = 2;
            else Ret = COMMWrRdEleLabel(offset,gDataLen,(Uint16*)&gaCommRcvBuff[7],WRITESERIES);//写入功能码      
        }


        if((gaCommRcvBuff[COMMCMD] == 0x10) ||
           (gaCommRcvBuff[COMMCMD] == 0x03) ||
           (gaCommRcvBuff[COMMCMD] == 0x06) ||
           (gaCommRcvBuff[COMMCMD] == 0x45) ||           
           (gaCommRcvBuff[COMMCMD] == 0x46)           
           )
        {
            if(Ret == 0) //操作正确
            {
                ModbusOkPacket(gaCommRcvBuff[COMMCMD]); //Modbus响应帧打包
            }
            else //操作失败，获取失败原因，并置相应的故障标志位
            {
                SetErrBit(Ret);
                ModbusErrPacket((Uint8)type);
            }
        }
        else if(gaCommRcvBuff[COMMCMD] == 0x7B) //读示波器缓冲区数据
        {
            ModbusOkPacket(gaCommRcvBuff[COMMCMD]); //Modbus响应帧打包
        }
    }

    if (gCommBrdcastFlag == 1) //广播帧处理
    {
        gCommBrdcastFlag = 0;
        gunCommErrBit.All = 0; //清所有错误标志位
        CleanUpErrFlag();
        SciaReadEna(TXRXENABLE); //打开接收
    }
    else
    {
         if(bResSending || bContinueSending)
         {
            gCommSendFlag = 0; 
         }
         else
         {
             gCommSendFlag = 1; //启动发送
         }
    }
}

/*************************************************
  函数名:         // SetErrBit
  输入:           // ret:Func接口函数返回值
  输出:           //
  子函数:         //
  描述:           // 根据Func接口函数返回值设置相应的故障标志位
*************************************************/
void SetErrBit(Uint16 ret)
{
    switch(ret)
    {
    case 0x02:
        gunCommErrBit.ErrBit.AddrOver = 1; //地址错误
        break;
    case 0x04:
        gunCommErrBit.ErrBit.ServoRun = 1; //伺服正在运行
        break;
    case 0x03:
        gunCommErrBit.ErrBit.ParaOver = 1; //参数值超过有效范围
        break;
    default:
        gunCommErrBit.ErrBit.CmdErr = 1;
        break;
    }
}

/*************************************************
  函数名:         // ModbusOkPacket
  输入:           // cmd:Modbus命令字
  输出:           //
  子函数:         //
  描述:           // Modbus正确响应帧打包
*************************************************/
void ModbusOkPacket(Uint8 cmd)
{
    Uint16 i;
    Uint16 CrcValue;
    Uint8 dataTemp;
    Uint16 Sum = 0;
    Uint16 LenTemp = 0;
	
//    if(bResSending || bContinueSending)
//         {
//            return;
//         }


    gaCommSendBuff[0] = gaCommRcvBuff[0]; //站号
    gaCommSendBuff[1] = gaCommRcvBuff[1]; //命令字

    switch(cmd)
    {
    case 0x03: //读功能码响应帧
    case 0x45: //读电子标签响应帧
        gaCommSendBuff[2] = gDataLen * 2; //数据长度，单位字节
        for(i = 0; i < gDataLen; i++)
        {
            dataTemp = gaCommSendBuff[3+2*i];
            gaCommSendBuff[3+2*i] = gaCommSendBuff[4+2*i];
            gaCommSendBuff[4+2*i] = dataTemp;
        }

		DINT;
		LenTemp = 3+gDataLen*2;
		if(LenTemp > 257)
		{
			CrcValue = 0;		
		}
        else
		{			
			CrcValue = ModbusCrcCalc(gaCommSendBuff, LenTemp); //计算CRC校验码
    	}
		EINT;

        gaCommSendBuff[4+2*gDataLen] = CrcValue>>8;
        gaCommSendBuff[3+2*gDataLen] = CrcValue & 0x00ff;

        gCommSendNum = 5+2*gDataLen; //总发送数据长度
        break;
    case 0x06: //写单个功能码响应帧
        for(i = 2; i < 8; i++)
        {
            if(i == 4)
            {
                gaCommSendBuff[i] = gaCommRcvBuff[i+1];
                gaCommSendBuff[i+1] = gaCommRcvBuff[i];
                i++;
            }
            else
            {
                gaCommSendBuff[i] = gaCommRcvBuff[i]; //与接收缓冲区数据相同
            }
        }

        gCommSendNum = 8; //总发送数据长度
        break;
    case 0x10: //写多个地址连续的功能码
        gaCommSendBuff[2] = gaCommRcvBuff[2]; //功能码组号
        gaCommSendBuff[3] = gaCommRcvBuff[3]; //功能码偏移
        gaCommSendBuff[4] = (gDataLen>>8) & 0x00ff; //写入功能码个数高位
        gaCommSendBuff[5] = gDataLen & 0x00ff;      //写入功能码个数低位
		DINT;
        CrcValue = ModbusCrcCalc(gaCommSendBuff, 6); //计算CRC校验码
		EINT;
        gaCommSendBuff[7] = CrcValue>>8;
        gaCommSendBuff[6] = CrcValue & 0x00ff;

        gCommSendNum = 8;
        break;
    case 0x7B: //读示波器缓冲区响应帧
        gaCommSendBuff[2] = 0xAA;
        gaCommSendBuff[3] = 0x0F;
        gaCommSendBuff[4] = 0x55;
        gaCommSendBuff[5] = 0xF0;

        if(!bContinueSending)
        {
            gaCommSendBuff[6] = gaCommRcvBuff[3];
        }
        else
        {
            gaCommSendBuff[6] = SectionNum;
        }

        Sum = 0;
        for(i = 0; i < 7; i++)
        {
            Sum += gaCommSendBuff[i];
        }

        for(i = 0; i < 128; i++)
        {
            gaCommSendBuff[7 + i] = *(UNI_OsciBuffer.all_8Bits + gaCommSendBuff[6] * 128 + (AuxFunCodeUnion.code.OS_Part<<13) + i);
            Sum += gaCommSendBuff[7 + i];
        }


        if ((gaCommSendBuff[6] == 63) && (AuxFunCodeUnion.code.OS_Part == 1) && (AuxFunCodeUnion.code.H2F_FSAState == 1))
        {
            AuxFunCodeUnion.code.H2F_FSAState = 2;
        }

        if ((gaCommSendBuff[6] == 63) && (AuxFunCodeUnion.code.OS_Part == 2) && (AuxFunCodeUnion.code.H2F_FSAState == 2))
        {
            AuxFunCodeUnion.code.H2F_FSAState = 0;
        }
        gaCommSendBuff[135] = Sum & 0x00ff;
        gaCommSendBuff[136] = Sum>>8;
        gCommSendNum = 137;
        break;
    case 0x46: //写电子标签响应帧
        gaCommSendBuff[2] = gaCommRcvBuff[2]; //功能码组号
        gaCommSendBuff[3] = gaCommRcvBuff[3]; //功能码偏移

        if(gaCommRcvBuff[3] >= 0x20)
        {
            gaCommSendBuff[4] = (gDataLen>>8) & 0x00ff; //写入功能码个数高位
            gaCommSendBuff[5] = gDataLen & 0x00ff;      //写入功能码个数低位
        }
        else
        {
            //下载加密特殊处理
            gaCommSendBuff[4] = gaCommRcvBuff[4]; 
            gaCommSendBuff[5] = gaCommRcvBuff[5];         
        }
		DINT;
        CrcValue = ModbusCrcCalc(gaCommSendBuff, 6); //计算CRC校验码
		EINT;
        gaCommSendBuff[7] = CrcValue>>8;
        gaCommSendBuff[6] = CrcValue & 0x00ff;

        gCommSendNum = 8;
        break;
    default:
        break;
    }
}

/*************************************************
  函数名:         // ModbusErrPacket
  输入:           //
  输出:           //
  子函数:         //
  描述:           // Modbus错误响应帧打包
*************************************************/
void ModbusErrPacket(Uint8 type)
{
    Uint16 crcValue;
    Uint8 errPos1 = 0;
	Uint8 errPos2 = 0;
	Uint8 crcPos = 0; 
	Uint16 LenTemp = 0;

//        if(bResSending || bContinueSending)
//         {
//            return;
//         }
    if((!bContinueSending)&& (gaCommRcvBuff[COMMCMD] != 0x7B))
    {
        gaCommSendBuff[0] = gaCommRcvBuff[ADDR];
        if(type == 0) //兼容以前的错误返回帧格式
        {
            gaCommSendBuff[1] = gaCommRcvBuff[COMMCMD];
            gaCommSendBuff[2] = 0x80;
            gaCommSendBuff[3] = 0x01;
            errPos1 = 4; //错误码在错误响应帧的起始位置
			errPos2 = errPos1+1; //错误码在错误响应帧的结束位置
			crcPos = 6;	//CRC检验码在错误响应帧的位置
			gCommSendNum = 8;
        }
        else
        {
			//标准错误返回帧
            gaCommSendBuff[1] = gaCommRcvBuff[COMMCMD] | 0x0080; //错误标志:命令字+0x80
            errPos2 = errPos1 = 2; //错误码在错误响应帧的位置
			crcPos = 3; //CRC检验码在错误响应帧的位置
			gCommSendNum = 5;
        }
    }
    else
    {
        gaCommSendBuff[0] = (Uint8)FunCodeUnion.code.CM_AxisAdress;
        gaCommSendBuff[1] = 0x7B | 0x80;
        gaCommSendBuff[2] = 0xAA;
        gaCommSendBuff[3] = 0x0F;
        gaCommSendBuff[4] = 0x55;
        gaCommSendBuff[5] = 0xF0;
        errPos1 = 6; //错误码在错误响应帧的起始位置
		errPos2 = errPos1+1; //错误码在错误响应帧的结束位置
		crcPos = 8;	//CRC检验码在错误响应帧的位置
		gCommSendNum = 10;
    }

    if (gunCommErrBit.ErrBit.CmdErr == 1)
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = type?0x01:0x02;
    }
    else if (gunCommErrBit.ErrBit.CrcChkErr == 1)
    {
		if(errPos2 == 2) //cmd not equal to 0x7B,send crc error message
		{
			gCommSendNum = 0;
			gunCommErrBit.All = 0;
            
			SciaSendEna(TXRXDISABLE);
            CleanUpErrFlag();
			SciaReadEna(TXRXENABLE);  //打开接收
			return;	
		}

		gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = 0x04;
    }
    else if (gunCommErrBit.ErrBit.AddrOver == 1)
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = type?0x02:0x08;
    }
    else if (gunCommErrBit.ErrBit.ParaOver == 1)
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = type?0x03:0x10;
    }
    else if (gunCommErrBit.ErrBit.ServoRun == 1)
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = type?0x04:0x80;
    } /*
    else if(gunCommErrBit.ErrBit.ZeroLength == 1) //读取参数长度为0
    {
        gaCommSendBuff[errPos1] = 0x00;
		gaCommSendBuff[errPos2] = type?0x05:0x03;
    }
    else if(gunCommErrBit.ErrBit.DataTypeNotMatch == 1) //数据类型不符合
    {
        gaCommSendBuff[errPos1] = 0x00;
		gaCommSendBuff[errPos2] = type?0x05:0x03;
    }*/
    else if(gunCommErrBit.ErrBit.DataReadDeny == 1) //扇区禁止读
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = 0x20;
    }
    else if(gunCommErrBit.ErrBit.BlockNotExist == 1) //扇区不存在
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = 0x08;
    }
    else if(gunCommErrBit.ErrBit.DataLengthExceed == 1) //读取数据长度不为255
    {
        gaCommSendBuff[errPos1] = 0x00;
        gaCommSendBuff[errPos2] = 0x06;
    }
    else
    {
        gaCommSendBuff[errPos1] = gunCommErrBit.All;
        gaCommSendBuff[errPos2] = gunCommErrBit.All >> 8;
    }

    //写错误码，地址待定
    FunCodeUnion.code.CM_ErrorType = gaCommSendBuff[errPos2];

    DINT;
	LenTemp = gCommSendNum-2;
	if(LenTemp > 257)
	{
		crcValue = 0;
	}
	else
	{
		crcValue = ModbusCrcCalc(gaCommSendBuff, LenTemp);//CRC校验不包括CRC部分，错误帧总长度为6		
	}
   	EINT;   

	gaCommSendBuff[crcPos] = crcValue & 0x00ff;
    gaCommSendBuff[crcPos+1] = (crcValue >> 8);
}

/*************************************************
  函数名:         // ModbusCrcCalc
  输入:           // data:帧首地址;length:帧长度
  输出:           // CRC校验值
  子函数:         //
  描述:           // CRC校验值计算
*************************************************/
Uint16 ModbusCrcCalc(Uint8 *data, Uint16 length)
{
    Uint16 crcValue = 0xffff;
    int16 i;

	Uint8 *pp;

	Uint16 Temp1;

    pp = data;
   
    while (length--)
    {
        if((pp - data) > 257) return 0;
        
        Temp1 = (Uint16)(*pp);
		crcValue = crcValue ^ Temp1;
		
		
		//crcValue ^= (Uint16)*data++;

        for (i = 0; i < 8; i++)
        {
            if (crcValue & 0x0001)
            {
                crcValue = (crcValue >> 1) ^ 0xA001;
            }
            else
            {
                crcValue = crcValue >> 1;
            }
        }

		pp++;
    }

    return (crcValue);
}

/**************************************************************
  函数名:         // ModbusErrCheck
  输入:           //
  输出:           // 报文检查结果: 0-无错误;1-错误
  子函数:         //
  描述:           // 根据Modbus协议对Modbus输入报文进行检查
**************************************************************/
Uint16 ModbusErrCheck(void)
{
    Uint16 data = 0;
    Uint16 crcValue;
    Uint16 cmd;
	Uint16 shiftNum = 0;
	Uint8 byteNum = 0;
	Uint16 CrcTemp = 0;	

    data = ((Uint16)gaCommRcvBuff[DATALH] << 8) |  gaCommRcvBuff[DATALL];

    crcValue = ((Uint16)gaCommRcvBuff[gRcvLenthMax -1] << 8) | gaCommRcvBuff[gRcvLenthMax - 2]; //CRC校验不包括CRC域
		
	DINT;
	if((gRcvLenthMax - 2) > 257)
	{
		CrcTemp = 0;
	}
	else
	{
		CrcTemp	= ModbusCrcCalc(gaCommRcvBuff, (gRcvLenthMax - 2));
	}
	EINT;

    if (crcValue != CrcTemp)
    {
       gunCommErrBit.ErrBit.CrcChkErr = 1; //CRC 校验错误
       return RXERROR;
    }

    cmd = gaCommRcvBuff[COMMCMD];

    if ((cmd != 0x03) && (cmd != 0x06) && (cmd != 0x10) && (cmd != 0x7B) && (cmd != 0x45) && (cmd != 0x46))//判断命令是否有效
    {
        gunCommErrBit.ErrBit.CmdErr = 1; //Modbus命令字错误码
        return RXERROR;
    }

    if (gaCommRcvBuff[COMMCMD] == 0x03)
    {
        if (data > READDATALENTH)//读取数据长度大于125字长，置参数错误
        {
            gunCommErrBit.ErrBit.ParaOver = 1;
            return RXERROR;
        }

        if(data == 0)
        {
            gunCommErrBit.ErrBit.ParaOver = 1; //读取数据长度为0
            return RXERROR;
        }
    }
    else if(gaCommRcvBuff[COMMCMD] == 0x10)
    {
		byteNum = gaCommRcvBuff[6];
        if((data == 0) || (byteNum == 0x00) || 
        	(data > WRITEDATALENTH) || (data * 2 != byteNum))
        {
            gunCommErrBit.ErrBit.ParaOver = 1; //写入寄存器数量和字节数不匹配
            return RXERROR;
        }
    }
    else if(gaCommRcvBuff[COMMCMD] == 0x7B)
    {
        if(gaCommRcvBuff[3] > 63)
        {
            gunCommErrBit.ErrBit.BlockNotExist = 1; //扇区不存在
            return RXERROR;
        }
		else
		{
			if(gaCommRcvBuff[3] < 16)
			{
                data = AuxFunCodeUnion.code.OS_SectionCtrlA;     //读扇区状态
				shiftNum = gaCommRcvBuff[3];
			}
			else if((gaCommRcvBuff[3] > 15) && (gaCommRcvBuff[3] < 32))
			{
                data = AuxFunCodeUnion.code.OS_SectionCtrlB;     //读扇区状态
				shiftNum = gaCommRcvBuff[3] - 16;
			}
			else if((gaCommRcvBuff[3] > 31) && (gaCommRcvBuff[3] < 48))
			{
                data = AuxFunCodeUnion.code.OS_SectionCtrlC;     //读扇区状态
				shiftNum = gaCommRcvBuff[3] - 32;
			}
			else if((gaCommRcvBuff[3] > 47) && (gaCommRcvBuff[3] < 64))
			{
                data = AuxFunCodeUnion.code.OS_SectionCtrlD;     //读扇区状态
				shiftNum = gaCommRcvBuff[3] - 48;	     
			}

			if(gaCommRcvBuff[5] != 64)
	        {
	            gunCommErrBit.ErrBit.DataLengthExceed = 1; //数据长度超过有效范围
	            return RXERROR;
	        }
	        else if((data & (0x0001<<shiftNum)) != 0) //示波器正在采样,禁止读
	        {
	            gunCommErrBit.ErrBit.DataReadDeny = 1; //扇区禁止操作
	            return RXERROR;
	        }	
		}    
    }
    else if (gaCommRcvBuff[COMMCMD] == 0x45)
    {
        if (data > 0x30)//读取数据长度大于48字长，置参数错误
        {
            gunCommErrBit.ErrBit.ParaOver = 1;
            return RXERROR;
        }

        if(data == 0)
        {
            gunCommErrBit.ErrBit.ParaOver = 1; //读取数据长度为0
            return RXERROR;
        }
    }
    else if(gaCommRcvBuff[COMMCMD] == 0x46)
    {
		byteNum = gaCommRcvBuff[6];
        if((data == 0) || (byteNum == 0x00) || 
        	(data > 0x10) || (data * 2 != byteNum))
        {
            gunCommErrBit.ErrBit.ParaOver = 1; //写入寄存器数量和字节数不匹配
            return RXERROR;
        }
    }

    return cTrue;
}

/**************************************************************
  函数名:         // ContinueSendingDeal
  输入:           //
  输出:           //
  子函数:         //
  描述:           //连续采样处理函数
**************************************************************/
void ContinueSendingDeal(void)
{
    Uint8  flag;
    Uint8  bSendEna;

    if(bResSending || bContinueSending)
    {
        return; //响应帧数据或者上次采样数据正在发送时返回
    }

    if((AuxFunCodeUnion.code.OS_SeriesTxCtrl & 0x9000) == 0x9000) //数据准备好
    {
        SectionNum = AuxFunCodeUnion.code.OS_SeriesTxCtrl & 0x003f; //低6位为扇区号
        flag = 0;
    }
    else
    {
        flag = 2;
    }

    if(flag == 0)
    {
        //packet datas and prepare to send
        bContinueSending = 1; //连续采样数据开始发送
        DINT;
        gaCommRcvBuff[1] = 0x7B;  //wzg 读数据缓冲区命令码
        EINT;
        ModbusOkPacket(0x7B);
        gCommSendNum = 9 + 2 * 64;
        bSendEna = 1;
    }
    else
    {
        bContinueSending = 0;
        bSendEna = 0;
    }

    if((bSendEna) && (0 == DMA_GetCmdStatus(DMA2_Stream7)))
    {
        UART_TX_DMA_LowLevelConfig((Uint32)(&gaCommSendBuff[0]), gCommSendNum);
        CleanUpErrFlag(); //清除usart故障信息

        SciaSendEna(TXRXENABLE);//打开SCI发送                 
        SciaReadEna(TXRXENABLE);//打开SCI接收
        
        AuxFunCodeUnion.code.OS_UartStatus |= 1;   //发送中 BIT0 = 1
    }

}

/**************************************************************
  函数名:         // COMM_ModbusDeal_MainLoop
  输入:           //
  输出:           //
  子函数:         //
  描述:           //Modbus模块调度函数
**************************************************************/
void COMM_ModbusDeal_MainLoop(void)
{
    static Uint16 ComSendDelayCnt = 0;           //发送延时
    Uint16 gCommSendCmpltflag = 0;

    //如果FPGA中断发送故障 
    if(1 == STR_FUNC_Gvar.ScheldularFlag.bit.FPGAIntErr) G_COMM_ModbusSchedual();

    COMM_UpdateSciSet();   

    if (RcvProcessRdy == cRCVPROCESSRDY) //接收完一帧数据
    {
        AuxFunCodeUnion.code.OS_UartStatus &= 1;   //接收完成 BIT1 = 0
        RcvProcessRdy=cRCVPROCESSNORDY;
        ModbusDealRcvData();//处理Modbus报文
    }

    if ((gCommSendFlag == 1) && (SendProcessRdy==cSENDPROCESSRDY) && 
		(gCommSendNum > 0))
    {
        if(ComSendDelayCnt < FunCodeUnion.code.CM_SendDelay)
        {
            ComSendDelayCnt++;
        }
        else
        {
            if((!bContinueSending) && (0 == DMA_GetCmdStatus(DMA2_Stream7))) //连续发送未开始,启动常规数据发送,直到常规数据发完
            {                
                SciaReadEna(TXRXDISABLE); //屏敝接收
                UART_TX_DMA_LowLevelConfig((Uint32)(&gaCommSendBuff[0]), gCommSendNum);

                CleanUpErrFlag(); //清除usart故障信息 
                SciaSendEna(TXRXENABLE);//打开SCI发送

                AuxFunCodeUnion.code.OS_UartStatus |= 1;  //发送中 BIT0 = 1

                bResSending = 1;
                gCommSendFlag = 0;
                SendProcessRdy = cSENDPROCESSNORDY;
            }
        }
    }

    ContinueSendingDeal(); //连续发送处理

    if(bResSending || bContinueSending)
    {
        if ((0 == DMA_GetCmdStatus(DMA2_Stream7)) && (GetSciStatus() &  0x0040)) //发送完成
        {
            DMA_Cmd(DMA2_Stream7, DISABLE);   /* Enable the DMA Tx Channel */                
            gCommSendCmpltflag = 1; //发送完成标志置1
            gunCommErrBit.All = 0;  //清所有错误标志位
            
            DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_FEIF7 | DMA_IT_DMEIF7 | DMA_IT_TEIF7 | DMA_IT_HTIF7 | DMA_IT_TCIF7);
            CleanUpErrFlag(); //清除usart故障信息
        }
    }

    if (gCommSendCmpltflag == 1) //发送完成
    {		
        SciaSendEna(TXRXDISABLE); //禁止发送
        CleanUpErrFlag(); //清除usart故障信息
        SciaReadEna(TXRXENABLE);  //启动接收

        gCommSendCmpltflag = 0;

        if(bResSending)
        {
            bResSending = 0;
        }

        if(bContinueSending)
        {
            bContinueSending = 0;
            AuxFunCodeUnion.code.OS_SeriesTxCtrl = 0;
        }

        AuxFunCodeUnion.code.OS_UartStatus &= 2;      //发送完成 BIT0 = 0
        ComSendDelayCnt=0;
    }
}


/**************************************************************
  函数名:         // G_COMM_ModbusSchedual
  输入:           //
  输出:           //
  子函数:         //
  描述:           //Modbus模块调度函数
**************************************************************/
void G_COMM_ModbusSchedual(void)
{
    static Uint32 gFrame3P5TimeCntNew;
    static Uint32 gFrame3P5TimeCntOld;
    static Uint32 gFrame1P5TimeCntNew;
    static Uint32 gFrame1P5TimeCntOld;

    if(gFrame3P5CntStartFlag == c3P5START)
    {
        gFrame3P5TimeCntNew=GetSysTime_1MHzClk();
        gFrame3P5TimeCntOld=gFrame3P5TimeCntNew;
        gFrame3P5CntStartFlag=c3P5END;
    }
    else if(gFrame3P5Arrive==c3P5NOARRIVE)
    {
        gFrame3P5TimeCntNew=GetSysTime_1MHzClk();
        if((Uint16)(gFrame3P5TimeCntNew-gFrame3P5TimeCntOld)>wFrame3P5Time)
        {
            gFrame3P5Arrive=c3P5ARRIVE;
        }
    }

    if(gFrame1P5CntStartFlag == c1P5START)
    {
        gFrame1P5TimeCntNew=GetSysTime_1MHzClk();
        gFrame1P5TimeCntOld=gFrame1P5TimeCntNew;
        gFrame1P5CntStartFlag=c1P5END;
    }
    else if(gFrame1P5Arrive==c1P5NOARRIVE)
    {
        gFrame1P5TimeCntNew=GetSysTime_1MHzClk();
        if((Uint16)(gFrame1P5TimeCntNew-gFrame1P5TimeCntOld)>wFrame1P5Time)
        {
            gFrame1P5Arrive=c1P5ARRIVE;
        }
    }

    if((gFrame3P5Arrive==c3P5ARRIVE)&&(gCommRcvLenth != 0))
    {
//        if(gCommRcvLenth != 0)
        {
            AuxFunCodeUnion.code.OS_UartStatus &= 1;    //接收完成 BIT1 = 0
            DINT;
			gRcvLenthMax = gCommRcvLenth;
            gCommRcvLenth = 0;
            EINT;
			if(!FrameValidate())
			{
				//为有效帧
            	RcvProcessRdy=cRCVPROCESSRDY;
            	SendProcessRdy=cSENDPROCESSRDY;
			}
        }
     }
}


/**************************************************************
  函数名:         // FrameValidate
  输入:           //
  输出:           // 判断结果：0-帧有效；1-帧无效
  子函数:         //
  描述:           //帧有效性判断
**************************************************************/ 
Uint16 FrameValidate(void)
{
	Uint16 FrameLenth = 0; //本机地址

	//判断报文长度是否正确
	if((gaCommRcvBuff[COMMCMD] != 0x10)  &&  (gaCommRcvBuff[COMMCMD] != 0x46))//除10指令外，其余指令的报文长度均为8
	{
		if(gRcvLenthMax != 8)
		{
			return 1;
		}
	} 
	else //10指令报文长度通过字节数计算
	{
		if(gaCommRcvBuff[6] != 0)
		{
			FrameLenth = 9 + gaCommRcvBuff[6];
			if(FrameLenth != gRcvLenthMax)
			{
				return 1;
			}
		}			
	}

	if(gaCommRcvBuff[ADDR] == (Uint8)FunCodeUnion.code.CM_AxisAdress) //一对一
	{
		return 0;
	}
	else if(gaCommRcvBuff[ADDR] == 0x00) //广播帧 
	{
		if(gaCommRcvBuff[COMMCMD] == 0x03) //非法广播帧
		{
			return 1;
		}
		else 
		{
			gCommBrdcastFlag = 1; //广播地址标志
			return 0;
		}
	}

    return 1;
}



/********************************* END OF FILE *********************************/
