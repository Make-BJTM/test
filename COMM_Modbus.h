/*******************************************************************************
//深圳市汇川技术有限公司 版权所有（C）All rights reserved.
// 文件名: COMM_Modbus.h
// 创建人: 韦水平            创建日期：2011年10月 [V.001]
// 描述: Modbus协议头文件  
// 修改历史:
********************************************************************************/ 

#ifndef COMM_MODBUS_H
#define COMM_MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h"

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */
#define CTRCMDLENTH            21   //上位机发送给下位机控制码长度12字节，如GROUP = 0x60,OFFSET=0X00;
#define FUNCCMDLENTH           8    //上位机发给下位机功能码帧长度10字节
#define WRITECANCFGLENTH       14   //位机发送CANLINK配置信息
#define READDATALENTH          125  //上位机一次最多能读125个字节长度的数据
#define WRITEDATALENTH         123  //上位机一次最多能写123个字节长度的数据
#define BRORDADDR              0    //广播地址
#define ADDR                   0    //gCommRcvBuff[ADDR]帧地址域，1~247
#define COMMCMD                1    //gCommRcvBuff[CMD]读写标志域，03读，06写
#define GROUP                  2    //gCommRcvBuff[GROUP]参数地址高有效字节，对应功能码组号
#define OFFSET                 3    //gCommRcvBuff[OFFSET]参数地址低有效字节，对应功能码组偏移量
#define DATALH                 4    //gCommRcvBuff[DATAHH]数据次低有效字节
#define DATALL                 5    //gCommRcvBuff[DATAHH]数据最低有效字节
#define CRCL                   6    //gCommRcvBuff[CRCL] CRC校验低有效字节
#define CRCH                   7    //gCommRcvBuff[CRCH] CRC校验高有效字节
#define c3P5START              0    //3.5字节计时启动
#define c3P5END                1    //3.5字节计时启动完成
#define c1P5START              0    //1.5字节计时启动
#define c1P5END                1    //1.5字节计时启动完成
#define c3P5ARRIVE             0    //3.5字节计时到达
#define c3P5NOARRIVE           1    //3.5字节计时未到达
#define c1P5ARRIVE             0    //1.5字节计时到达
#define c1P5NOARRIVE           1    //1.5字节计时未到达
#define cRCVPROCESSRDY         0    //接收帧可以进行处理
#define cRCVPROCESSNORDY       1    //接收帧未就绪，不可以进行处理
#define cSENDPROCESSRDY        0    //发送帧可以进行处理
#define cSENDPROCESSNORDY      1    //发送帧未就绪，不可以进行处理
#define RXERROR                0
#define TXRXDISABLE            0
#define TXRXENABLE             1

/* Exported_Macros -----------------------------------------------------------*/
/* 带参数的宏定义 */

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 
typedef struct
{
    Uint16 CmdErr:1;            // 1    命令码错误 Err2
	Uint16 CrcChkErr:1;         // 2    CRC校验故障 Err3
	Uint16 AddrOver:1;          // 3    无效地址 Err4
	Uint16 ParaOver:1;          // 4    无效参数 Err5
	Uint16 ServoRun:1;          // 7   电机正在运行
//	Uint16 ZeroLength:1;		// 读取参数长度为0
//	Uint16 DataTypeNotMatch:1;  // 数据类型不符合
	Uint16 DataReadDeny:1;      // 数据读禁止操作
	Uint16 BlockNotExist:1;     // 扇区不存在
	Uint16 DataLengthExceed:1;  // 长度超过有效范围
    Uint16 Rsvd:8;              // 15 保留
}STR_MODBUS_ERROR;  

typedef union
{
    volatile Uint16 All;
    volatile STR_MODBUS_ERROR ErrBit;
}UNI_MODBUS_ERR_REG;

typedef  Uint8 COMM_RCV_MSG[268];            //上位机发送给下位机的帧结构
typedef  Uint8 COMM_SEND_MSG[268];           //下位机反馈上位机的帧结构

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */
extern COMM_RCV_MSG gaCommRcvBuff;           //接收帧缓存
extern COMM_SEND_MSG gaCommSendBuff;         //发送帧缓存
extern UNI_MODBUS_ERR_REG gunCommErrBit;     //通信错误标志结构体 

extern Uint16 gCommRcvLenth;                 //已接收字节数

extern Uint16 wFrame3P5Time;                 //3.5字节对应PWM触发次数
extern Uint16 wFrame1P5Time;                 //1.5字节对应PWM触发次数

extern Uint16 gFrame3P5CntStartFlag;         //3.5字节计时启动标志
extern Uint16 gFrame1P5CntStartFlag;         //1.5字节计时启动标志

extern Uint16 gFrame3P5Arrive;               //3.5字节计时到达标志
extern Uint16 gFrame1P5Arrive;	             //1.5字节计时到达标志

extern Uint16 RcvProcessRdy;                 //接收处理状态
extern Uint16 SendProcessRdy;                //发送处理状态



/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 



#ifdef __cplusplus
}
#endif

#endif /*__COMM_MODBUS_H*/

/********************************* END OF FILE *********************************/

