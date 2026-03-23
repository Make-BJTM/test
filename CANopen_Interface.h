/*************** (C) COPYRIGHT 2013  Inovance Technology Co., Ltd****************
* File Name		: CANopen_InterFace.h
* Author		: Denglei	
* Version		: V0.0.1
* Date			: 
* Description	: 
* Modify		:
********************************************************************************/

#ifndef __CANOPEN_INTERFACE_H
#define __CANOPEN_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "PUB_GlobalPrototypes.h"

#define OBD_INIT            0x00                			// 初始化OBD
#define OBD_LOAD        	0x01							// 参数加载EEPROM 
#define OBD_STORE        	0x02							// 保存当前值至EEPROM
#define OBD_RESTORE      	0x03							// 保存默认值值EEPROM

typedef struct
{
    Uint16	wEmcyErrCode;
    Uint8	bErrRegister;
    Uint32	dwManuErrCode;
} tEmcParam;									   			// 紧急报文格式

extern Uint8 GetNodeState(void);							// 通讯部分状态机

extern Uint16 ComErrState(void);							// 通信错误状态

extern Uint8 CANPollingAppError(tEmcParam *EmcyPara); 		// 应用区错误状态

extern Uint32 PdoCheckObj(Uint16 Index, Uint8 subIndex, Uint32 datasize, Uint32 rw);
															// 检查应用区对象字典是否可映射
//extern Uint32 *pGetFuncodeAddr(Uint16 CiaIndexTemp,Uint8 CiaSubIndexTemp, Uint16 wPDOx);
															// 对象字典数据地址
//extern Uint32 SDO_Read_FunCode (Uint16 index, Uint8 subindex, Uint32 *dataSize, Uint8* pData);
															// 读应用区对象字典

//extern Uint32 SDO_Write_FunCode (Uint16 index, Uint8 subindex, Uint32 dataSize, Uint8* pData);
															// 写应用区对象字典

//extern Uint32 AppDataToEeprom(Uint8 bDirection);			// 将402数据存入EEPROM
//extern Uint32 FuncCodeToEeprom(Uint8 bDirection);			// 将功能码数据存入EEPROM


//extern void WriteIntpltFIFO(Uint16 wPDOx);					// 写环形缓冲

extern Uint16 ObdAccessEntry (Uint8 bDirection);			// 伺服在执行H02-31时调用


#ifdef __cplusplus
}
#endif

#endif
