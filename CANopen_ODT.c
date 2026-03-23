/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "CANopen_OD.h"
#include "FUNC_FunCode.h"
#include "FUNC_AuxFunCode.h"
#include "FUNC_FunCodeDefault.h"
#include "FUNC_GlobalVariable.h"
#include "CANopen_InterFace.h"
#include "FUNC_ErrorCode.h" 
#include "FUNC_OperEeprom.h"
#include "FUNC_WWDG.h" 
#include "ECT_InterFace.h"



/* Private_Constants ---------------------------------------------------------*/
/* 宏定义 常数类*/
#define EEPROM_CHECK_WORD1      (0x010D + DRIVER_TYPE * 1000)           // EEPROM校验字1
#define EEPROM_CHECK_WORD2      (Uint16)(0xFFFF - EEPROM_CHECK_WORD1)   // EEPROM校验字2

#define NULL                         0




//属性相关宏定义
//bit.AccessType   BIT:0-1    00:RW   01:WO    10:RO
#define ACCESS_RW                           0x00
#define ACCESS_WO                           0x01
#define ACCESS_RO                           0x02
#define ACCESS_RSVD                         0x03
//bit.DataType   BIT:2-4     010:有符号8位  011:有符号16位  100:有符号32位
//                           101:无符号8位  110:无符号16位  111:无符号32位 
#define TYPE_INT8                    ((Uint32)2 << 2) //0x02
#define TYPE_INT16                   ((Uint32)3 << 2) //0x03 
#define TYPE_INT32                   ((Uint32)4 << 2) //0x04 
#define TYPE_UINT8                   ((Uint32)5 << 2) //0x05
#define TYPE_UINT16                  ((Uint32)6 << 2) //0x06
#define TYPE_UINT32                  ((Uint32)7 << 2) //0x07
//bit.DataSize   BIT:5-8    0001:1个字节  0010:2个字节  0100:4个字节  1000:8个字节
#define ONE_BYTE                     ((Uint32)1 << 5) //0x01
#define TWO_BYTE                     ((Uint32)2 << 5) //0x02
#define FOUR_BYTE                    ((Uint32)4 << 5) //0x04
#define EIGHT_BYTE                   ((Uint32)8 << 5) //0x08
//bit.PDOMapping BIT:9      0-UNMapping   1-Mapping
#define UNMAPPING                    0x00 
#define MAPPING                      ((Uint32)1 << 9) //0x01
//bit.ODBelong BIT:10-11      00-BOTH   01-ECT_ONLT  10-CANOPEN_ONLY
#define BOTH                         0x00 
#define CANOPEN_ONLY                 ((Uint32)1 << 10) //0x01
#define ECT_ONLY                     ((Uint32)2 << 10) //0x01
//bit.ComSave BIT:12      0-UNCOMSAVE   1-COMSAVE
#define UNCOMSAVE                    0x00 
#define COMSAVE                      ((Uint32)1 << 12) //0x01
//SDO abort codes
//#define TOGGLE_BIT_NOT_ALTERNATED                       0x05030000 //Toggle bit not alternated.
//#define SDO_PROTOCOL_TIMED_OUT                          0x05040000 //SDO protocol timed out.
//#define COMMAND_SPECIFIER_ERROR                         0x05040001 //Client/server command specifier not valid or unknown.
//#define INVALID_BLOCK_SIZE                              0x05040002 //Invalid block size (block mode only).
//#define INVALID_SEQUENCE_NUMBER                         0x05040003 //Invalid sequence number (block mode only).
//#define CRC_ERROR                                       0x05040004 //CRC error (block mode only).
//#define OUT_OF_MEMORY                                   0x05040005 //Out of memory.
//#define OD_UNSUPPORTED_ACCESS                           0x06010000 //Unsupported access to an object.
//#define OD_WRITE_ONLY                                   0x06010001 //Attempt to read a write only object.
//#define OD_READ_ONLY                                    0x06010002 //Attempt to write a read only object.
//#define OD_NOT_EXIST                                    0x06020000 //Object does not exist in the object dictionary.
//#define OD_UNMAPPABLE                                   0x06040041 //Object cannot be mapped to the PDO.
//#define OD_NUMBER_AND_LENGTH_EXCEEDED                   0x06040042 //The number and length of the objects to be mapped would exceed PDO length.
//#define PARAMETER_INCOMPATIBILITY                       0x06040043 //General parameter incompatibility reason.
//#define INTERNAL_INCOMPATIBILITY_IN_THE_DEVICE          0x06040047 //General internal incompatibility in the device.
//#define ACCESS_FAILED_BY_HARDWARE_ERROR                 0x06060000 //Access failed due to an hardware error.
//#define UNMATCHED_LENGTH                                0x06070010 //Data type does not match, length of service parameter does not match
//#define TOO_HIGH_LENGTH                                 0x06070012 //Data type does not match, length of service parameter too high
//#define TOO_LOW_LENGTH                                  0x06070013 //Data type does not match, length of service parameter too low
//#define SUBINDEX_NOT_EXIST                              0x06090011 //Sub-index does not exist.
//#define VALUE_RANGE_EXCEEDED                            0x06090030 //Value range of parameter exceeded (only for write access).
//#define TOO_HIGH_VALUE                                  0x06090031 //Value of parameter written too high.
//#define TOO_LOW_VALUE                                   0x06090032 //Value of parameter written too low.
//#define MAX_VALUE_LESS_THAN_MIN_VALUE                   0x06090036 //Maximum value is less than minimum value.
//#define GENERAL_ERROR                                   0x08000000 //general error
//#define DATA_PROCESSED_ERROR                            0x08000020 //Data cannot be transferred or stored to the application.
//#define DATA_PROCESSED_ERROR_BY_LOCAL_CONTROL           0x08000021 //Data cannot be transferred or stored to the application because of local control.
//#define DATA_PROCESSED_ERROR_BY_PRESENT_DEVICE_STATE    0x08000022 //Data cannot be transferred or stored to the application because of the present device state.
//#define OD_LOAD_ERROR                                   0x08000023 //Object dictionary dynamic generation fails or no object dictionary ispresent (e.g. object dictionary is generated from file and generation fails because of an file error).



#define     COMM_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE	1	//0x08000022  由于当前设备状态导致数据不能传送或者保存到应用（密码错误）
#define     COMM_OBJECT_CANT_BE_PDOMAPPED						2	//0x06040041 对象不能够映射到PDO
#define     COMM_HARDWARE_ERROR									3	//0x06060000 硬件错误导致对象访问失败（Modbus的从站设备故障）
#define     COMM_OBJECT_NOT_EXISTING							4	//0x06020000 对象字典中的对象不存在（无效地址）
#define     COMM_VALUE_EXCEEDED									5	//0x06090030 超出参数的值范围（无效参数）
#define     COMM_DATA_CANNOT_BE_READ_OR_STORED					6	//0x08000020 数据不能传送或保存到应用（参数更改无效）
#define     COMM_DATA_CANNOT_BE_READ_OR_STORED_BECAUSE_OF_LOCAL_CONTROL	  7	 //0x08000021 由于本地控制导致数据不能传送保存到应用（系统锁定）
#define     COMM_GENERAL_ERROR									8	//0x08000000 一般性错误
#define     COMM_UNSUPPORTED_ACCESS								9	//0x06010000 对象不支持访问
#define     COMM_WRITE_ONLY_ENTRY								10	//0x06010001 试图读只写对象
#define     COMM_READ_ONLY_ENTRY								11	//0x06010002 试图写只读对象
#define     COMM_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0				12	//0x06010003 写对象子索引时，必须先清索引0
#define     COMM_PARAM_LENGTH_ERROR								13	//0x06070010 数据类型不匹配，服务参数长度不匹配
#define     COMM_PARAM_LENGTH_TOO_LONG							14	//0x06070012 数据类型不匹配，服务参数长度太大
#define     COMM_PARAM_LENGTH_TOO_SHORT							15	//0x06070013 数据类型不匹配，服务参数长度太短
#define     COMM_SUBINDEX_NOT_EXISTING							16	//0x06090011 子索引不存在
#define     COMM_VALUE_TOO_GREAT								17	//0x06090031 写入参数数值太大
#define     COMM_VALUE_TOO_SMALL								18	//0x06090032 写入参数值太小
#define     COMM_MAX_VALUE_IS_LESS_THAN_MIN_VALUE				19	//0x06090036 最大值小于最小值
#define     COMM_NO_OBJECT_DICTIONARY_IS_PRESENT				20	//0x08000023 对象字典动态产生错误或对象字典不存在





/* Private_Macros ------------------------------------------------------------*/
/* 宏定义 函数类 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */

STR_OBJECTDICTIONARYSTANDARD ObjectDictionaryStandard;
STR_SERVORERRORCODE  STR_SerErrCode;
STR_ODATTRIB  STR_ODAttrib;
UNION_OMRONINSIDEDI       union_OmronInside;


//获取对象字典的首地址
Uint32 * const ODStandardTable[193] = 
{
    (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode,//603Fh
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.ControlWord.all,//6040h
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.StatusWord.all,//6041h
    (Uint32 *)&PUB_Null_PointVar32Init,//6042h
    (Uint32 *)&PUB_Null_PointVar32Init,//6043h
    (Uint32 *)&PUB_Null_PointVar32Init,//6044h
    (Uint32 *)&PUB_Null_PointVar32Init,//6045h
    (Uint32 *)&PUB_Null_PointVar32Init,//6046h
    (Uint32 *)&PUB_Null_PointVar32Init,//6047h
    (Uint32 *)&PUB_Null_PointVar32Init,//6048h
    (Uint32 *)&PUB_Null_PointVar32Init,//6049h
    (Uint32 *)&PUB_Null_PointVar32Init,//604Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//604Bh
    (Uint32 *)&PUB_Null_PointVar32Init,//604Ch
    (Uint32 *)&PUB_Null_PointVar32Init,//604Dh
    (Uint32 *)&PUB_Null_PointVar32Init,//604Eh
    (Uint32 *)&PUB_Null_PointVar32Init,//604Fh
    (Uint32 *)&PUB_Null_PointVar32Init,//6050h
    (Uint32 *)&PUB_Null_PointVar32Init,//6051h
    (Uint32 *)&PUB_Null_PointVar32Init,//6052h
    (Uint32 *)&PUB_Null_PointVar32Init,//6053h
    (Uint32 *)&PUB_Null_PointVar32Init,//6054h
    (Uint32 *)&PUB_Null_PointVar32Init,//6055h
    (Uint32 *)&PUB_Null_PointVar32Init,//6056h
    (Uint32 *)&PUB_Null_PointVar32Init,//6057h
    (Uint32 *)&PUB_Null_PointVar32Init,//6058h
    (Uint32 *)&PUB_Null_PointVar32Init,//6059h
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.QuickStopOptionCode,//605Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//605Bh
    (Uint32 *)&PUB_Null_PointVar32Init,//605Ch
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.HaltOptionCode,//605Dh
    (Uint32 *)&PUB_Null_PointVar32Init,//605Eh
    (Uint32 *)&PUB_Null_PointVar32Init,//605Fh
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.ModesOfOperation,//6060h
    (Uint32 *)&ObjectDictionaryStandard.DeviceControl.ModesOfOperationDisplay,//6061h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionDemandValue,//6062h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionActualValueInc,//6063h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionActualValue,//6064h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorWindow,//6065h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorTimeOut,//6066h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionWindow,//6067h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionWindowTime,//6068h
    (Uint32 *)&PUB_Null_PointVar32Init,//6069h
    (Uint32 *)&PUB_Null_PointVar32Init,//606Ah
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityDemandValue,//606Bh
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityActualValue,//606Ch
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityWindow,//606Dh
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityWindowTime,//606Eh
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityThreshold,//606Fh
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode1.VelocityThresholdTime,//6070h
    (Uint32 *)&ObjectDictionaryStandard.ProToqMode1.TargetToq,//6071h
    (Uint32 *)&ObjectDictionaryStandard.ProToqMode1.MaxToq,//6072h
    (Uint32 *)&PUB_Null_PointVar32Init,//6073h
    (Uint32 *)&ObjectDictionaryStandard.ProToqMode1.ToqDemandValue,//6074h
    (Uint32 *)&PUB_Null_PointVar32Init,//6075h
    (Uint32 *)&PUB_Null_PointVar32Init,//6076h
    (Uint32 *)&ObjectDictionaryStandard.ProToqMode1.ToqActualValue,//6077h
    (Uint32 *)&PUB_Null_PointVar32Init,//6078h
    (Uint32 *)&PUB_Null_PointVar32Init,//6079h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.TargetPosition,//607Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//607Bh
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.HomeOffset,//607Ch
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.SoftwarePositionLimit.NumberOfEntries,//607Dh
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.Polarity,//607Eh
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity,//607Fh
    (Uint32 *)&PUB_Null_PointVar32Init,//6080h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileVelocity,//6081h
    (Uint32 *)&PUB_Null_PointVar32Init,//6082h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileAcceleration,//6083h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileDeceleration,//6084h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration,//6085h
    (Uint32 *)&ObjectDictionaryStandard.ProPosMode.MotionProfileType,//6086h
    (Uint32 *)&ObjectDictionaryStandard.ProToqMode2.ToqSlope,//6087h
    (Uint32 *)&PUB_Null_PointVar32Init,//6088h
    (Uint32 *)&PUB_Null_PointVar32Init,//6089h
    (Uint32 *)&PUB_Null_PointVar32Init,//608Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//608Bh
    (Uint32 *)&PUB_Null_PointVar32Init,//608Ch
    (Uint32 *)&PUB_Null_PointVar32Init,//608Dh
    (Uint32 *)&PUB_Null_PointVar32Init,//608Eh
    (Uint32 *)&PUB_Null_PointVar32Init,//608Fh
    (Uint32 *)&PUB_Null_PointVar32Init,//6090h
    (Uint32 *)&ObjectDictionaryStandard.FactorGroup.GearRatio.NumberOfEntries,//6091h
    (Uint32 *)&PUB_Null_PointVar32Init,//6092h
    (Uint32 *)&ObjectDictionaryStandard.FactorGroup.PositionFactor.NumberOfEntries,//6093h
    (Uint32 *)&ObjectDictionaryStandard.FactorGroup.VelocityEncoderFactor.NumberOfEntries,//6094h
    (Uint32 *)&ObjectDictionaryStandard.FactorGroup.VelocityFactor1.NumberOfEntries,//6095h
    (Uint32 *)&PUB_Null_PointVar32Init,//6096h
    (Uint32 *)&ObjectDictionaryStandard.FactorGroup.AccelerationFactor.NumberOfEntries,//6097h
    (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingMethod,//6098h
    (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingSpeeds.NumberOfEntries,//6099h
    (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingAcceleration,//609Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//609Ah
    (Uint32 *)&PUB_Null_PointVar32Init,//609Bh
    (Uint32 *)&PUB_Null_PointVar32Init,//609Ch
    (Uint32 *)&PUB_Null_PointVar32Init,//609Dh
    (Uint32 *)&PUB_Null_PointVar32Init,//609Eh
    (Uint32 *)&PUB_Null_PointVar32Init,//609Fh
    (Uint32 *)&PUB_Null_PointVar32Init,//60A1h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A2h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A3h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A4h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A5h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A6h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A7h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A8h
    (Uint32 *)&PUB_Null_PointVar32Init,//60A9h
    (Uint32 *)&PUB_Null_PointVar32Init,//60AAh
    (Uint32 *)&PUB_Null_PointVar32Init,//60ABh
    (Uint32 *)&PUB_Null_PointVar32Init,//60ACh
    (Uint32 *)&PUB_Null_PointVar32Init,//60ADh
    (Uint32 *)&PUB_Null_PointVar32Init,//60AEh
    (Uint32 *)&PUB_Null_PointVar32Init,//60AFh
    (Uint32 *)&ObjectDictionaryStandard.CstOffset.PosOff,//60B0h
    (Uint32 *)&ObjectDictionaryStandard.CstOffset.VelOff,//60B1h
    (Uint32 *)&ObjectDictionaryStandard.CstOffset.ToqOff,//60B2h
    (Uint32 *)&PUB_Null_PointVar32Init,//60B3h
    (Uint32 *)&PUB_Null_PointVar32Init,//60B4h
    (Uint32 *)&PUB_Null_PointVar32Init,//60B5h
    (Uint32 *)&PUB_Null_PointVar32Init,//60B6h
    (Uint32 *)&PUB_Null_PointVar32Init,//60B7h
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Func,//60B8h
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Status,//60B9h
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Pb1PosValatPE,//60BAh
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Pb1PosValatNE,//60BBh
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Pb2PosValatPE,//60BCh
    (Uint32 *)&ObjectDictionaryStandard.TouchProbe.Pb2PosValatNE,//60BDh
    (Uint32 *)&PUB_Null_PointVar32Init,//60BEh
    (Uint32 *)&PUB_Null_PointVar32Init,//60BFh
    (Uint32 *)&PUB_Null_PointVar32Init,//60C0h
    (Uint32 *)&ObjectDictionaryStandard.InterpltPosMode1.InterpltDataRecord.NumberOfEntrys,//60C1h 
    (Uint32 *)&ObjectDictionaryStandard.InterpltPosMode1.InterpltPeriod.NumberOfEntrys,//60C2h
    (Uint32 *)&PUB_Null_PointVar32Init,//60C3h
    (Uint32 *)&PUB_Null_PointVar32Init,//60C4h
    (Uint32 *)&ObjectDictionaryStandard.MaxAcc.MaxAcceleration,//60C5h
    (Uint32 *)&ObjectDictionaryStandard.MaxAcc.MaxDeceleration,//60C6h
    (Uint32 *)&PUB_Null_PointVar32Init,//60C7h
    (Uint32 *)&PUB_Null_PointVar32Init,//60C8h
    (Uint32 *)&PUB_Null_PointVar32Init,//60C9h
    (Uint32 *)&PUB_Null_PointVar32Init,//60CAh
    (Uint32 *)&PUB_Null_PointVar32Init,//60CBh
    (Uint32 *)&PUB_Null_PointVar32Init,//60CCh
    (Uint32 *)&PUB_Null_PointVar32Init,//60CDh
    (Uint32 *)&PUB_Null_PointVar32Init,//60CEh
    (Uint32 *)&PUB_Null_PointVar32Init,//60CFh
    (Uint32 *)&PUB_Null_PointVar32Init,//60D0h
    (Uint32 *)&PUB_Null_PointVar32Init,//60D1h
    (Uint32 *)&PUB_Null_PointVar32Init,//60D2h
    (Uint32 *)&PUB_Null_PointVar32Init,//60D3h
    (Uint32 *)&PUB_Null_PointVar32Init,//60D4h
 	(Uint32 *)&ObjectDictionaryStandard.TouchProbe2.TP1PosEdgeCnt,//60D5h
	(Uint32 *)&ObjectDictionaryStandard.TouchProbe2.TP1NegEdgeCnt,//60D6h
	(Uint32 *)&ObjectDictionaryStandard.TouchProbe2.TP2PosEdgeCnt,//60D7h
	(Uint32 *)&ObjectDictionaryStandard.TouchProbe2.TP2NegEdgeCnt,//60D8h   
    (Uint32 *)&PUB_Null_PointVar32Init,//60D9h
    (Uint32 *)&PUB_Null_PointVar32Init,//60DAh
    (Uint32 *)&PUB_Null_PointVar32Init,//60DBh
    (Uint32 *)&PUB_Null_PointVar32Init,//60DCh
    (Uint32 *)&PUB_Null_PointVar32Init,//60DDh
    (Uint32 *)&PUB_Null_PointVar32Init,//60DEh
    (Uint32 *)&PUB_Null_PointVar32Init,//60DFh
    (Uint32 *)&ObjectDictionaryStandard.MaxToqLmt.MaxPosToq,//60E0h
    (Uint32 *)&ObjectDictionaryStandard.MaxToqLmt.MaxNegToq,//60E1h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E2h
    (Uint32 *)&ObjectDictionaryStandard.HomandPosCalMethod.SurHoming.NumberOfEntrys,//60E3h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E4h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E5h
    (Uint32 *)&ObjectDictionaryStandard.HomandPosCalMethod.ActualPosCalMethod,//60E6h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E7h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E8h
    (Uint32 *)&PUB_Null_PointVar32Init,//60E9h
    (Uint32 *)&PUB_Null_PointVar32Init,//60EAh
    (Uint32 *)&PUB_Null_PointVar32Init,//60EBh
    (Uint32 *)&PUB_Null_PointVar32Init,//60ECh
    (Uint32 *)&PUB_Null_PointVar32Init,//60EDh
    (Uint32 *)&PUB_Null_PointVar32Init,//60EEh
    (Uint32 *)&PUB_Null_PointVar32Init,//60EFh
    (Uint32 *)&PUB_Null_PointVar32Init,//60F0h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F1h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F2h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F3h
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc2.FollowingErrorActualValue,//60F4h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F5h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F6h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F7h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F8h
    (Uint32 *)&PUB_Null_PointVar32Init,//60F9h
    (Uint32 *)&PUB_Null_PointVar32Init,//60FAh
    (Uint32 *)&PUB_Null_PointVar32Init,//60FBh
    (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc2.PositionDemandValueInc,//60FCh
    (Uint32 *)&ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.all,//60FDh
    (Uint32 *)&ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.NumberOfEntries,//60FEh
    (Uint32 *)&ObjectDictionaryStandard.ProVelMode2.TargetVelocity,//60FFh

};
//获取对象字典各索引属性

const  STR_OBJECCTDICTIONARY_DEFAULT     ObjectDictionaryDefault[119]=
{
/*  10
    Uint32 ObjectDictionaryStandard.ComEntryErrCode.ErrorCode,              //603Fh
    Uint32 ObjectDictionaryStandard.DeviceControl.ControlWord.all,          //6040h
    Uint32 ObjectDictionaryStandard.DeviceControl.StatusWord.all,           //6041h
    Uint32 ObjectDictionaryStandard.DeviceControl.QuickStopOptionCode,      //605Ah
    Uint32 ObjectDictionaryStandard.DeviceControl.HaltOptionCode,           //605Dh
    Uint32 ObjectDictionaryStandard.DeviceControl.ModesOfOperation,         //6060h
    Uint32 ObjectDictionaryStandard.DeviceControl.ModesOfOperationDisplay,  //6061h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.PositionDemandValue,       //6062h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.PositionActualValueInc,    //6063h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.PositionActualValue,       //6064h
*/                                                                      
/*默认属性：AccessType| DataType| DataSize | PDOMapping*/
/*注释      子索引个数         默认值       下限                上限                属性*/
/*603Fh*/       0,          0,              0,                  (Uint32)0xFFFF,     ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH,
/*6040h*/       0,          0,              0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH,
/*6041h*/       0,          0,              0,                  (Uint32)0xFFFF,     ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH,
/*605Ah*/       0,          2,              0,                  (Uint32)0x0007,     ACCESS_RW|TYPE_INT16|TWO_BYTE|UNMAPPING|BOTH|COMSAVE,       
/*605Dh*/       0,          1,              0x0001,             (Uint32)0x0003,     ACCESS_RW|TYPE_INT16|TWO_BYTE|UNMAPPING|ECT_ONLY|COMSAVE,       
#if ECT_ENABLE_SWITCH
/*6060h*/       0,          0,              0x00,               (Uint32)0x0A,       ACCESS_RW|TYPE_INT8|ONE_BYTE|MAPPING|BOTH|COMSAVE,
/*6061h*/       0,          0,              0x00,               (Uint32)0x0A,       ACCESS_RO|TYPE_INT8|ONE_BYTE|MAPPING|BOTH, 
#elif CAN_ENABLE_SWITCH
/*6060h*/       0,          0,              0x00,               (Uint32)0x07,       ACCESS_RW|TYPE_INT8|ONE_BYTE|MAPPING|BOTH|COMSAVE,
/*6061h*/       0,          0,              0x00,               (Uint32)0x07,       ACCESS_RO|TYPE_INT8|ONE_BYTE|MAPPING|BOTH, 
#endif
/*6062h*/       0,          0,              (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,
/*6063h*/       0,          0,              (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,
/*6064h*/       0,          0,              (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,

/*10
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorWindow,      //6065h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.PositionWindow,            //6067h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc1.PositionWindowTime,        //6068h
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityDemandValue,        //606Bh
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityActualValue,        //606Ch
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityWindow,             //606Dh
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityWindowTime,         //606Eh
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityThreshold,          //606Fh
    Uint32 ObjectDictionaryStandard.ProVelMode1.VelocityThresholdTime,      //6070h
    Uint32 ObjectDictionaryStandard.ProToqMode1.TargetToq,                  //6071h
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*6065h*/       0,    (Uint32)0x00300000,    (Uint32)0x00000000, (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6066h*/       0,    0,   	                 0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,
/*6067h*/       0,    (Uint32)0x000002DE,    0,                  (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6068h*/       0,    0,                  0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,
/*606Bh*/       0,    0,                     (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY,
/*606Ch*/       0,    0,                     (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,
/*606Dh*/       0,    0x000A,                0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,
/*606Eh*/       0,    0,                     0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,
/*606Fh*/       0,    0x000A,                0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6070h*/       0,    0,                     0,                  (Uint32)0xFFFF,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6071h*/       0,    0,                    (Uint32)0xEC78,      (Uint32)0x1388,     ACCESS_RW|TYPE_INT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,

/*10
    Uint32 ObjectDictionaryStandard.ProToqMode1.MaxToq,                     //6072h
    Uint32 ObjectDictionaryStandard.ProToqMode1.ToqDemandValue,             //6074h
    Uint32 ObjectDictionaryStandard.ProToqMode1.ToqActualValue,             //6077h
    Uint32 ObjectDictionaryStandard.ProPosMode.TargetPosition,              //607Ah
    Uint32 ObjectDictionaryStandard.ProPosMode.HomeOffset,                  //607Ch
    Uint32 ObjectDictionaryStandard.ProPosMode.SoftwarePositionLimit.NumberOfEntries,//607Dh
    Uint32 ObjectDictionaryStandard.ProPosMode.Polarity,                             //607Eh
    Uint32 ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity,                   //607Fh

*/
/*默认属性：AccessType| DataType| DataSize */
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*6072h*/       0,    (Uint32)0x1388,        0,                  (Uint32)0x1388,     ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|COMSAVE,
/*6074h*/       0,    0,                     (Uint32)0xEC78,     (Uint32)0x1388,     ACCESS_RO|TYPE_INT16|TWO_BYTE|MAPPING|ECT_ONLY,
/*6077h*/       0,    0,                     (Uint32)0xEC78,     (Uint32)0x1388,     ACCESS_RO|TYPE_INT16|TWO_BYTE|MAPPING|BOTH,
/*607Ah*/       0,    0,                     (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*607Ch*/       0,    0,                     (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*607Dh-0h*/    2,    2,                     0,                  (Uint32)2,          ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|BOTH,
/*607Dh-1h*/    2,    (Uint32)0x80000000,    (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*607Dh-2h*/    2,    (Uint32)0x7FFFFFFF,    (Uint32)0x80000000, (Uint32)0x7FFFFFFF, ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*607Eh*/       0,    0,                     0,                  (Uint32)0xFF,       ACCESS_RW|TYPE_UINT8|ONE_BYTE|MAPPING|BOTH|COMSAVE,
#if ECT_ENABLE_SWITCH
/*607Fh*/       0,    (Uint32)0x06400000,    0,                  (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#elif CAN_ENABLE_SWITCH
/*607Fh*/       0,    (Uint32)0x00001770,    0,                  (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#endif
/*10
    Uint32 ObjectDictionaryStandard.ProPosMode.ProfileVelocity,                      //6081h
    Uint32 ObjectDictionaryStandard.ProPosMode.ProfileAcceleration,                  //6083h
    Uint32 ObjectDictionaryStandard.ProPosMode.ProfileDeceleration,                  //6084h
    Uint32 ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration,                //6085h
    Uint32 ObjectDictionaryStandard.ProPosMode.MotionProfileType,                    //6086h
    Uint32 ObjectDictionaryStandard.ProToqMode2.ToqSlope,                            //6087h
    Uint32 ObjectDictionaryStandard.FactorGroup.GearRatio.NumberOfEntries,             //6091h

*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
#if ECT_ENABLE_SWITCH
/*6081h*/       0,    (Uint32)0x001AAAAB,    (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6083h*/       0,    (Uint32)0x682AAAAB,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6084h*/       0,    (Uint32)0x682AAAAB,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6085h*/       0,    (Uint32)0x682AAAAB,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#elif CAN_ENABLE_SWITCH
/*6081h*/       0,    (Uint32)0x00000064,    (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6083h*/       0,    (Uint32)0x00000064,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6084h*/       0,    (Uint32)0x00000064,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6085h*/       0,    (Uint32)0x00000064,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#endif
/*6086h*/       0,     0,                    (Uint32)0x8000,     (Uint32)0x7FFF,      ACCESS_RW|TYPE_INT16|TWO_BYTE|MAPPING|BOTH|COMSAVE,
/*6087h*/       0,    (Uint32)0xFFFFFFFF,    0,                  (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6091h-0h*/    2,     2,                    0,                  (Uint32)2,           ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|ECT_ONLY,
/*6091h-1h*/    2,     1,                    0x00000001,         (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|ECT_ONLY|COMSAVE,
/*6091h-2h*/    2,     1,                    0x00000001,         (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|ECT_ONLY|COMSAVE,
/*6093h-0h*/    2,    2,              0,                  (Uint32)2,          ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,

/*  10  
    Uint32 ObjectDictionaryStandard.FactorGroup.PositionFactor.NumberOfEntries,             //6093h
    Uint32 ObjectDictionaryStandard.FactorGroup.VelocityEncoderFactor.NumberOfEntries,      //6094h
    Uint32 ObjectDictionaryStandard.FactorGroup.VelocityFactor1.NumberOfEntries,            //6095h
    Uint32 ObjectDictionaryStandard.FactorGroup.AccelerationFactor.NumberOfEntries,         //6097h
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*6093h-1h*/    2,    1,              0,                  (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6093h-2h*/    2,    1,              0x00000001,         (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6094h-0h*/    2,    2,              0,                  (Uint32)2,          ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,
/*6094h-1h*/    2,    (Uint32)0x00100000, (Uint32)0x00000000, (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6094h-2h*/    2,    (Uint32)0x0000003C, (Uint32)0x00000001, (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6095h-0h*/    2,    2,              0,                  (Uint32)2,          ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,
/*6095h-1h*/    2,    1,              0x00000000,         (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6095h-2h*/    2,    1,              0x00000001,         (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6097h-0h*/    2,    2,              0,                  (Uint32)2,          ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,
/*6097h-1h*/    2,      (Uint32)0x3E800000, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,

/* 10
    Uint32 ObjectDictionaryStandard.HomingMode.HomingMethod,                            //6098h
    Uint32 ObjectDictionaryStandard.HomingMode.HomingSpeeds.NumberOfEntries,            //6099h
    Uint32 ObjectDictionaryStandard.HomingMode.HomingAcceleration,                      //609Ah
    Uint32 ObjectDictionaryStandard.CstOffset.PosOff,                                   //60B0h
    Uint32 ObjectDictionaryStandard.CstOffset.VelOff,                                   //60B1h
    Uint32 ObjectDictionaryStandard.CstOffset.ToqOff,                                   //60B2h
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*6097h-2h*/    2,      (Uint32)0x0000003C, (Uint32)0x00000001, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*6098h*/       0,      1,                  0,                  (Uint32)0x23,        ACCESS_RW|TYPE_INT8|ONE_BYTE|MAPPING|BOTH|COMSAVE,
/*6099h-0h*/    2,      2,                  0,                  (Uint32)2,           ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|BOTH,
#if ECT_ENABLE_SWITCH
/*6099h-1h*/    2,      (Uint32)0x001AAAAB, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6099h-2h*/    2,      (Uint32)0x0002AAAB, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*609Ah*/       0,      (Uint32)0x682AAAAB, 0x00000000,         (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#elif CAN_ENABLE_SWITCH
/*6099h-1h*/    2,      (Uint32)0x00000064, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6099h-2h*/    2,      (Uint32)0x0000000A, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*609Ah*/       0,      (Uint32)0x00000064, 0x00000000,         (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
#endif
/*60B0h*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60B1h*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60B2h*/       0,      0,                  (Uint32)0xEC78,     (Uint32)0x1388,      ACCESS_RW|TYPE_INT16|TWO_BYTE|MAPPING|ECT_ONLY,
/*60B8h*/       0,      0,                  0,                  (Uint32)0xFFFF,      ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY,

/* 10
    Uint32 ObjectDictionaryStandard.TouchProbe.Func,                                    //60B8h
    Uint32 ObjectDictionaryStandard.TouchProbe.Status,                                  //60B9h
    Uint32 ObjectDictionaryStandard.TouchProbe.Pb1PosValatPE,                           //60BAh
    Uint32 ObjectDictionaryStandard.TouchProbe.Pb1PosValatNE,                           //60BBh
    Uint32 ObjectDictionaryStandard.TouchProbe.Pb2PosValatPE,                           //60BCh
    Uint32 ObjectDictionaryStandard.TouchProbe.Pb2PosValatNE,                           //60BDh
    Uint32 ObjectDictionaryStandard.InterpltPosMode1.InterpltDataRecord.NumberOfEntrys, //60C1h 
    Uint32 ObjectDictionaryStandard.InterpltPosMode1.InterpltPeriod.NumberOfEntrys,     //60C2h
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60B9h*/       0,      0,                  0,                  (Uint32)0xFFFF,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY,
/*60BAh*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60BBh*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60BCh*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60BDh*/       0,      0,                  0x80000000,         (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|ECT_ONLY,
/*60C1h-0h*/	1,		1,				    0,			         1,		             ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,
/*60C1h-1h*/	1,		0,				    0x80000000, 		(Uint32)0x7FFFFFFF,  ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY,
/*60C2h-0h*/	2,		2,				    0, 		            (Uint32)2, 		     ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|CANOPEN_ONLY,
/*60C2h-1h*/	2,		1,				    1,                  (Uint32)0x14,        ACCESS_RW|TYPE_UINT8|ONE_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*60C2h-2h*/	2,		(Uint32)0xFD,		(Uint32)0xFD, 		(Uint32)0xFD, 		 ACCESS_RW|TYPE_INT8|ONE_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,

/*34
    Uint32 ObjectDictionaryStandard.MaxAcc.MaxAcceleration,                             //60C5h
    Uint32 ObjectDictionaryStandard.MaxAcc.MaxDeceleration,                             //60C6h
 	Uint32 ObjectDictionaryStandard.OD.TouchProbe2.TP1PosEdgeCnt								,//60D5h
	Uint32 ObjectDictionaryStandard.OD.TouchProbe2.TP1NegEdgeCnt								,//60D6h
	Uint32 ObjectDictionaryStandard.OD.TouchProbe2.TP2PosEdgeCnt								,//60D7h
	Uint32 ObjectDictionaryStandard.OD.TouchProbe2.TP2NegEdgeCnt							    ,//60D8h   
    Uint32 ObjectDictionaryStandard.MaxToqLmt.MaxPosToq,                                //60E0h
    Uint32 ObjectDictionaryStandard.MaxToqLmt.MaxNegToq,                                //60E1h
    Uint32 ObjectDictionaryStandard.HomandPosCalMethod.SurHoming.NumberOfEntrys,        //60E3h
    Uint32 ObjectDictionaryStandard.HomandPosCalMethod.ActualPosCalMethod,              //60E6h
    Uint32 ObjectDictionaryStandard.PosCtrlFunc2.FollowingErrorActualValue,             //60F4h
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60C5h*/       0,      (Uint32)0x000003E8, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*60C6h*/       0,      (Uint32)0x000003E8, (Uint32)0x00000000, (Uint32)0xFFFFFFFF,  ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|CANOPEN_ONLY|COMSAVE,
/*60D5h*/       0,      (Uint32)0,          0,                  (Uint32)0xFFFF,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|UNCOMSAVE,
/*60D6h*/       0,      (Uint32)0,          0,                  (Uint32)0xFFFF,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|UNCOMSAVE,
/*60D7h*/       0,      (Uint32)0,          0,                  (Uint32)0xFFFF,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|UNCOMSAVE,
/*60D8h*/       0,      (Uint32)0,          0,                  (Uint32)0xFFFF,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|UNCOMSAVE,
/*60E0h*/       0,      (Uint32)0x1388,     0,                  (Uint32)0x1388,      ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|COMSAVE,
/*60E1h*/       0,      (Uint32)0x1388,     0,                  (Uint32)0x1388,      ACCESS_RW|TYPE_UINT16|TWO_BYTE|MAPPING|ECT_ONLY|COMSAVE,
/*60E3h-00h*/    31,    31,                 0,                  (Uint32)0x1F,        ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-01h*/    31,    (Uint32)0x0301,     0,                  (Uint32)0x0301,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-02h*/    31,    (Uint32)0x0302,     0,                  (Uint32)0x0302,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-03h*/    31,    (Uint32)0x0303,     0,                  (Uint32)0x0303,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-04h*/    31,    (Uint32)0x0304,     0,                  (Uint32)0x0304,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-05h*/    31,    (Uint32)0x0305,     0,                  (Uint32)0x0305,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,

/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60E3h-06h*/    31,    (Uint32)0x0306,     0,                  (Uint32)0x0306,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-07h*/    31,    (Uint32)0x0307,     0,                  (Uint32)0x0307,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-08h*/    31,    (Uint32)0x0308,     0,                  (Uint32)0x0308,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-09h*/    31,    (Uint32)0x0309,     0,                  (Uint32)0x0309,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Ah*/    31,    (Uint32)0x030A,     0,                  (Uint32)0x030A,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Bh*/    31,    (Uint32)0x030B,     0,                  (Uint32)0x030B,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Ch*/    31,    (Uint32)0x030C,     0,                  (Uint32)0x030C,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Dh*/    31,    (Uint32)0x030D,     0,                  (Uint32)0x030D,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Eh*/    31,    (Uint32)0x030E,     0,                  (Uint32)0x030E,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-0Fh*/    31,    (Uint32)0x0311,     0,                  (Uint32)0x0311,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,

/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60E3h-10h*/    31,    (Uint32)0x0312,     0,                  (Uint32)0x0312,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-11h*/    31,    (Uint32)0x0313,     0,                  (Uint32)0x0313,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-12h*/    31,    (Uint32)0x0314,     0,                  (Uint32)0x0314,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-13h*/    31,    (Uint32)0x0315,     0,                  (Uint32)0x0315,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-14h*/    31,    (Uint32)0x0316,     0,                  (Uint32)0x0316,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-15h*/    31,    (Uint32)0x0317,     0,                  (Uint32)0x0317,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-16h*/    31,    (Uint32)0x0318,     0,                  (Uint32)0x0318,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-17h*/    31,    (Uint32)0x0319,     0,                  (Uint32)0x0319,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-18h*/    31,    (Uint32)0x031A,     0,                  (Uint32)0x031A,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-19h*/    31,    (Uint32)0x031B,     0,                  (Uint32)0x031B,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,

/*10
    Uint32 ObjectDictionaryStandard.PosCtrlFunc2.PositionDemandValue,                    //60FCh
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60E3h-1Ah*/    31,    (Uint32)0x031C,     0,                  (Uint32)0x031C,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-1Bh*/    31,    (Uint32)0x031D,     0,                  (Uint32)0x031D,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-1Ch*/    31,    (Uint32)0x031E,     0,                  (Uint32)0x031E,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-1Dh*/    31,    (Uint32)0x0321,     0,                  (Uint32)0x0321,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-1Eh*/    31,    (Uint32)0x0322,     0,                  (Uint32)0x0322,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E3h-1Fh*/    31,    (Uint32)0x0323,     0,                  (Uint32)0x0323,      ACCESS_RO|TYPE_UINT16|TWO_BYTE|UNMAPPING|ECT_ONLY,
/*60E6h*/        0,     (Uint32)0,          0,                  (Uint32)0x01,        ACCESS_RW|TYPE_UINT8|ONE_BYTE|UNMAPPING|ECT_ONLY|COMSAVE,
/*60F4h*/        0,     0,                 (Uint32)0x80000000,  (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,
/*60FCh*/        0,     0,                 (Uint32)0x80000000,  (Uint32)0x7FFFFFFF,  ACCESS_RO|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH,
/*60FDh*/       0,          0,              0x00000000,         (Uint32)0xFFFFFFFF, ACCESS_RO|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH,

/*4
    Uint32 ObjectDictionaryStandard.ComEntryDIDO.DigitalInputs.all,                      //60FDh
    Uint32 ObjectDictionaryStandard.ComEntryDIDO.DigitalOutputs.NumberOfEntries,         //60FEh
    Uint32 ObjectDictionaryStandard.ProVelMode2.TargetVelocity,                          //60FFh
*/
/*默认属性：AccessType| DataType| DataSize*/
/*注释      子索引个数    默认值            下限                 上限               属性*/
/*60FEh-0h*/    2,          2,              0x00,               (Uint32)0xFF,       ACCESS_RO|TYPE_UINT8|ONE_BYTE|UNMAPPING|BOTH,
/*60FEh-1h*/    2,          0,              0x00000000,         (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|MAPPING|BOTH,
/*60FEh-2h*/    2,          0,              0x00000000,         (Uint32)0xFFFFFFFF, ACCESS_RW|TYPE_UINT32|FOUR_BYTE|UNMAPPING|BOTH|COMSAVE,
/*60FFh*/       0,      (Uint32)0,         (Uint32)0x80000000,  (Uint32)0x7FFFFFFF, ACCESS_RW|TYPE_INT32|FOUR_BYTE|MAPPING|BOTH|COMSAVE,
/*6502h*/       /*0,          0,              0x00000000,         0xFFFFFFFF,         ACCESS_RO|TYPE_UINT32|FOUR_BYTE,*/
};



/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */
Uint32 * Scan_FunCode_Table(Uint16* ErrCode, Uint16 Index, Uint8 Subindex, STR_ODATTRIB *pODAttrib);
Uint32 * Scan_OD_Table(Uint16* ErrCode, Uint16 Index, Uint8 Subindex,STR_ODATTRIB *pODAttrib);
Uint16 LimitCheck(Uint16 IndexTemp, Uint32 DataInput);
Uint16 GetODFunCodeOffset(Uint32 *pAddEnd);
void AppDataToCurrentValue(void);								

/*******************************************************************************
  函数名: Uint32 Scan_FunCode_Table (Uint32* ErrCode, Uint16 Index, Uint16 Subindex); 
  输入:   参数1：	ErrCode		SDO中断故障码；
          参数2：	Index 	    对象字典索引
          参数3:    Subindex    对象字典子索引
  输出:   不成功，返回0
          成功，返回对象字典地址。
  子函数: 无       
  描述:   搜索预查找的对象字典索引值是否存在，若存在，返回该对象字典地址
********************************************************************************/ 
Uint32 * Scan_FunCode_Table(Uint16* ErrCode, Uint16 Index, Uint8 Subindex, STR_ODATTRIB *pODAttrib)
{
    Uint16 Group = 0;
    Uint16 Offset = 0;
    Uint16 SubindexCount = 0;
    Uint16 SubindexCountOpen = 0;
    Uint8  FoundFlag = 0;
    Uint16 *pFunCodeAddress=&PUB_Null_PointVar16Init;
    Uint32 *pAddress=&PUB_Null_PointVar32Init;

    switch(Index)
    {
        case 0x2000://00电机参数
            SubindexCount = H00LEN;
            SubindexCountOpen = 9;
            FoundFlag = 1;
            break;

        case 0x2001://01组驱动器参数
            SubindexCount = H01LEN;
            SubindexCountOpen = 3;
            FoundFlag = 1;
            break;
            
        case 0x2002://02组控制参数
            SubindexCount = H02LEN;
            SubindexCountOpen = 42;
            FoundFlag = 1;
            break;

        case 0x2003://03组DI参数
            SubindexCount = H03LEN;
            SubindexCountOpen = 36;
            FoundFlag = 1;
            break;

        case 0x2004://04组输出参数
            SubindexCount = H04LEN;
            SubindexCountOpen = 56;
            FoundFlag = 1;
            break;

        case 0x2005://05组位置控制参数
            SubindexCount = H05LEN;
            SubindexCountOpen = 62;
            FoundFlag = 1;
            break;
            
        case 0x2006://06组速度控制参数
            SubindexCount = H06LEN;
            SubindexCountOpen = 16;
            FoundFlag = 1;
            break;
            
        case 0x2007://07组转矩控制参数
            SubindexCount = H07LEN;
            SubindexCountOpen = 41;
            FoundFlag = 1;
            break;
            
        case 0x2008://08组增益参数
            SubindexCount = H08LEN;
            SubindexCountOpen = 26;
            FoundFlag = 1;
            break;
            
        case 0x2009://09组自调整参数
            SubindexCount = H09LEN;
            SubindexCountOpen = 40;
            FoundFlag = 1;
            break;
            
        case 0x200A://0A组保护参数
            SubindexCount = H0ALEN;
            SubindexCountOpen = 37;
            FoundFlag = 1;
            break;
            
        case 0x200B://0B组显示参数
            SubindexCount = H0BLEN;
            SubindexCountOpen = 100;
            FoundFlag = 1;
            break;
            
        case 0x200C://0C组通讯参数
            SubindexCount = H0CLEN;
            SubindexCountOpen = 46;
            FoundFlag = 1;
            break;
            
        case 0x200D://0D组辅助功能参数
            SubindexCount = H0DLEN;
            SubindexCountOpen = 23;
            FoundFlag = 1;
            break;
    
        case 0x200E:
            SubindexCount = H0ELEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x200F://0F组全闭环参数  注意05-38默认值为0
            SubindexCount = H0FLEN;
            SubindexCountOpen = 26;
            FoundFlag = 1;
            break;
            
        case 0x2010:
            SubindexCount = H10LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2011://11组多段位置
            SubindexCount = H11LEN;
            //FoundFlag = 1;
            break;
            
        case 0x2012://12组多段速度
            SubindexCount = H12LEN;
            //FoundFlag = 1;
            break;
            
        case 0x2013:
            SubindexCount = H13LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2014:
            SubindexCount = H14LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2015:
            SubindexCount = H15LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2016:
            SubindexCount = H16LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2017://虚拟DI
            SubindexCount = H17LEN;
            SubindexCountOpen = 65;
            FoundFlag = 1;
            break;
            
        case 0x2018:
            SubindexCount = H18LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2019:
            SubindexCount = H19LEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x201A:
            SubindexCount = H1ALEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x201B:
            SubindexCount = H1BLEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x201C:
            SubindexCount = H1CLEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x201D:
            SubindexCount = H1DLEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x202F:
            SubindexCount = H2FLEN;//暂不支持
            //FoundFlag = 1;
            break;
            
        case 0x2030:
            SubindexCount = H30LEN;
            SubindexCountOpen = 4;
            FoundFlag = 1;
            break;
            
        case 0x2031:
            SubindexCount = H31LEN;
            SubindexCountOpen = 5;
            FoundFlag = 1;
            break;
            
        case 0x2032:
            SubindexCount = H32LEN;//暂不支持
            //FoundFlag = 1;
            break;

        case 0x203F:
            SubindexCount = 0;
            SubindexCountOpen = 0;
            FoundFlag = 1;
            break;

        default:
            break;
    }

    if(!FoundFlag)
    {
        *ErrCode = COMM_OBJECT_NOT_EXISTING;
        pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
		return pAddress;
    }
    
    if(Index == 0x203F)
    {
        if(Subindex == 0)
        {
            pAddress = (Uint32 *)&STR_SerErrCode.ServorErrorCode;
            //属性
            pODAttrib->DataType = ATTRIB_TYPE_UINT32;
            pODAttrib->DataSize = ATTRIB_FOUR_BYTE;
            pODAttrib->AccessType = ATTRIB_RO;
            pODAttrib->Active = 0;
            pODAttrib->Mapping = ATTRIB_MAPPING;
			pODAttrib->SubindexCount = 0;
			pODAttrib->ODBelong = ATTRIB_BOTH;
            pODAttrib->ComSave = ATTRIB_UNCOMSAVE;
            return pAddress;
        }
        else
        {
    		*ErrCode = COMM_SUBINDEX_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }
    }
    else if(Subindex > SubindexCount)
    {
		*ErrCode = COMM_SUBINDEX_NOT_EXISTING;
        pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
		return pAddress;
    }
    else
    {
        Uint16 temp1 = 0;
        Group  = Index & 0x00FF;
			
		//未输入密码
        if(FunCodeUnion.code.OEM_OEMPass != OEMPASSWORD)
        { 
            if(Group == 1)//H01组
            {
                //可见长度
				pODAttrib->SubindexCount = SubindexCountOpen;
				if(Subindex > pODAttrib->SubindexCount)
				{
					*ErrCode = COMM_UNSUPPORTED_ACCESS;
                    pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
            		return pAddress;
				}
				else
				{
				}				
            }
			else if(Group == 0)//H00组
			{
	            //未输入H0240 不用进行0000为65535的检测
	            if(FunCodeUnion.code.MT_EnVisable <= 10000) 
	            {
	                pODAttrib->SubindexCount = SubindexCountOpen;
					if(Subindex > pODAttrib->SubindexCount)
					{
						*ErrCode = COMM_UNSUPPORTED_ACCESS;
                        pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
                		return pAddress;
					}
					else
					{
					}
	            }
				//已输入0240，可读使能打开
                else
				{
					pODAttrib->SubindexCount = 38;
				}
			}
			else
			{
			   pODAttrib->SubindexCount = SubindexCountOpen;
			}
        }
		else//已经输入密码读写权限打开
		{
            if(Group == 0)
            {
                pODAttrib->SubindexCount = 38;
            } 
            else if(Group == 1)
            {
                pODAttrib->SubindexCount = 67;
            }
            else
            {
                pODAttrib->SubindexCount = SubindexCount; 		 
            }
		}


		if(Subindex == 0)//读子索引个数
		{
		    pODAttrib->DataType = ATTRIB_TYPE_UINT8;
		    pODAttrib->DataSize = ATTRIB_ONE_BYTE;
		    pODAttrib->AccessType = ATTRIB_RO;
		    pODAttrib->Active = 0;
		    pODAttrib->Mapping = ATTRIB_UNMAPPING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
		}
		pODAttrib->Mapping = ATTRIB_MAPPING;
        Offset = Subindex - 1;
		temp1 = GetGroupCodeDftIndex(Group ,Offset);//属性表的索引号

        //保留参数
        if(FunCodeDeft[temp1].Attrib.bit.Writable == ATTRIB_RSVD_WRT)
        {
            *ErrCode = COMM_UNSUPPORTED_ACCESS;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }

        //16位功能码
        if(FunCodeDeft[temp1].Attrib.bit.DataBits == ATTRIB_ONE_WORD)
        {
            pFunCodeAddress = ((Uint16 *)FunCode_GroupStartAddr[Group] + Offset);
            pAddress = (Uint32 *)pFunCodeAddress;
            //功能码属性判断
            pODAttrib->DataSize = ATTRIB_TWO_BYTE;
            pODAttrib->DataType = ((FunCodeDeft[temp1].Attrib.bit.Sign == 1)? ATTRIB_TYPE_INT16 : ATTRIB_TYPE_UINT16);
        }
        else
        {
            //32位功能码，必须从低位进入索引
            if(FunCodeDeft[temp1].Attrib.bit.DataIndex == ATTRIB_HIGH_WORD)
            {
                *ErrCode = COMM_SUBINDEX_NOT_EXISTING;
                pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
        		return pAddress;
            }
            
            pFunCodeAddress = ((Uint16 *)FunCode_GroupStartAddr[Group] + Offset);
            pAddress = (Uint32 *)pFunCodeAddress;
            //功能码属性判断
            pODAttrib->DataSize = ATTRIB_FOUR_BYTE;
            pODAttrib->DataType = ((FunCodeDeft[temp1].Attrib.bit.Sign == 1)? ATTRIB_TYPE_INT32 : ATTRIB_TYPE_UINT32);
        }

        pODAttrib->Active  = FunCodeDeft[temp1].Attrib.bit.Active;
        pODAttrib->ComSave = FunCodeDeft[temp1].Attrib.bit.CommSaveEn;

        if(Group==0)
		{
			//0241功能
			if(FunCodeUnion.code.OEM_OEMPass != OEMPASSWORD) //除了0000均为只读
			{
			    if(Subindex>1)
				{
					pODAttrib->AccessType = ATTRIB_RO;
				}
				else//0000
				{
					if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)
                    {
                        pODAttrib->AccessType = ATTRIB_RO;
                    }
                    else
                    {
                        pODAttrib->AccessType = ATTRIB_RW;
                    }
				} 
			}
			else//密码权限打开
			{
		        //如果H0000!=65535或14000,00组除了H0000外都不可以改写
		        if(((FunCodeUnion.code.MT_MotorModel == 65535) || (14 != (FunCodeUnion.code.MT_MotorModel/1000))) && 
		        (Subindex > H00_PANELDISPLEN_OEM) )
		        {
    				switch(FunCodeDeft[temp1].Attrib.bit.Writable)
    	            {
    	                case ATTRIB_DISP_WRT:
    	                    pODAttrib->AccessType = ATTRIB_RO;
    	                    break;
    	
    	                case ATTRIB_POSD_WRT:
    	                    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)
    	                    {
    	                        pODAttrib->AccessType = ATTRIB_RO;
    	                    }
    	                    else
    	                    {
    	                        pODAttrib->AccessType = ATTRIB_RW;
    	                    }
    	                    break;
    	
    	                case ATTRIB_ANY_WRT:
    	                    pODAttrib->AccessType = ATTRIB_RW;
    	                    break;
    	
    	                default:
    	                    break;
    	            }
		        }
				else
				{
				    if(Subindex>1)
					{
						pODAttrib->AccessType = ATTRIB_RO;
					}
					else//0000
					{
						if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)
	                    {
	                        pODAttrib->AccessType = ATTRIB_RO;
	                    }
	                    else
	                    {
	                        pODAttrib->AccessType = ATTRIB_RW;
	                    }
					} 
				}

			}
		}
		else
		{
			switch(FunCodeDeft[temp1].Attrib.bit.Writable)
            {
                case ATTRIB_DISP_WRT:
                    pODAttrib->AccessType = ATTRIB_RO;
                    break;

                case ATTRIB_POSD_WRT:
                    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)
                    {
                        pODAttrib->AccessType = ATTRIB_RO;
                    }
                    else
                    {
                        pODAttrib->AccessType = ATTRIB_RW;
                    }
                    break;

                case ATTRIB_ANY_WRT:
                    pODAttrib->AccessType = ATTRIB_RW;
                    break;

                default:
                    break;
            }
		}
        return pAddress;
    }
}

/*******************************************************************************
  函数名: Uint32 Scan_ObjectDictionaryStandardTable (Uint32* ErrCode, Uint16 Index, Uint16 Subindex); 
  输入:   参数1：	errCode		SDO中断故障码；
          参数2：	index 	    对象字典索引
          参数3:    Subindex    对象字典子索引
  输出:   不成功，返回0
          成功，返回对象字典首地址。
  子函数: 无       
  描述:   搜索预查找的对象字典索引值是否存在，若存在，返回该对象字典首地址
********************************************************************************/ 
Uint32 *  Scan_OD_Table(Uint16* ErrCode, Uint16 Index, Uint8 Subindex,STR_ODATTRIB *pODAttrib)
{
    Uint16 IndexOffset = 0;
    Uint32 *pAddress = &PUB_Null_PointVar32Init;
    int32 indextemp = 0;

    if(Index == 0x6502)//6502特殊处理
    {
        pODAttrib->SubindexCount = 0;

        if(Subindex > pODAttrib->SubindexCount)
        {
		    *ErrCode = COMM_SUBINDEX_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }
        pAddress = (Uint32 *)&ObjectDictionaryStandard.ComEntrySurpDrivMod.SupportedDriveModes.all;
        //属性
        pODAttrib->DataType   = ATTRIB_TYPE_UINT32;
        pODAttrib->DataSize   = ATTRIB_FOUR_BYTE;
        pODAttrib->AccessType = ATTRIB_RO;
        pODAttrib->Mapping    = ATTRIB_MAPPING;
        pODAttrib->ODBelong   = ATTRIB_BOTH;
        pODAttrib->ComSave    = ATTRIB_UNCOMSAVE;
        return pAddress;
    }
    else if((Index >= 0x603F) && (Index <= 0x60FF))//目前包含的对象字典范围:603F~60FF
    {
		IndexOffset = Index - 0x603F;
        pAddress = GetOD_Add(IndexOffset,0);//0h子索引的地址

        if(pAddress == ((Uint32 *)&PUB_Null_PointVar32Init))//空的对象
        {
            *ErrCode = COMM_OBJECT_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }
	    //0h子索引在属性表中的索引号
        indextemp =(int32)(pAddress-((Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode));

        //OD在CANopen和EtherCAT中是否分别支持的判断
        pODAttrib->ODBelong = ObjectDictionaryDefault[indextemp].Attrib.bit.ODBelong;

    //以CANopen为接口测试时，关闭预编译，以EtherCAT为接口测试时，打开预编译
    #if ECT_ENABLE_SWITCH
        if(pODAttrib->ODBelong == 1)//ECT软件 CANopen only
        {
	        *ErrCode = COMM_OBJECT_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }
    #elif  CAN_ENABLE_SWITCH
        if(pODAttrib->ODBelong == 2)//CANopen软件 ECT only
        {
	        *ErrCode = COMM_OBJECT_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }
    #endif
    
        pODAttrib->SubindexCount = ObjectDictionaryDefault[indextemp].SubindexCount;
        if(Subindex > pODAttrib->SubindexCount)
        {
	        *ErrCode = COMM_SUBINDEX_NOT_EXISTING;
            pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    		return pAddress;
        }

        pAddress = GetOD_Add(IndexOffset , Subindex);//非0h的子索引的地址
		indextemp =(int32)(pAddress-((Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode));
        //属性
        pODAttrib->DataType   = ObjectDictionaryDefault[indextemp].Attrib.bit.DataType;
        pODAttrib->DataSize   = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;
        pODAttrib->AccessType = ObjectDictionaryDefault[indextemp].Attrib.bit.AccessType;
        pODAttrib->Mapping    = ObjectDictionaryDefault[indextemp].Attrib.bit.PDOMapping;
        pODAttrib->ComSave    = ObjectDictionaryDefault[indextemp].Attrib.bit.ComSave;
        return pAddress;

    }
    else
    {
        *ErrCode = COMM_OBJECT_NOT_EXISTING;
        pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
		return pAddress;
    }
}


/*******************************************************************************
  函数名: Uint16 C_Read_FunCode(Uint16 GroupIndex,Uint8 Offset,Uint8*dataSize,Uint32* pData); 
  输入:   参数1：	index		对象字典索引；
          参数2：	subindex 	对象字典子索引；
          参数3：	dataSize	数据长度，读：输出长度；写：输入长度；
          参数4：	pData		数据指针；
  输出:   0--成功，大于0--错误码。
  子函数: 无       
  描述:   读出指定的对象字典子索引的数据
********************************************************************************/ 
Uint16 C_Read_FunCode(Uint16 GroupIndex,Uint8 Offset,Uint8*dataSize,Uint32* pData)
{
    Uint32 *pAddress=&PUB_Null_PointVar32Init;
    Uint16 ErrCode = 0;
    Uint16 FunCodeGroup = 0;
    Uint16 FunCodeOffset = 0;
	Uint16 *pAddrNext=&PUB_Null_PointVar16Init;
	Uint16 *pAddr = &PUB_Null_PointVar16Init;

    STR_ODATTRIB *pODRead = &STR_ODAttrib;

    if((GroupIndex == 0x6502) || ((GroupIndex >= 0x603F) && (GroupIndex <= 0x60FF)))
    {
        pAddress = Scan_OD_Table(&ErrCode, GroupIndex,Offset,pODRead);
        //有错误
        if (ErrCode != 0) return ErrCode;

        if((pODRead->AccessType == ATTRIB_RW)||(pODRead->AccessType == ATTRIB_RO))
        {
            //首先把长度返回
            *dataSize = (Uint8)(pODRead->DataSize<<3);
            //如果读的是子索引个数
            if((pODRead->SubindexCount > 0)&&(Offset == 0))*pData= pODRead->SubindexCount;

            else
            {
                *pData = *pAddress;
            }
        }
        else
        {
	        ErrCode = COMM_WRITE_ONLY_ENTRY;
        }
    }
    else if((GroupIndex == 0x203F) || ((GroupIndex >= 0x2000) && (GroupIndex <= 0x2032)))
    {
        pAddress = Scan_FunCode_Table(&ErrCode, GroupIndex,Offset,pODRead);

        //有错误
        if (ErrCode != 0) return ErrCode;

        if((pODRead->AccessType == ATTRIB_RW)||(pODRead->AccessType == ATTRIB_RO))
        {
            //首先把长度返回
            *dataSize = (Uint8)(pODRead->DataSize<<3);
            //如果是203F
            if(GroupIndex == 0x203F)
            {
                *pData = *pAddress;
            }
            else
            {
                if(Offset==0) *pData = pODRead->SubindexCount;
				else
				{
					//无错误,读取功能码
	                FunCodeGroup  = GroupIndex & 0x00FF;
	                FunCodeOffset = Offset - 1;

	                pAddr = (Uint16*)FunCode_GroupStartAddr[(FunCodeGroup)] + (FunCodeOffset);
	    			pAddrNext = (Uint16*)FunCode_GroupStartAddr[(FunCodeGroup)] + (FunCodeOffset + 1);
	
	                switch (pODRead->DataType)
	                {   
	                    case ATTRIB_TYPE_INT16:
	                    case ATTRIB_TYPE_UINT16:
                            if((FunCodeGroup==0x02)&&((FunCodeOffset==40)||(FunCodeOffset==41)))
                            {
                                *pData = 0;
                            }
                            else
                            {
                                *pData = *pAddr;
                            }
	                        break;
	                        
	                    case ATTRIB_TYPE_INT32:
	                    case ATTRIB_TYPE_UINT32:
                                *pData = (((Uint32)(*pAddrNext))<<16) + (Uint32)(*pAddr);
	                        break;
	
	                    default:    
	        		        ErrCode = COMM_PARAM_LENGTH_ERROR;
	                        break;
	                }
				}
            }
        }
        else
        {
	        ErrCode = COMM_WRITE_ONLY_ENTRY;
        }
    }
    else
    {
        ErrCode = COMM_OBJECT_NOT_EXISTING;
    }
	return ErrCode;
}
/*******************************************************************************
  函数名: Uint32 SDO_Write_FunCode (Uint16 index, Uint8 subindex, Uint32 dataSize, Uint16* pData); 
  输入:   参数1：	index		对象字典索引；
          参数2：	subindex 	对象字典子索引；
          参数3：	dataSize	数据长度，读：输出长度；写：输入长度；
          参数4：	pData		数据指针；
  输出:   0--成功，大于0--错误码。
  子函数: 无       
  描述:   写指定的对象字典子索引的数据
********************************************************************************/ 
Uint16 C_Write_FunCode(Uint16 GroupIndex,Uint8 Offset,Uint8 dataSize,Uint32 Data,Uint8 EepromFlag)
{
   
    Uint16 ErrCode = 0;
    Uint32 *pAddress=&PUB_Null_PointVar32Init;
	Uint16 *pAddrNext=&PUB_Null_PointVar16Init;
	Uint16 *pAddr = &PUB_Null_PointVar16Init;
    Uint16 FunCodeGroup = 0;
    Uint16 FunCodeOffset = 0;
    Uint8  LmtCheck = 0;
    int32  IndexTemp = 0;//临时序号
    Uint32 FunCodeTemp0 =0;
    int32 FunCodeTemp1=0;
    Uint16 ValueTemp0=0;
    Uint16 ValueTemp1=0;
    Uint8 ValueEqual = 0;
    Uint8 ValueTemp2 = 0;
    
    STR_ODATTRIB *pODWrite = &STR_ODAttrib;

    if((GroupIndex == 0x6502)||((GroupIndex >= 0x603F)&&(GroupIndex <= 0x60FF)))//需判断
    {
        pAddress = Scan_OD_Table(&ErrCode, GroupIndex,Offset,pODWrite);

        //有错误
        if (ErrCode != 0) return ErrCode;

        if((pODWrite->AccessType == ATTRIB_RW)||(pODWrite->AccessType == ATTRIB_WO))
        {
            //OD复位过程中
            if((STR_FUNC_Gvar.ManageFunCodeOutput.ResetFunCode == 1)||(STR_FUNC_Gvar.ManageFunCodeOutput.CANResetFunCode==1)||
                (STR_FUNC_Gvar.ManageFunCodeOutput.ResetOD == 1)) return COMM_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE;
			
			if(((dataSize>>3) > (Uint8)pODWrite->DataSize))return COMM_PARAM_LENGTH_TOO_LONG;//数据长度high

            if(((dataSize>>3) < (Uint8)pODWrite->DataSize))return COMM_PARAM_LENGTH_TOO_SHORT;//数据长度low

            IndexTemp =(int32)(pAddress - (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode);

            //SDO写6060，参考施耐德进行处理
            if(GroupIndex == 0x6060)
            {
                ValueTemp2 = (Uint8)Data;
                ErrCode = LimitCheck(IndexTemp, (Uint32)ValueTemp2);

                if(ErrCode == 0)
                {
                    #if ECT_ENABLE_SWITCH
                        if((ValueTemp2 != ECTPOSMOD)&&(ValueTemp2 != ECTSPDMOD)
                            &&(ValueTemp2 != ECTTOQMOD)&&(ValueTemp2 != ECTHOMMOD)
                            &&(ValueTemp2 != ECTCSPMOD)&&(ValueTemp2 != ECTCSVMOD)
                            &&(ValueTemp2 != ECTCSTMOD))
                            return COMM_GENERAL_ERROR;
                        
                    #elif  CAN_ENABLE_SWITCH
                        if((ValueTemp2 != CANOPENPROPOSMOD)&&(ValueTemp2 != CANOPENPROVELMOD)
                            &&(ValueTemp2 != CANOPENHOMMOD)&&(ValueTemp2 != CANOPENINTPMD)
							&&(ValueTemp2 != CANOPENPROTOQMOD))
                            return COMM_GENERAL_ERROR;
                        
                    #endif

                    else
                    {
                        *pAddress = ValueTemp2;
                    }
                }
                else{}
            }
            else
            {
                switch (pODWrite->DataType)
                {
                    case ATTRIB_TYPE_INT8: //需要判断上下限
                    case ATTRIB_TYPE_UINT8:
                        ValueTemp2 = (Uint8)Data;
                        ErrCode = LimitCheck(IndexTemp, (Uint32)ValueTemp2);
                        //根据现场应用修改
                        if((GroupIndex == 0x6098)&&((ValueTemp2==15)||(ValueTemp2==16)||(ValueTemp2==31)||(ValueTemp2==32)))
                        {
                            ErrCode = COMM_VALUE_EXCEEDED;
                            PostErrMsg(ODVALUEERR);
                        }
                        else{}

                        if(ErrCode == 0) *pAddress = ValueTemp2;
                        else{}
                        break;

                    case ATTRIB_TYPE_INT16:
                    case ATTRIB_TYPE_UINT16:
                        ValueTemp0 = (Uint16)Data;
                        ErrCode    = LimitCheck(IndexTemp, (Uint32)ValueTemp0);
                        //根据现场应用修改
                        if((GroupIndex == 0x605A)&&(ValueTemp0==0x0004))
                        {
                            ErrCode = COMM_VALUE_EXCEEDED;
                        }
                        else{}
                        
                        if(ErrCode == 0) *pAddress = ValueTemp0;
                        else{}
    				    break;
                    
                    case ATTRIB_TYPE_INT32:
                    case ATTRIB_TYPE_UINT32:
                        FunCodeTemp0 = Data;
                        ErrCode      = LimitCheck(IndexTemp, FunCodeTemp0);
                        if(ErrCode == 0) *pAddress = FunCodeTemp0;
                        else{}
                        break;

                    default:    
    		            ErrCode = COMM_PARAM_LENGTH_ERROR;
                    break;
                }
            }//-------------------- Eeprom存储数据 --------------------
            if(((FunCodeUnion.code.CM_WriteEepromEnable == 2)||(FunCodeUnion.code.CM_WriteEepromEnable == 3))
                &&(pODWrite->ComSave == ATTRIB_COMSAVE))
            {
                if(pODWrite->DataSize <= ATTRIB_TWO_BYTE)
    			{
                    IndexTemp = GetODFunCodeOffset(pAddress);
                    if((FunCodeUnion.all[IndexTemp]  != (Uint16)(*pAddress)))
                    {
                        FunCodeUnion.all[IndexTemp] = (Uint16)(*pAddress);
                        SaveToEepromOne(IndexTemp);
                    }
                    else{}
    			}
    			else
    			{
                    IndexTemp = GetODFunCodeOffset(pAddress);
                    if((A_SHIFT16_PLUS_B(FunCodeUnion.all[IndexTemp+1],FunCodeUnion.all[IndexTemp])) != *pAddress)
                    {
                        FunCodeUnion.all[IndexTemp] = (Uint16)(*pAddress); 
                        FunCodeUnion.all[IndexTemp + 1] = (Uint16)((*pAddress) >> 16); 
                        SaveToEepromSeri(IndexTemp,IndexTemp + 1);
                    }
                    else{}
    			}
            }
        }
        else
        {
	        ErrCode = COMM_READ_ONLY_ENTRY;
        }
    }
    else if((GroupIndex == 0x203F)||((GroupIndex >= 0x2000)&&(GroupIndex <= 0x2032)))
    {
        pAddress = Scan_FunCode_Table(&ErrCode, GroupIndex,Offset,pODWrite);
        //有错误
        if (ErrCode != 0) return ErrCode;

        if((pODWrite->AccessType == ATTRIB_RW)||(pODWrite->AccessType == ATTRIB_WO))
        {
            //功能码复位过程中
            if((STR_FUNC_Gvar.ManageFunCodeOutput.ResetFunCode == 1)
			    ||(STR_FUNC_Gvar.ManageFunCodeOutput.CANResetFunCode == 1)
				||(STR_FUNC_Gvar.ManageFunCodeOutput.ResetOD == 1)) return COMM_DATA_CANNOT_BE_READ_OR_STORED_IN_THIS_STATE;

            if(((dataSize>>3) > (Uint8)pODWrite->DataSize))return COMM_PARAM_LENGTH_TOO_LONG;//数据长度high

            if(((dataSize>>3) < (Uint8)pODWrite->DataSize))return COMM_PARAM_LENGTH_TOO_SHORT;//数据长度low

            FunCodeGroup  = GroupIndex & 0x00FF;
            FunCodeOffset = Offset - 1;

            //H0904置1时，H0938和H0939只做显示
            if((FunCodeUnion.code.AT_LowOscMod == 1) && (FunCodeGroup == 0x09)
                && (FunCodeOffset == 38 || (FunCodeOffset == 39)))
            {
                return COMM_READ_ONLY_ENTRY;
            } 
            
            IndexTemp = GetGroupCodeDftIndex(FunCodeGroup ,FunCodeOffset);//属性表的索引号
            pAddr = (Uint16*)FunCode_GroupStartAddr[(FunCodeGroup)] + (FunCodeOffset);
			pAddrNext = (Uint16*)FunCode_GroupStartAddr[(FunCodeGroup)] + (FunCodeOffset + 1);
           
            //----------H0241设定检查----------
            if((FunCodeGroup == 0x02) && (FunCodeOffset == 41) && (Data != OEMPASSWORD)) return COMM_VALUE_EXCEEDED;

            //----------H0240设定检查----------
            if((FunCodeGroup == 0x02) && (FunCodeOffset == 40) && (Data < 10000)) return COMM_VALUE_EXCEEDED;


            switch (pODWrite->DataType)
            {  
				case ATTRIB_TYPE_INT16:
				    ValueTemp0 = (int16)((Uint16)Data);
                    LmtCheck = LimitCheck_0neWord(IndexTemp,ValueTemp0);
                    if(LmtCheck != 0) 
                    {   
                        ErrCode = COMM_VALUE_EXCEEDED;
                    }
                    else//还要加入941标志
					{
                        if((*(int16*)pAddr) != ((int16)ValueTemp0))*(int16*)pAddr = (int16)ValueTemp0;
                        else ValueEqual = 1;
                    }
                    break;

                case ATTRIB_TYPE_UINT16:
                    ValueTemp0 = (Uint16)Data;
                    LmtCheck = LimitCheck_0neWord(IndexTemp,ValueTemp0);
                    if(LmtCheck != 0) 
                    {   
                        ErrCode = COMM_VALUE_EXCEEDED;
                    }
                    else//还要加入941标志
					{
                        if((*pAddr) != ValueTemp0)*pAddr = ValueTemp0; 
                        else ValueEqual = 1;
                    }
				    break;
                
                case ATTRIB_TYPE_INT32:
				    ValueTemp0 =(int16)(Data&0x0000FFFF);
				    ValueTemp1 =(int16)(Data>>16);
                    FunCodeTemp1 = (int32)Data;
				    //调用内联比较函数
                    LmtCheck = LimitCheck_TwoWords(IndexTemp, FunCodeTemp1);
                    if(LmtCheck != 0) 
                    {   
                        ErrCode = COMM_VALUE_EXCEEDED;
                    }
                    else//还要加入941标志
					{
                        if(((*(int16 *)pAddr) != ((int16)ValueTemp0))||((*(int16 *)pAddrNext) != ((int16)ValueTemp1)))
                        {
                            *(int16 *)pAddr  = (int16)ValueTemp0;
                            *(int16 *)pAddrNext = (int16)ValueTemp1;
                        }
                        else ValueEqual = 1;
                    }
				    break;

                case ATTRIB_TYPE_UINT32:
				    ValueTemp0 =(Uint16)(Data&0x0000FFFF);
				    ValueTemp1 =(Uint16)((Data&0xFFFF0000)>>16);
                    FunCodeTemp0 = Data;
				    //调用内联比较函数
                    LmtCheck = LimitCheck_TwoWords(IndexTemp, FunCodeTemp0);
                    if(LmtCheck != 0) 
                    {   
                        ErrCode = COMM_VALUE_EXCEEDED;
                    }
                    else//还要加入941标志
					{
                        if(((*pAddr) != ValueTemp0)||((*pAddrNext) != ValueTemp1))
                        {
                            *pAddr = ValueTemp0;
                            *pAddrNext  = ValueTemp1;
                        }
                        else ValueEqual = 1;
                    }
                    break;

                default:    
		            ErrCode = COMM_PARAM_LENGTH_ERROR;
                    break;
            }
        }
        else
        {
	        ErrCode = COMM_READ_ONLY_ENTRY;
        }
        
        //写入数据与原数据不相等
        if(ValueEqual == 0)
        {
            //如果更改H03 H04 H17组参数 STR_FUNC_Gvar.ManageFunCodeOutput.AiAoDiDoUpdate置1
            if((FunCodeGroup == 0x03) || (FunCodeGroup == 0x04) || (FunCodeGroup == 0x17))
            {
                STR_FUNC_Gvar.ManageFunCodeOutput.AiAoDiDoUpdate = 1;
            }

            //-------------------- Eeprom存储数据 --------------------
            if((FunCodeGroup == 0x0C) && (FunCodeOffset == 13))
            {
                SaveToEepromOne(GetGroupCodeIndex(FunCodeGroup ,FunCodeOffset));               
            }
            //-------------------- Eeprom存储数据 --------------------

            // 再次上电生效的警告
            if(pODWrite->Active == 1)
            {
                PostErrMsg(PCHGDWARN);
            }
        }
        else
        {}

        if(((FunCodeUnion.code.CM_WriteEepromEnable == 1)||(FunCodeUnion.code.CM_WriteEepromEnable == 3))&&
           (FunCodeDeft[IndexTemp].Attrib.bit.CommSaveEn == 0) &&
           (FunCodeGroup != 0x0D) && (FunCodeGroup != 0x0B) && (FunCodeGroup != 0x31) && (FunCodeGroup != 0x32))
        {
            if(pODWrite->DataSize == ATTRIB_TWO_BYTE)
			{
                IndexTemp = GetGroupCodeIndex(FunCodeGroup ,FunCodeOffset);
                SaveToEepromOne(IndexTemp);
			}
			else
			{
                IndexTemp = GetGroupCodeIndex(FunCodeGroup ,FunCodeOffset);
                SaveToEepromSeri(IndexTemp,IndexTemp + 1);
			}
        }
	}
    else
    {
        ErrCode = COMM_OBJECT_NOT_EXISTING;
    }
    
	
    	
	return ErrCode;	
}

/*******************************************************************************
  函数名: Uint32 PdoCheckObj(Uint16 Index, Uint8 Subindex, Uint32 DataSize, Uint32 RW) 
  输入:   参数1：	Index		对象字典索引；
          参数2：	Subindex 	对象字典子索引；
          参数3：	DataSize	数据长度，读：输出长度；写：输入长度；
          参数4：	RW		    对象字典可读写性；
  输出:   0--成功，大于0--错误码。
  子函数: 无       
  描述:   确认对象字典是否可以映射到PDO,可读--TPDO,可写---RPDO
********************************************************************************/ 
Uint32 PdoCheckObj(Uint16 Index, Uint8 Subindex, Uint32 DataSize, Uint32 rw)
{
    Uint32 AccessTypeReadTemp=0;
    Uint32 AccessTypeWriteTemp=0;
//    Uint32 *pAddress=&PUB_Null_PointVar32Init;
    Uint16 ErrCode = 0;

    STR_ODATTRIB *pODMapping = &STR_ODAttrib;

    if((Index == 0x6502)||((Index >= 0x603F)&&(Index <= 0x60FF)))//需判断
    {
        //pAddress = 
        Scan_OD_Table(&ErrCode, Index,Subindex,pODMapping);
    }
    else if((Index == 0x203F)||(Index >= 0x2000)&&(Index <= 0x2032))
    {
        //pAddress = 
        Scan_FunCode_Table(&ErrCode, Index,Subindex,pODMapping);
    }
    else
    {
        return COMM_OBJECT_NOT_EXISTING;
    }

    //有错误
    if (ErrCode != 0) return ErrCode;

    if(pODMapping->Mapping == 0) return COMM_OBJECT_CANT_BE_PDOMAPPED;

    if((DataSize > pODMapping->DataSize))return COMM_PARAM_LENGTH_TOO_LONG;//数据长度high

    if((DataSize < pODMapping->DataSize))return COMM_PARAM_LENGTH_TOO_SHORT;//数据长度low

    switch(pODMapping->AccessType)
    {
        case ATTRIB_RW:
            AccessTypeReadTemp = 0;
            AccessTypeWriteTemp = 1;
            break;
            
        case ATTRIB_WO:
            AccessTypeReadTemp = 1;
            AccessTypeWriteTemp = 1;
            break;

        case ATTRIB_RO:
            AccessTypeReadTemp = 0;
            AccessTypeWriteTemp = 0;
            break;

        default:
            ErrCode = COMM_OBJECT_CANT_BE_PDOMAPPED;
            break;                        
    }
    if(((rw + AccessTypeReadTemp) == 0) ||((rw + AccessTypeWriteTemp)== 2) )return NULL;
    else return COMM_OBJECT_CANT_BE_PDOMAPPED;
    
}

/*******************************************************************************
  函数名: Uint32 *pGetFuncodeAddr(Uint16 CiaIndexTemp,Uint8CiaSubIndexTemp);
  输入:   参数1：	CiaIndexTemp		对象字典索引；
          参数2：	CiaSubIndexTemp 	对象字典子索引；
  输出:   对象字典地址指针。
  子函数: Uint32 PdoCheckObj(Uint16 Index, Uint8 Subindex, Uint32 DataSize, Uint32 RW)       
  描述:   返回对象字典地址，PDO映射用
********************************************************************************/
//Uint32 *pGetFuncodeAddr(Uint16 CiaIndexTemp,Uint8 CiaSubIndexTemp, Uint16 wPDOx)
Uint8 *C_pGetFuncodeAddr(Uint16 GroupIndex, Uint8 Offset)

{
    Uint32 *pAddress=&PUB_Null_PointVar32Init;
    Uint16 ErrCode = 0;

    STR_ODATTRIB *pODAdd = &STR_ODAttrib;
    
	//判断60C1映射的PDOx
	#if CAN_ENABLE_SWITCH
	if(GroupIndex == 0x60C1)
	{
		ObjectDictionaryStandard.InterpltPosMode2.IPmappedRPDOx = wPDOx; 
	}
    #endif

    //301会先判断可读写性，在调用该函数,所以在此处直接读地址即可
    if((GroupIndex == 0x6502)||((GroupIndex >= 0x603F)&&(GroupIndex <= 0x60FF)))//需判断
    {
        pAddress = Scan_OD_Table(&ErrCode, GroupIndex,Offset,pODAdd);
        //有错误
        if (ErrCode != 0) pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    }
    else if((GroupIndex == 0x203F)||(GroupIndex >= 0x2000)&&(GroupIndex <= 0x2032))
    {
        pAddress = Scan_FunCode_Table(&ErrCode, GroupIndex,Offset,pODAdd);
        //有错误
        if (ErrCode != 0)pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    }
    else
    {
        ErrCode = COMM_OBJECT_NOT_EXISTING;
        pAddress = (Uint32 *)&PUB_Null_PointVar32Init;
    }
    return (Uint8 *)pAddress;

}
/*******************************************************************************
  函数名:void CanopenServorErrorMsgFresh(Uint16 PostErrCode,Uint16 InnerErrCode) 
  输入: PostErrCode   外部故障码，内部故障码
        InnerErrCode
  输出: 无
  子函数: 无   
  描述:  查询伺服运行的故障码，603F--402故障，203F--伺服内部故障,在PostErrMsg(Uint32 PostErrCode)函数中被调用
********************************************************************************/
void CanopenServorErrorMsgFresh(Uint16 PostErrCode,Uint16 InnerErrCode) 
{

    Uint16 ErrorCodeStandard = 0;
    STR_SerErrCode.ServorErrorCode = A_SHIFT16_PLUS_B(InnerErrCode,PostErrCode);
    STR_SerErrCode.ServorErrorCode = STR_SerErrCode.ServorErrorCode & 0x0FFF0FFF;

    //只根据外部故障码区分，传递603F
	PostErrCode = PostErrCode& 0x0FFF; 

    switch(PostErrCode)
    {
        case 0x0200:
            ErrorCodeStandard = 0x2311;//Continuous over current No.1
            break;
            
        case 0x0201:
            ErrorCodeStandard = 0x2312;//Continuous over current No.2
            break;

        case 0x0210:
            ErrorCodeStandard = 0x2330;//Earth leakage
            break;

        case 0x0430:
            ErrorCodeStandard = 0x3120;//mains under-voltage
            break;

        case 0x0420:
        case 0x0990:
            ErrorCodeStandard = 0x3130;//Phase failure
            break;

        case 0x0400:
        case 0x0920:
            ErrorCodeStandard = 0x3210;//DC link over-voltage
            break;

        case 0x0410:
            ErrorCodeStandard = 0x3220;//DC link under-voltage
            break;

        case 0x0610:
        case 0x0620:
        case 0x0909:
            ErrorCodeStandard = 0x3230;//Load error
            break;

        case 0x0939:
            ErrorCodeStandard = 0x3331;//Field circuit interrupted
            break;

        case 0x0650:
        case 0x0760:
            ErrorCodeStandard = 0x4210;//Excess temperature device
            break;

        case 0x0831:
        case 0x0834:
        case 0x0835:
            ErrorCodeStandard = 0x5210;//Measurement circuit
            break;

        case 0x0121:
            ErrorCodeStandard = 0x5441;//Contact 1= manufacturer specific
            break;

        case 0x0900:
            ErrorCodeStandard = 0x5442;//Contact 2= manufacturer specific
            break;

        case 0x0950:
            ErrorCodeStandard = 0x5443;//Contact 3= manufacturer specific
            break;

        case 0x0952:
            ErrorCodeStandard = 0x5444;//Contact 4= manufacturer specific
            break;

        case 0x0108:
            ErrorCodeStandard = 0x5530;//Non volatile data memory
            break;

        case 0x0101:
        case 0x0105:
        case 0x0111:
        case 0x0130:
        case 0x0131:
        case 0x0110:
        case 0x0922:
        case 0x0941:
        case 0x0B03:
        case 0x0D09:
        case 0x0D10:
        case 0x0D11:
        case 0x0998:
        case 0x0B04:
            ErrorCodeStandard = 0x6320;//Parameter error
            break;

        case 0x0630:
            ErrorCodeStandard = 0x7121;//Non volatile data memory
            break;

        case 0x0120:
        case 0x0122:
            ErrorCodeStandard = 0x7122;//Motor error or commutation malfunc.
            break;
            
        case 0x0136:
        case 0x0A33:
        case 0x0A34:
        case 0x0A35:
        case 0x0980:
        case 0x0740:
        case 0x0745:
        case 0x0732:
        case 0x0731:
        case 0x0733:
        case 0x0735:
        case 0x0730:
        case 0x0755:
        case 0x0756:
            ErrorCodeStandard = 0x7305;//Incremental sensor 1 fault
            break;

        case 0x0770:
            ErrorCodeStandard = 0x7306;//Incremental sensor 2 fault
            break;
            
        case 0x0102:
        case 0x0103:
        case 0x0104:
            ErrorCodeStandard = 0x7500;//Communication
            break;

        case 0x0942:
            ErrorCodeStandard = 0x7600;//Data storage
            break;

        case 0x0D05:
            ErrorCodeStandard = 0x8160;//EMCY_NMT_TO_INIT	
            break;
            
        case 0x0D06:
            ErrorCodeStandard = 0x8170;//EMCY_NMT_TO_INIT	
            break;
            
        case 0x0500:
            ErrorCodeStandard = 0x8400;//Velocity speed controller
            break;

        case 0x0B00:
        case 0x0B02:
            ErrorCodeStandard = 0x8611;//Following error
            break;
            
            
        case 0x0208:
        case 0x0220:
        case 0x0207:
        case 0x0234:
        case 0x0602:
        case 0x0510:
        case 0x0B01:
        case 0x0A40:
        case 0x0601:
        case 0x0E08:
		case 0x0E07:
        case 0x0E11:
        case 0x0E12:
        case 0x0E13:
        case 0x0E15:
            ErrorCodeStandard = PostErrCode & 0x0FFF;//厂家自定义
            break;

        default:
		    ErrorCodeStandard = PostErrCode & 0x0FFF;//厂家自定义
            break;
            
    }
    ObjectDictionaryStandard.ComEntryErrCode.ErrorCode = ErrorCodeStandard;//603F
}

/*******************************************************************************
  函数名: Uint16 ResetAppData(void)								
  输入:   无
  输出:   0 --成功；否则--错误码
  子函数: 无   
  描述:   6000系列加载EEPROM数值
********************************************************************************/
#if CAN_ENABLE_SWITCH
Uint32 CANResetAppData(void)//已不调用
{
    //Uint16 Counter = 0;
    static Uint32 ErrCode = 0;

    //先读H1C~H1D
    if((STR_FUNC_Gvar.ManageFunCodeOutput.CANReadFunCode == 0)&&
        (STR_FUNC_Gvar.ManageFunCodeOutput.CANReadAppOD == 0)&&
        (STR_FUNC_Gvar.ManageFunCodeOutput.CANReadComOD == 0))
    {
        STR_FUNC_Gvar.ManageFunCodeOutput.CANReadAppOD = 1;
        ReadFromEeprom(H1C00INDEX,(HRsvd00INDEX - 1));
        EepromProcess();
		ErrCode = 2L;
    }
    else
    {
        if(EepromProcess()!=1)
        {
            ReadAppDataFromEeprom();
            STR_FUNC_Gvar.ManageFunCodeOutput.CANReadAppOD = 0;
            ErrCode = 0L;
        }
    }
	return ErrCode;
}
#endif
/*******************************************************************************
  函数名: Uint16 ResetFuncCode(void)								
  输入:   无
  输出:   0 --成功；否则--错误码
  子函数: 无   
  描述:  2000加载EEPROM数值
********************************************************************************/
#if CAN_ENABLE_SWITCH
Uint32 CANResetFuncCode (void)//已不调用
{
    Uint16 Counter = 0;
    static Uint32 ErrCode = 0;

    //先读H1C~H1D
    if((STR_FUNC_Gvar.ManageFunCodeOutput.CANReadFunCode == 0)&&
        (STR_FUNC_Gvar.ManageFunCodeOutput.CANReadAppOD == 0)&&
        (STR_FUNC_Gvar.ManageFunCodeOutput.CANReadComOD == 0))
    {
        STR_FUNC_Gvar.ManageFunCodeOutput.CANReadFunCode = 1;
        //先初始化辅助功能码
        for(Counter=0;Counter<AUXFUNCODELEN;Counter++)
        {
            AuxFunCodeUnion.all[Counter] = GetCodeDftValue(Counter + H0B00DFTINDEX);
        }

        //读取2000系列功能码
        ReadFromEeprom(0,(H1800INDEX-1));
        EepromProcess();
		ErrCode = 1L;
    }
    else
    {
        if(EepromProcess()!=1)
        {
            //不存储在Eeprom但需要初始化的变量
            FunCodeUnion.code.OEM_SoftVersion = VERSION_H0100;
            FunCodeUnion.code.OEM_FpgaVersion = FpgaSoftVersion;
            FunCodeUnion.code.MT_NonStandardVerL = VERSION_H0002;
            FunCodeUnion.code.MT_NonStandardVerH = VERSION_H0003;
            FunCodeUnion.code.OEM_DSPVerBD = VERSION_H0150;
            FunCodeUnion.code.OEM_FPGAVerBD = VERSION_H0151;
            C_SetCcmVision();
            FunCodeUnion.code.BP_UserPass = 0;
            STR_FUNC_Gvar.ManageFunCodeOutput.CANReadFunCode = 0;
            ErrCode=0L;
        }
    }
	return ErrCode;
}
#endif
/*******************************************************************************
  函数名: Uint32 AppDataToEeprom(Uint8 bDirection);								
  输入:   无
  输出:   0 --成功；否则--错误码(ACCESS_FAILED_BY_HARDWARE_ERROR)
  子函数: 无   
  描述:   
        0x1010：保存当前值到EEPROM
        01 保存所有参数
        02 保存通讯参数//CANopen only
        03 保存402区参数//CANopen only
        04 保存功能码区参数//CANopen only
        当子索引写"save"时，对应的区域参数保存一次。
        //保存6000h参数到EEPROM
        // bDirection = OBD_STORE，保存当前值
        // bDirection = OBD_RESTORE，保存默认值

********************************************************************************/
#if CAN_ENABLE_SWITCH
Uint32 AppDataToEeprom(Uint8 bDirection)
{
    Uint32 ErrCode = 0;
    static Uint16 Step = 0;
    //不可同时对EEPROM任意2种以上的存储
    
    //伺服运行时不可保存功能码
    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)return COMM_HARDWARE_ERROR;
	
	switch (bDirection)
    {
        case OBD_STORE://52.5ms
            for(Step=1;Step<=2;)
            {
                if(STR_FUNC_Gvar.ManageFunCodeOutput.SaveOD == 0)
                {
                    STR_FUNC_Gvar.ManageFunCodeOutput.SaveOD = 1;

                    //校验字清零,复位期间如果掉电,重新上电后复位所有的功能码
                    FunCodeUnion.code.EepromCheckWord1 = 0;
                    FunCodeUnion.code.EepromCheckWord2 = 0;
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                    
                    Step++;
                    AppDataToCurrentValue();
                    SaveToEepromSeri(H1C00INDEX,(HRsvd00INDEX - 1));
    				EepromProcess();
                }
                else
                {
                    if(EepromProcess() != 1)     //存储完成
                    {
                        //恢复校验字
                        FunCodeUnion.code.EepromCheckWord1 = EEPROM_CHECK_WORD1;
                        FunCodeUnion.code.EepromCheckWord2 = EEPROM_CHECK_WORD2;
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                        Step++;
                        
                        //复位完成
                        STR_FUNC_Gvar.ManageFunCodeOutput.SaveOD = 0;
                        
                    }
                }
            }
            break;
            
        case OBD_RESTORE:
            for(Step=1;Step<=2;)
            {

                if(STR_FUNC_Gvar.ManageFunCodeOutput.ResetOD == 0)  //0:未复位
                {
                    STR_FUNC_Gvar.ManageFunCodeOutput.ResetOD = 1;
                    //校验字清零,复位期间如果掉电,重新上电后复位所有的功能码
                    FunCodeUnion.code.EepromCheckWord1 = 0;
                    FunCodeUnion.code.EepromCheckWord2 = 0;
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                    Step++;
                    AppDataToDefultValue();
                    SaveToEepromSeri(H1C00INDEX,(HRsvd00INDEX - 1));
    				EepromProcess();
                }
                else
                {
                    if(EepromProcess() != 1)     //存储完成
                    {
                        //恢复校验字
                        FunCodeUnion.code.EepromCheckWord1 = EEPROM_CHECK_WORD1;
                        FunCodeUnion.code.EepromCheckWord2 = EEPROM_CHECK_WORD2;
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                        Step++;
                        //复位完成
                        STR_FUNC_Gvar.ManageFunCodeOutput.ResetOD = 0;
                    }
                }
            }
            break;

        default:
            break;
    }
    
    return ErrCode;

}
#endif
/*******************************************************************************
  函数名: Uint32 FuncCodeToEeprom(Uint8 bDirection);								
  输入:   无
  输出:   0 --成功；否则--错误码(ACCESS_FAILED_BY_HARDWARE_ERROR)
  子函数: 无   
  描述:   
        0x1010：保存当前值到EEPROM
        01 保存所有参数
        02 保存通讯参数//CANopen only
        03 保存402区参数//CANopen only
        04 保存功能码区参数//CANopen only
        当子索引写"save"时，对应的区域参数保存一次。
        //保存2000h参数到EEPROM
        // bDirection = OBD_STORE，保存当前值
        // bDirection = OBD_RESTORE，保存默认值
********************************************************************************/
#if CAN_ENABLE_SWITCH
Uint32 FuncCodeToEeprom(Uint8 bDirection)
{
    Uint32 ErrCode = 0;
    Uint32 Counter = 0;
    static Uint16 Step = 0;
    Uint64 Temp = 0;
    Uint64 Temp_1 = 0;

    //伺服运行时不可保存功能码
    if(STR_FUNC_Gvar.MonitorFlag.bit.ServoRunStatus == RUN)return COMM_HARDWARE_ERROR;
	
	switch (bDirection)
    {
        case OBD_STORE://760ms
            for(Step=1;Step<=2;)
            {
                if(STR_FUNC_Gvar.ManageFunCodeOutput.SaveFunCode == 0)
                {
                    STR_FUNC_Gvar.ManageFunCodeOutput.SaveFunCode = 1;

                    //校验字清零,复位期间如果掉电,重新上电后复位所有的功能码
                    FunCodeUnion.code.EepromCheckWord1 = 0;
                    FunCodeUnion.code.EepromCheckWord2 = 0;
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                    
                    Step++;
                    SaveToEepromSeri(H0000INDEX,(H1800INDEX-1));
                    EepromProcess();
                }
                else
                {
                    if(EepromProcess() != 1)
                    {
                        //恢复校验字
                        FunCodeUnion.code.EepromCheckWord1 = EEPROM_CHECK_WORD1;
                        FunCodeUnion.code.EepromCheckWord2 = EEPROM_CHECK_WORD2;
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                        Step++;
                        STR_FUNC_Gvar.ManageFunCodeOutput.SaveFunCode = 0;     //存储完成
                    }
                }            
            }
            break;
            
        case OBD_RESTORE:
            for(Step=1;Step<=2;)
            {

                if(STR_FUNC_Gvar.ManageFunCodeOutput.CANResetFunCode == 0)  //0:未复位
                {
                    STR_FUNC_Gvar.ManageFunCodeOutput.CANResetFunCode  = 1;
                    
                    //校验字清零,复位期间如果掉电,重新上电后复位所有的功能码
                    FunCodeUnion.code.EepromCheckWord1 = 0;
                    FunCodeUnion.code.EepromCheckWord2 = 0;
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                    SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                    
                    Step++;

                    //将H00组和H01组之外的功能码当前值存入EEPROM
                    for(Counter=H0200INDEX;Counter<H1800INDEX;Counter++)
                    {
                        if(FunCodeDeft[Counter - H0000INDEX].Attrib.bit.Writable >= ATTRIB_DISP_WRT)
                        {
                            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter - H0000INDEX);
                        }
                        //该循环运行时间要几个ms,需要喂狗
                        ServiceDog();
                    }

                    //根据编码器分辨率设置电子齿轮比 H05_07 H05_09 H05_011 H05_13        
                    //FPGA速度平均值滤波使能H08_22 速度反馈低通滤波截止频率H08_23   速度反馈选择H08_25
                    if((FunCodeUnion.code.MT_EncoderSel & 0x0f0) == 0)    //省线式编码器
                    {
                        FunCodeUnion.code.PL_PosFirCmxLow = 4;             //H05_07 电子齿数比1 分子 L
                        FunCodeUnion.code.PL_PosFirCmxHigh = 0;            //H05_08 电子齿数比1 分子 H
                        FunCodeUnion.code.PL_PosFirCdvLow = 1;             //H05_09 电子齿数比1 分母 L
                        FunCodeUnion.code.PL_PosFirCdvHigh = 0;            //H05_10 电子齿数比1 分母 H

                        FunCodeUnion.code.PL_PosSecCmxLow = 4;             //H05_11 电子齿数比2 分子 L
                        FunCodeUnion.code.PL_PosSecCmxHigh = 0;            //H05_12 电子齿数比2 分子 H
                        FunCodeUnion.code.PL_PosSecCdvLow = 1;             //H05_13 电子齿数比2 分母 L
                        FunCodeUnion.code.PL_PosSecCdvHigh = 0;            //H05_14 电子齿数比2 分母 H 

                        FunCodeUnion.code.GN_SpdFdbFilt_On = 0;            //H08_22 FPGA速度平均值滤波使能  默认为0           
                        FunCodeUnion.code.GN_SpdLpFiltFc = 4000;           //H08_23 速度反馈低通滤波截止频率
                        FunCodeUnion.code.GN_SpdFbSel = 0;                 //H08_25速度反馈选择
                    }
                    else if((FunCodeUnion.code.MT_EncoderSel & 0xf0) == 0x10)    //绝对式编码器
                    {
                        FunCodeUnion.code.PL_PosFirCmxLow = FunCodeUnion.code.MT_EncoderPensL;             //H05_07 电子齿数比1 分子 L
                        FunCodeUnion.code.PL_PosFirCmxHigh = FunCodeUnion.code.MT_EncoderPensH;            //H05_08 电子齿数比1 分子 H
                        FunCodeUnion.code.PL_PosFirCdvLow = 10000;             //H05_09 电子齿数比1 分母 L
                        FunCodeUnion.code.PL_PosFirCdvHigh = 0;            //H05_10 电子齿数比1 分母 H
                        FunCodeUnion.code.PL_PosSecCmxLow = FunCodeUnion.code.MT_EncoderPensL;             //H05_11 电子齿数比2 分子 L
                        FunCodeUnion.code.PL_PosSecCmxHigh = FunCodeUnion.code.MT_EncoderPensH;            //H05_12 电子齿数比2 分子 H
                        FunCodeUnion.code.PL_PosSecCdvLow = 10000;             //H05_13 电子齿数比2 分母 L
                        FunCodeUnion.code.PL_PosSecCdvHigh = 0;            //H05_14 电子齿数比2 分母 H        

                        FunCodeUnion.code.GN_SpdFdbFilt_On = 0;            //H08_22 FPGA速度平均值滤波使能  默认为0           
                        FunCodeUnion.code.GN_SpdLpFiltFc = 4000;           //H08_23 速度反馈低通滤波截止频率        
                        FunCodeUnion.code.GN_SpdFbSel = 1;                 //H08_25速度反馈选择
                    }
                    else if((FunCodeUnion.code.MT_EncoderSel & 0xf0) == 0x20)    //旋变
                    {
                        FunCodeUnion.code.PL_PosFirCmxLow = FunCodeUnion.code.MT_EncoderPensL;             //H05_07 电子齿数比1 分子 L
                        FunCodeUnion.code.PL_PosFirCmxHigh = FunCodeUnion.code.MT_EncoderPensH;            //H05_08 电子齿数比1 分子 H
                        FunCodeUnion.code.PL_PosFirCdvLow = 10000;             //H05_09 电子齿数比1 分母 L
                        FunCodeUnion.code.PL_PosFirCdvHigh = 0;            //H05_10 电子齿数比1 分母 H
                        FunCodeUnion.code.PL_PosSecCmxLow = FunCodeUnion.code.MT_EncoderPensL;             //H05_11 电子齿数比2 分子 L
                        FunCodeUnion.code.PL_PosSecCmxHigh = FunCodeUnion.code.MT_EncoderPensH;            //H05_12 电子齿数比2 分子 H
                        FunCodeUnion.code.PL_PosSecCdvLow = 10000;             //H05_13 电子齿数比2 分母 L
                        FunCodeUnion.code.PL_PosSecCdvHigh = 0;            //H05_14 电子齿数比2 分母 H 
                                
                        FunCodeUnion.code.GN_SpdFdbFilt_On = 4;            //H08_22 FPGA速度平均值滤波使能  默认为0           
                        FunCodeUnion.code.GN_SpdLpFiltFc = 4000;           //H08_23 速度反馈低通滤波截止频率
                        FunCodeUnion.code.GN_SpdFbSel = 1;                 //H08_25速度反馈选择                   
                    }
                    else if((FunCodeUnion.code.MT_EncoderSel & 0xf0) == 0x30)    //光栅尺
                    {
                        FunCodeUnion.code.PL_PosFirCmxLow = 1;             //H05_07 电子齿数比1 分子 L
                        FunCodeUnion.code.PL_PosFirCmxHigh = 0;            //H05_08 电子齿数比1 分子 H
                        FunCodeUnion.code.PL_PosFirCdvLow = 1;             //H05_09 电子齿数比1 分母 L
                        FunCodeUnion.code.PL_PosFirCdvHigh = 0;            //H05_10 电子齿数比1 分母 H

                        FunCodeUnion.code.PL_PosSecCmxLow = 1;             //H05_11 电子齿数比2 分子 L
                        FunCodeUnion.code.PL_PosSecCmxHigh = 0;            //H05_12 电子齿数比2 分子 H
                        FunCodeUnion.code.PL_PosSecCdvLow = 1;             //H05_13 电子齿数比2 分母 L
                        FunCodeUnion.code.PL_PosSecCdvHigh = 0;            //H05_14 电子齿数比2 分母 H        

                        FunCodeUnion.code.GN_SpdFdbFilt_On = 0;            //H08_22 FPGA速度平均值滤波使能  默认为0           
                        FunCodeUnion.code.GN_SpdLpFiltFc = 1000;           //H08_23 速度反馈低通滤波截止频率 
                        FunCodeUnion.code.GN_SpdFbSel = 1;                 //H08_25速度反馈选择                   
                    }

                    #if NONSTANDARD_PROJECT == LINEARMOT
                        FunCodeUnion.code.PL_PosReachValue = 20;            //H05_21 定位完成幅度
                        FunCodeUnion.code.ER_PerrFaultVluLow = 32767;       //H0A10 位置偏差过大故障设定值低16位
                        FunCodeUnion.code.ER_PerrFaultVluHigh = 0;          //H0A11 位置偏差过大故障设定值高16位 
                    #else
                        //根据编码器分辨率设置定位完成幅度H05_21   位置偏差过大故障设定值H0A10 H0A11
                        Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
			            if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;
                        
						Temp_1 = ((Uint64)7L * Temp) / (Uint64)10000L;
                        FunCodeUnion.code.PL_PosReachValue = (Uint16)Temp_1;            //H05_21 定位完成幅度
                         
                        Temp_1 = ((Uint64)32767L * Temp) / (Uint64)10000L;         
                        FunCodeUnion.code.ER_PerrFaultVluLow = (Uint16)Temp_1;                      //H0A10 位置偏差过大故障设定值低16位
                        FunCodeUnion.code.ER_PerrFaultVluHigh = (Uint16)((Uint32)Temp_1 >> 16);     //H0A11 位置偏差过大故障设定值高16位 
                    #endif
                    
                    SaveToEepromSeri(H0200INDEX,(H1800INDEX-1));
                    EepromProcess();
                }
                else     //1：复位中
                {
                    if(EepromProcess() != 1)     //存储完成
                    {
                        //恢复校验字
                        FunCodeUnion.code.EepromCheckWord1 = EEPROM_CHECK_WORD1;
                        FunCodeUnion.code.EepromCheckWord2 = EEPROM_CHECK_WORD2;
                        SaveToEepromSeri(0,1);
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord1));
                        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.EepromCheckWord2));
                        Step++;
                        //复位完成
                        STR_FUNC_Gvar.ManageFunCodeOutput.CANResetFunCode  = 0;
                    }
                }
            }
            break;

        default:
            break;
    }
    return ErrCode;
}
#endif
/*******************************************************************************
  函数名: void AppDataFromEeprom(void)								
  输入:   无
  输出:   0 --成功；否则--错误码
  子函数: 无   
  描述:   将所有可存储在EEPROM中的OD保存入功能码
********************************************************************************/
void AppDataToCurrentValue(void)								
{
    Uint32 *pAdd = &PUB_Null_PointVar32Init;
    Uint32 *pAddStart = &PUB_Null_PointVar32Init;
    Uint32 *pAddEnd = &PUB_Null_PointVar32Init;
    Uint32 indextemp = 0;
    Uint32 Counter = 0;
    Uint32 DataSize = 0;
    Uint32 ComSave = 0;
    
    //至402区
    pAddStart = (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode;//603F
    pAddEnd   = (Uint32 *)&ObjectDictionaryStandard.ProVelMode2.TargetVelocity;//60FF

    for(pAdd = pAddStart;pAdd <= pAddEnd;pAdd++)
    {

        //属性表索引号
        indextemp =(int32)(pAdd - pAddStart);
        ComSave = ObjectDictionaryDefault[indextemp].Attrib.bit.ComSave;
        if(ComSave == ATTRIB_COMSAVE)
        {
            Counter  = GetODFunCodeOffset(pAdd);
            DataSize = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;
            if(DataSize == ATTRIB_TWO_BYTE)
			{
                if((FunCodeUnion.all[Counter]  != (Uint16)(*pAdd)))
                {
                    FunCodeUnion.all[Counter] =(Uint16)(*pAdd);
                }
                else{}
			}
			else
			{
                if((A_SHIFT16_PLUS_B(FunCodeUnion.all[Counter+1],FunCodeUnion.all[Counter])) != (Uint32)(*pAdd))
                {
                    FunCodeUnion.all[Counter] = (Uint16)(*pAdd); 
                    FunCodeUnion.all[Counter + 1] = (Uint16)((*pAdd) >> 16); 
                }
                else{}
			}

        }
    }

    //没有用到的功能码
    for(Counter = Counter+1;Counter<HRsvd00INDEX;Counter++)
    {
        if(FunCodeDeft[Counter - H0000INDEX].Attrib.bit.Writable != ATTRIB_DISP_WRT)
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter - H0000INDEX);
        }
        //该循环运行时间要几个ms,需要喂狗
        //ServiceDog();
    }
}

/*******************************************************************************
  函数名: Uint32 AppDataFromEeprom(void)								
  输入:   无
  输出:   0 --成功；否则--错误码
  子函数: 无   
  描述:   将所有OD恢复默认值
********************************************************************************/
void AppDataToDefultValue(void)								
{
    Uint32 *pAdd = &PUB_Null_PointVar32Init;
    Uint32 *pAddStart = &PUB_Null_PointVar32Init;
    Uint32 *pAddEnd = &PUB_Null_PointVar32Init;
    Uint32 indextemp = 0;
    Uint32 Counter = 0;
    Uint32 DataSize = 0;
    Uint32 ComSave = 0;

    int32 index6065 = 0;
    int32 index6067 = 0;
    int32 index607F = 0;
    int32 index6081 = 0;
    int32 index6083 = 0;
    int32 index6084 = 0;
    int32 index6085 = 0;
    int32 index609901 = 0;
    int32 index609902 = 0;
    int32 index609A = 0;

    Uint64 Temp = 0;
    Uint32 ValueTemp = 0;    
    
    
    //至402区
    pAddStart = (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode;//603F

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.FollowingErrorWindow;//6065--Following Err Window
    index6065 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.PosCtrlFunc1.PositionWindow;//6067--Position Window
    index6067 = (int32)(pAddEnd - pAddStart);
    
    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.ProPosMode.MaxProfileVelocity;//607F---MaxProfileVelocity
    index607F = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileVelocity;//6081--ProfileVelocity
    index6081 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileAcceleration;//6083--ProfileAcceleration
    index6083 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.ProPosMode.ProfileDeceleration;//6084--ProfileDeceleration
    index6084 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.ProPosMode.QuickStopDeceleration;//6085--QuickStopDeceleration
    index6085 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingSpeeds.SpeedDuringSearchForSwitch;//6099-01--SpeedDuringSearchForSwitch
    index609901 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingSpeeds.SpeedDuringSearchForZero;//6099-02--SpeedDuringSearchForZero
    index609902 = (int32)(pAddEnd - pAddStart);

    pAddEnd = (Uint32 *)&ObjectDictionaryStandard.HomingMode.HomingAcceleration;//609A--HomingAcceleration
    index609A = (int32)(pAddEnd - pAddStart);
    
    pAddEnd   = (Uint32 *)&ObjectDictionaryStandard.ProVelMode2.TargetVelocity;//60FF
    for(pAdd = pAddStart;pAdd <= pAddEnd;pAdd++)
    {
        //属性表索引号
        indextemp =(int32)(pAdd - pAddStart);
        ComSave = ObjectDictionaryDefault[indextemp].Attrib.bit.ComSave;
        
        if(ComSave == ATTRIB_COMSAVE)
        {
            Counter  = GetODFunCodeOffset(pAdd);
            DataSize = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;

            if(DataSize <= ATTRIB_TWO_BYTE)
			{
                if((FunCodeUnion.all[Counter]  != (Uint16)ObjectDictionaryDefault[indextemp].Value))
                {
                    FunCodeUnion.all[Counter] =(Uint16)ObjectDictionaryDefault[indextemp].Value;
                }
                else{}
			}
			else
			{
                if(indextemp==index6065)
                {
                    #if NONSTANDARD_PROJECT == LINEARMOT
                        //6065
                        ValueTemp = 32767;
                    #else
                        //根据编码器分辨率设置位置偏差过大阈值
                        Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
                        if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;
                        ValueTemp = (Uint32)(((Uint64)32767L * Temp) / (Uint64)10000L);
                    #endif
                }
                else if(indextemp==index6067)
                {
                    #if NONSTANDARD_PROJECT == LINEARMOT
                        //6067
                        ValueTemp = 20;
                    #else
                        //根据编码器分辨率设置定位完成幅度6067
                        Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
                        if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;
                        ValueTemp = (Uint32)(((Uint64)7L * Temp) / (Uint64)10000L);
                    #endif
                }
                else if(indextemp==index607F)
                {
                    ValueTemp = (Uint32)FunCodeUnion.code.MT_MaxSpd;

                    Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
                	if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;

                    ValueTemp = (Uint32)(((Uint64)ValueTemp * Temp) / (Uint64)60L);
                }
                else if((indextemp==index6081)||(indextemp==index609901)||(indextemp==index609902))
                {
                    Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
                	if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;
					ValueTemp = (Uint32)(((Uint64)100L * Temp) / (Uint64)60L);

                    if(indextemp==index609902)ValueTemp = ValueTemp/10L;
                }
                else if((indextemp==index6083)||(indextemp==index6084)||(indextemp==index6085)||(indextemp==index609A))
                {
                    Temp = (Uint64)((Uint32)FunCodeUnion.code.MT_EncoderPensH << 16) + FunCodeUnion.code.MT_EncoderPensL;
                	if(0 == (FunCodeUnion.code.MT_EncoderSel & 0x0f0)) Temp = Temp << 2;
					ValueTemp = (Uint32)(((Uint64)100000L * Temp) / (Uint64)60L);
                }
                else
                {
                    ValueTemp = (Uint32)ObjectDictionaryDefault[indextemp].Value;
                }

                if((A_SHIFT16_PLUS_B(FunCodeUnion.all[Counter+1],FunCodeUnion.all[Counter])) != ValueTemp)
                {
                    FunCodeUnion.all[Counter] = (Uint16)ValueTemp; 
                    FunCodeUnion.all[Counter + 1] = (Uint16)(ValueTemp >> 16); 
                }
                else{}
			}

        }
        //该循环运行时间要几个ms,需要喂狗
        ServiceDog();

    }

    //没有用到的功能码
    for(Counter = Counter+1;Counter<HRsvd00INDEX;Counter++)
    {
        if(FunCodeDeft[Counter - H0000INDEX].Attrib.bit.Writable != ATTRIB_DISP_WRT)
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter - H0000INDEX);
        }
        //该循环运行时间要几个ms,需要喂狗
        ServiceDog();
    }

}
/*******************************************************************************
  函数名: Uint32 AppDataFromEeprom(void)								
  输入:   无
  输出:   0 --成功；否则--错误码
  子函数: 无   
  描述:   从EEPROM读取402对象已经存储的值，然后存入Od
********************************************************************************/
void ReadAppDataFromEeprom(void)								
{
    Uint32 *pAdd = &PUB_Null_PointVar32Init;
    Uint32 *pAddStart = &PUB_Null_PointVar32Init;
    Uint32 *pAddEnd = &PUB_Null_PointVar32Init;
    Uint32 indextemp = 0;
    Uint32 Counter = 0;
    Uint32 DataSize = 0;
    Uint32 ComSave = 0;

    //至402区
    pAddStart = (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode;//603F
    pAddEnd   = (Uint32 *)&ObjectDictionaryStandard.ProVelMode2.TargetVelocity;//60FF
    for(pAdd = pAddStart;pAdd <= pAddEnd;pAdd++)
    {
        //属性表索引号
        indextemp =(int32)(pAdd - pAddStart);
        ComSave = ObjectDictionaryDefault[indextemp].Attrib.bit.ComSave;
        
        if(ComSave == ATTRIB_COMSAVE)
        {
            Counter   = GetODFunCodeOffset(pAdd);
            DataSize  = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;
            if(DataSize <= ATTRIB_TWO_BYTE)
			{
                *pAdd = FunCodeUnion.all[Counter];
			}
			else
			{
                *pAdd = ((Uint32)FunCodeUnion.all[Counter+1] << 16 )+((Uint32)FunCodeUnion.all[Counter]);
			}
        }
        else//加载默认值
        {
            *pAdd = ObjectDictionaryDefault[indextemp].Value;
        }
        //该循环运行时间要几个ms,需要喂狗
        ServiceDog();
    }

    //没有用到的功能码
    for(Counter = Counter+1;Counter<HRsvd00INDEX;Counter++)
    {
        if(FunCodeDeft[Counter - H0000INDEX].Attrib.bit.Writable != ATTRIB_DISP_WRT)
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter - H0000INDEX);
        }
        //该循环运行时间要几个ms,需要喂狗
        ServiceDog();
    }
}

/*******************************************************************************
  函数名: 
  输入:   无 
  输出:   无 
  子函数: 无
  描述：
********************************************************************************/
Uint16 LimitCheck(Uint16 IndexTemp, Uint32 DataInput)
{
    Uint32 LmtTemp = 0;
    Uint32 DataType = 0;
    Uint16 ErrCode = 0;
   
    DataType = ObjectDictionaryDefault[IndexTemp].Attrib.bit.DataType;
    //取得下限值                    
    LmtTemp = ObjectDictionaryDefault[IndexTemp].LowerLmt;

	switch(DataType)
    {
        case ATTRIB_TYPE_UINT8:
            if((Uint8)DataInput < (Uint8)LmtTemp)   ErrCode=COMM_VALUE_TOO_SMALL;
			break;
            
        case ATTRIB_TYPE_INT8://此处必须注意，若直接用int8比较，无法得到正确结果
            if((int8)((int32)DataInput) < (int8)((int32)LmtTemp)) ErrCode=COMM_VALUE_TOO_SMALL;
            break;

        case ATTRIB_TYPE_UINT16:
            if((Uint16)DataInput < (Uint16)LmtTemp) ErrCode=COMM_VALUE_TOO_SMALL;
			break;

        case ATTRIB_TYPE_INT16:
            if((int16)DataInput < (int16)LmtTemp)   ErrCode=COMM_VALUE_TOO_SMALL;
			break;

        case ATTRIB_TYPE_UINT32:
            if((Uint32)DataInput < (Uint32)LmtTemp) ErrCode=COMM_VALUE_TOO_SMALL;
			break;

        case ATTRIB_TYPE_INT32:
            if((int32)DataInput < (int32)LmtTemp)   ErrCode=COMM_VALUE_TOO_SMALL;
			break;

			default:
			break;
    }
        
    //取得上限值
    LmtTemp = ObjectDictionaryDefault[IndexTemp].UpperLmt;
    
    switch(DataType)
    {
        case ATTRIB_TYPE_UINT8:
            if((Uint8)DataInput > (Uint8)LmtTemp)   ErrCode=COMM_VALUE_TOO_GREAT;
			break;
            
        case ATTRIB_TYPE_INT8:
            if((int8)((int32)DataInput) > (int8)((int32)LmtTemp)) ErrCode=COMM_VALUE_TOO_GREAT;
            break;
            
        case ATTRIB_TYPE_UINT16:
            if((Uint16)DataInput > (Uint16)LmtTemp) ErrCode=COMM_VALUE_TOO_GREAT;
			break;
            
        case ATTRIB_TYPE_INT16:
            if((int16)DataInput > (int16)LmtTemp)   ErrCode=COMM_VALUE_TOO_GREAT;
			break;
            
        case ATTRIB_TYPE_UINT32:
            if((Uint32)DataInput > (Uint32)LmtTemp) ErrCode=COMM_VALUE_TOO_GREAT;
			break;
            
        case ATTRIB_TYPE_INT32:
            if((int32)DataInput > (int32)LmtTemp)   ErrCode=COMM_VALUE_TOO_GREAT;
			break;
            
			default:
			break;
    } 

    return ErrCode;
}
/*******************************************************************************
  函数名: 
  输入:   EndAddr ----属性为RW或者WO的OD地址
  输出:   无 
  子函数: 无
  描述:   获得OD存储到FunCode时，在功能码结构体表中的偏置
********************************************************************************/
Uint16 GetODFunCodeOffset(Uint32 *pAddEnd)
{
    Uint32 *pAdd = &PUB_Null_PointVar32Init;
    Uint32 *pAddStart = &PUB_Null_PointVar32Init;
    Uint32 *pAddEndSearch = &PUB_Null_PointVar32Init;
    Uint32 indextemp = 0;
    Uint16 Counter = 0;
    Uint32 DataSize = 0;
    Uint32 ComSave = 0;
    
    //至402区
    pAddStart = (Uint32 *)&ObjectDictionaryStandard.ComEntryErrCode.ErrorCode;//603F
    pAddEndSearch = pAddEnd - 1;

    for(pAdd = pAddStart,Counter=(H1C00INDEX-1);pAdd <= pAddEndSearch;pAdd++)
    {
        //属性表索引号
        indextemp =(int32)(pAdd - pAddStart);
        ComSave = ObjectDictionaryDefault[indextemp].Attrib.bit.ComSave;
        DataSize = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;
        
        if(ComSave == ATTRIB_COMSAVE)
        {
            if(DataSize == ATTRIB_FOUR_BYTE)//32bit占用2个功能码
            {
                //当前碰到校验字
                if(((Counter + 1)== (H1D00INDEX - 1))||((Counter + 1) == (HRsvd00INDEX - 1)))
                {
                    FunCodeUnion.all[Counter+1] = GetCodeDftValue(Counter+1- H0000INDEX);
                    Counter = Counter + 3;//加校验字，占用3个功能码
                }

                //下一个是校验字
                else if(((Counter + 2)== (H1D00INDEX - 1))||((Counter + 2) == (HRsvd00INDEX - 1)))
                {
                    FunCodeUnion.all[Counter+1] = GetCodeDftValue(Counter+1- H0000INDEX);
                    FunCodeUnion.all[Counter+2] = GetCodeDftValue(Counter+2- H0000INDEX);
                    Counter = Counter + 4;//加校验字，占用4个功能码
                }
                else
                {
                    Counter = Counter + 2;
                }
            }
            else//只占用一个功能码
            {
                //当前碰到校验字
                if(((Counter +1)== (H1D00INDEX - 1))||((Counter+1) == (HRsvd00INDEX - 1)))
                {
                    FunCodeUnion.all[Counter+1] = GetCodeDftValue(Counter+1- H0000INDEX);
                    Counter = Counter + 2;//加校验字，占用2个功能码
                }
                else
                {
                    Counter = Counter + 1;
                }
            }
        }
    }
    
    Counter = Counter + 1;
    indextemp =(int32)(pAddEnd - pAddStart);
    DataSize = ObjectDictionaryDefault[indextemp].Attrib.bit.DataSize;

    if(DataSize==ATTRIB_FOUR_BYTE)
    {
        if((Counter == (H1D00INDEX - 1))||(Counter == (HRsvd00INDEX - 1)))
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter- H0000INDEX);
            Counter = Counter + 1;//跳过校验字
        }
        else if(((Counter +1)== (H1D00INDEX - 1))||((Counter +1) == (HRsvd00INDEX - 1)))
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter- H0000INDEX);
            FunCodeUnion.all[Counter+1] = GetCodeDftValue(Counter+1- H0000INDEX);
            Counter = Counter + 2;//跳过当前功能码和校验字
        }
    }
    else
    {
        if((Counter == (H1D00INDEX - 1))||(Counter == (HRsvd00INDEX - 1)))
        {
            FunCodeUnion.all[Counter] = GetCodeDftValue(Counter- H0000INDEX);
            Counter = Counter + 1;//跳过校验字
        }
    } 
    return Counter;
}
/*******************************************************************************
  函数名: void ComParaMonitor(void)
  输入:   
  输出:    
  子函数: 
  描述:   
********************************************************************************/
//Uint32 pComData[1] ={0};
int32 ComParaMonitor(Uint16 MonParaHigh,Uint16 MonParaLow)
{
    Uint32 Temp = 0;
    Uint16 Index = 0;
    Uint8  Subindex = 0;
    Uint32 Errcode = 0;
    Uint8 pDataSize =0;
    int32 MonParaValue = 0;
    Uint32 pComData =0;

    Temp = ((Uint32)MonParaHigh<<16) + MonParaLow;

    if(Temp == 0)return 0;//没有参数需要监控
    
    Index    = (Uint16)((Temp &0x00FFFF00)>>8);
    Subindex = (Uint16)(Temp &0x000000FF);
    
    Errcode = C_Read_FunCode(Index, Subindex, &pDataSize, &pComData);

	if(Errcode !=0)return 0;//参数不存在

    pDataSize = pDataSize>>3; 
    
    if(pDataSize == ATTRIB_ONE_BYTE)
    {
        MonParaValue = (int32)(pComData & 0x000000FF);
    }
    else if(pDataSize == ATTRIB_TWO_BYTE)
    {
        MonParaValue = (int32)(pComData & 0x0000FFFF);
    }
    else if(pDataSize == ATTRIB_FOUR_BYTE)
    {
        MonParaValue = (int32)pComData;
    }
    return MonParaValue;
}
/*******************************************************************************
  函数名: void ComParaMonitor(void)
  输入:   
  输出:    
  子函数: 
  描述:   
********************************************************************************/
void GetComValue(void)
{
    int32 Temp = 0;

    Temp = ComParaMonitor(FunCodeUnion.code.ComMoniPara1IndexHigh,FunCodeUnion.code.ComMoniPara1IndexLow);
    FunCodeUnion.code.ComMoniPara1Low = (Uint16)(Temp & 0x0000FFFF);
    FunCodeUnion.code.ComMoniPara1High = (Uint16)(Temp >> 16);

    Temp = ComParaMonitor(FunCodeUnion.code.ComMoniPara2IndexHigh,FunCodeUnion.code.ComMoniPara2IndexLow);
    FunCodeUnion.code.ComMoniPara2Low = (Uint16)(Temp & 0x0000FFFF);
    FunCodeUnion.code.ComMoniPara2High = (Uint16)(Temp >> 16);


    Temp = ComParaMonitor(FunCodeUnion.code.ComMoniPara3IndexHigh,FunCodeUnion.code.ComMoniPara3IndexLow);
    FunCodeUnion.code.ComMoniPara3Low = (Uint16)(Temp & 0x0000FFFF);
    FunCodeUnion.code.ComMoniPara3High = (Uint16)(Temp >> 16);


    Temp = ComParaMonitor(FunCodeUnion.code.ComMoniPara4IndexHigh,FunCodeUnion.code.ComMoniPara4IndexLow);
    FunCodeUnion.code.ComMoniPara4Low = (Uint16)(Temp & 0x0000FFFF);
    FunCodeUnion.code.ComMoniPara4High = (Uint16)(Temp >> 16);

}
/********************************* END OF FILE *********************************/
