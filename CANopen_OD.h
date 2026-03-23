
#ifndef CANOPENOBJECTDICTIONARY_H
#define CANOPENOBJECTDICTIONARY_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif 
/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */

#include "PUB_GlobalPrototypes.h"

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */


//属性相关宏定义
//bit.AccessType   BIT:0-1    00:RW   01:WO    10:RO
#define ATTRIB_RW                           0
#define ATTRIB_WO                           1
#define ATTRIB_RO                           2
#define ATTRIB_RSVD                         3
//bit.DataType   BIT:2-4     010:有符号8位  011:有符号16位  100:有符号32位
//                           101:无符号8位  110:无符号16位  111:无符号32位 
#define ATTRIB_TYPE_INT8                    2 //0x02
#define ATTRIB_TYPE_INT16                   3  //0x03 
#define ATTRIB_TYPE_INT32                   4 //0x04 
#define ATTRIB_TYPE_UINT8                   5 //0x05
#define ATTRIB_TYPE_UINT16                  6 //0x06
#define ATTRIB_TYPE_UINT32                  7 //0x07
//bit.DataSize   BIT:5-8    0001:1个字节  0010:2个字节  0100:4个字节  1000:8个字节
#define ATTRIB_ONE_BYTE                     1 //0x01
#define ATTRIB_TWO_BYTE                     2 //0x02
#define ATTRIB_FOUR_BYTE                    4 //0x04
#define ATTRIB_EIGHT_BYTE                   8 //0x08
//bit.PDOMapping BIT:9      0-UNMapping   1-Mapping
#define ATTRIB_UNMAPPING                    0 
#define ATTRIB_MAPPING                      1 //0x01
//bit.ODBelong BIT:10-11      00-BOTH   01-ECT_ONLT  10-CANOPEN_ONLY
#define ATTRIB_BOTH                         0 
#define ATTRIB_CANOPEN_ONLY                 1
#define ATTRIB_ECT_ONLY                     2
//bit.ComSave BIT:12      0-UNCOMSAVE   1-COMSAVE
#define ATTRIB_UNCOMSAVE                    0
#define ATTRIB_COMSAVE                      1


/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */
//extern Uint16 FpgaSoftVersion;
//203FH
typedef struct{

    Uint32 ServorErrorCode;//203F  Uint32 暂时将203F放在这里 RO

}STR_SERVORERRORCODE;

//6502H
typedef struct{

    Uint32 PP:1;//0
    Uint32 VL:1;//1ww
    Uint32 PV:1;//2
    Uint32 PT:1;//3
    Uint32 Rsvd4:1;//4
    Uint32 HM:1;//5
    Uint32 IP:1;//6
    Uint32 CSP:1;//7
    Uint32 CSV:1;//8
    Uint32 CST:1;//9
    Uint32 Rsvd10:1;//10
    Uint32 Rsvd11:1;//11
    Uint32 Rsvd12:1;//12
    Uint32 Rsvd13:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 Rsvd15:1;//15
    Uint32 RsvdHigWord:16;//

}STR_SUPPORTEDDRIVEMODES_BIT;

typedef union{
    Uint32 all;
    volatile STR_SUPPORTEDDRIVEMODES_BIT bit;
}UNI_SUPPORTEDDRIVEMODES;


typedef struct{
    volatile UNI_SUPPORTEDDRIVEMODES SupportedDriveModes;
    
}STR_COMMONENTRYSUPPORTEDDRIVEMODES;

//603Fh     
typedef struct{
    Uint32 ErrorCode;//603Fh    Uint16

}STR_COMMONENTRYERRORCODE;


//60FDh
typedef struct{

    Uint32 NegLimitSwitch:1;//0
    Uint32 PosLimitSwitch:1;//1
    Uint32 HomeSwitch:1;//2
    Uint32 Rsvd3:1;//3
    Uint32 Rsvd4:1;//4
    Uint32 Rsvd5:1;//5
    Uint32 Rsvd6:1;//6
    Uint32 Rsvd7:1;//7
    Uint32 Rsvd8:1;//8
    Uint32 Rsvd9:1;//9
    Uint32 Rsvd10:1;//10
    Uint32 Rsvd11:1;//11
    Uint32 Rsvd12:1;//12
    Uint32 Rsvd13:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 Rsvd15:1;//15
	Uint32 DI1:1;//16
	Uint32 DI2:1;//17
	Uint32 DI3:1;//18
	Uint32 DI4:1;//19
	Uint32 DI5:1;//20
	Uint32 DI6:1;//21
	Uint32 DI7:1;//22
	Uint32 DI8:1;//23
	Uint32 DI9:1;//24
    Uint32 Rsvd25:1;//25
    Uint32 Rsvd26:1;//26
    Uint32 Rsvd27:1;//27
    Uint32 Rsvd28:1;//28
    Uint32 Rsvd29:1;//29
    Uint32 Rsvd30:1;//30
    Uint32 Rsvd31:1;//31
}STR_DIGITALINPUTS_BIT;

typedef union{
    Uint32 all;
    volatile STR_DIGITALINPUTS_BIT bit;
}UNI_DIGITALINPUTS;


//60FEh-01h
typedef struct{
    Uint32 BrakeEnable:1;//0
    Uint32 Rsvd1:1;//1
    Uint32 Rsvd2:1;//2
    Uint32 Rsvd3:1;//3
    Uint32 Rsvd4:1;//4
    Uint32 Rsvd5:1;//5
    Uint32 Rsvd6:1;//6
    Uint32 Rsvd7:1;//7
    Uint32 Rsvd8:1;//8
    Uint32 Rsvd9:1;//9
    Uint32 Rsvd10:1;//10
    Uint32 Rsvd11:1;//11
    Uint32 Rsvd12:1;//12
    Uint32 Rsvd13:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 Rsvd15:1;//15
	Uint32 DO1:1;
	Uint32 DO2:1;
	Uint32 DO3:1;
    Uint32 RsvdHigWord:13;//
}STR_PHYSICALOUTPUTS_BIT;

typedef union{
    Uint32 all;
    volatile STR_PHYSICALOUTPUTS_BIT bit;
}UNI_PHYSICALOUTPUTS;

typedef struct{
    volatile UNI_PHYSICALOUTPUTS PhysicalOutput;  //60FEh
    
}STR_DIGITALOUTPUTS01;


//60FEh-02
typedef struct{
    Uint32 SetBrake:1;//0
    Uint32 Rsvd1:1;//1
    Uint32 Rsvd2:1;//2
    Uint32 Rsvd3:1;//3
    Uint32 Rsvd4:1;//4
    Uint32 Rsvd5:1;//5
    Uint32 Rsvd6:1;//6
    Uint32 Rsvd7:1;//7
    Uint32 Rsvd8:1;//8
    Uint32 Rsvd9:1;//9
    Uint32 Rsvd10:1;//10
    Uint32 Rsvd11:1;//11
    Uint32 Rsvd12:1;//12
    Uint32 Rsvd13:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 Rsvd15:1;//15
	Uint32 DO1:1;
	Uint32 DO2:1;
	Uint32 DO3:1;
    Uint32 RsvdHigWord:13;//
}STR_BITMASK_BIT;

typedef union{
    Uint32 all;
    volatile STR_BITMASK_BIT bit;
}UNI_BITMASK;

typedef struct{
    volatile UNI_BITMASK BitMask;  //60FEh
    
}STR_DIGITALOUTPUTS02;

typedef struct{
    Uint32 NumberOfEntries;
    STR_DIGITALOUTPUTS01 DOFunc;  //60FE-01h
    STR_DIGITALOUTPUTS02 DOEnable;  //60FE-02h
    
}STR_DIGITALOUTPUTS;



typedef struct{
    volatile UNI_DIGITALINPUTS DigitalInputs;//60FDh
    volatile STR_DIGITALOUTPUTS DigitalOutputs;  //60FEh

}STR_COMMONENTRYDIDO;

//6040h
typedef struct{

    Uint32 SwitchOn:1;//0
    Uint32 VoltageEnable:1;//1
    Uint32 QuickStop:1;//2
    Uint32 OperationEnable:1;//3
    Uint32 OperationModeSpecific:3;//4
    Uint32 FaultReset:1;//7
    Uint32 Halt:1;//8
    Uint32 Rsvd9:1;//9
    Uint32 Rsvd10:1;//10
    Uint32 Rsvd11:1;//11
    Uint32 Rsvd12:1;//12
    Uint32 Rsvd13:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 Rsvd15:1;//15
    Uint32 RsvdHigWord:16;//


}STR_CONTROLWORD_BIT;

typedef union{
    Uint32 all;
    volatile STR_CONTROLWORD_BIT bit;
}UNI_CONTROLWORD;


//6041h
typedef struct{

    Uint32 RdyToSwitchOn:1;//0
    Uint32 SwitchedOn:1;//1
    Uint32 OperationEnabled:1;//2
    Uint32 Fault:1;//3
    Uint32 VoltageEnabled:1;//4
    Uint32 QuickStop:1;//5
    Uint32 SwitchOnDisabled:1;//6
    Uint32 Warning:1;//7
    Uint32 Rsvd8:1;//8
    Uint32 Remote:1;//9
    Uint32 TargetReached:1;//10
    Uint32 InternalLimitActive:1;//11
    Uint32 OperationModeSpecific1:1;//12
    Uint32 OperationModeSpecific2:1;//13
    Uint32 Rsvd14:1;//14
    Uint32 HomeFind:1;//15---参考施耐德，本次上电已进行过原点复归，参考点已找到
    Uint32 RsvdHigWord:16;//


}STR_STATUSWORD_BIT;

typedef union{
    Uint32 all;
    volatile STR_STATUSWORD_BIT bit;
}UNI_STATUSWORD;



typedef struct{

    volatile UNI_CONTROLWORD ControlWord;//6040h  Uint16
    volatile UNI_STATUSWORD StatusWord;//6041h   Uint16
    Uint32 QuickStopOptionCode;//605Ah  int16
    Uint32 HaltOptionCode; //605Dh
    Uint32 ModesOfOperation;//6060h      int8
    Uint32 ModesOfOperationDisplay;//6061h   int8
}STR_DEVICECONTROL;

typedef struct{

    //Gear Ratio//6091h
    Uint32 NumberOfEntries;
    Uint32 MotorRev;//
    Uint32 ShaftRev;//

}STR_GEARRATIO;


typedef struct{

    //PositionFactor//6093h
    Uint32 NumberOfEntries;
    Uint32 Numerator;//
    Uint32 FeedConstant;//

}STR_POSITIONFACTOR;


typedef struct{

    //VelocityEncoderFactor;//6094h
    Uint32 NumberOfEntries;//
    Uint32 Numerator;//
    Uint32 Divisor;//


}STR_VELOCITYENCODERFACTOR;



typedef struct{

    //VelocityFactor1;//6095h
    Uint32 NumberOfEntries;//
    Uint32 Numerator;//
    Uint32 Divisor;//


}STR_VELOCITYFACTOR1;

typedef struct{

    //AccelerationFactor;//6097h
    Uint32 NumberOfEntries;//
    Uint32 Numerator;//
    Uint32 Divisor;//


}STR_ACCELERATIONFACTOR;


typedef struct{

    volatile STR_GEARRATIO GearRatio;//6091h
    volatile STR_POSITIONFACTOR PositionFactor;//6093h
    volatile STR_VELOCITYENCODERFACTOR VelocityEncoderFactor;//6094h
    volatile STR_VELOCITYFACTOR1 VelocityFactor1;//6095h
    volatile STR_ACCELERATIONFACTOR AccelerationFactor;//6097h

}STR_FACTORGROUP;


//607Dh
typedef struct{

    //SoftwarePositionLimit;
    Uint32 NumberOfEntries;//
    Uint32 MinPositionLimit;//  int32
    Uint32 MaxPositionLimit;//  int32

}STR_SOFTWAREPOSITIONLIMIT;



typedef struct{

    Uint32 TargetPosition;//607Ah    int32
    Uint32 HomeOffset;//607Ch   int32
    volatile STR_SOFTWAREPOSITIONLIMIT SoftwarePositionLimit;//607Dh
    Uint32 Polarity;//607Eh  Uint8
    Uint32 MaxProfileVelocity;//607Fh
    Uint32 ProfileVelocity;//6081h
    Uint32 ProfileAcceleration;//6083h
    Uint32 ProfileDeceleration;//6084h
    Uint32 QuickStopDeceleration;//6085h
    Uint32 MotionProfileType;//6086h     int16

}STR_PROFILEPOSITIONMODE;


typedef struct{

    Uint32 ToqSlope;//6087h    Uint32

}STR_PROFILETORQUEMODE2;


typedef struct{

    Uint32 MaxAcceleration;//60C5h
    Uint32 MaxDeceleration;//60C6h

}STR_MAXACCLIMIT;


typedef struct{
    //HomingSpeeds;//6099h
    Uint32 NumberOfEntries;//
    Uint32 SpeedDuringSearchForSwitch;//
    Uint32 SpeedDuringSearchForZero;//
}STR_HOMINGSPEED;


typedef struct{

    
    Uint32 HomingMethod;//6098h     int8
    volatile STR_HOMINGSPEED HomingSpeeds;//6099h
    Uint32 HomingAcceleration;//609Ah
}STR_HOMINGMODE;


typedef struct{

    Uint32 PositionDemandValue;//6062h      int32
    Uint32 PositionActualValueInc;//6063h   int32
    Uint32 PositionActualValue;//6064h      int32
    Uint32 FollowingErrorWindow;//6065h
	Uint32 FollowingErrorTimeOut        ;//6066h   Uint16
    Uint32 PositionWindow;//6067h           Uint16
    Uint32 PositionWindowTime;//6068h       Uint16

}STR_POSITIONCONTROLFUNCTION1;

typedef struct{

    Uint32 FollowingErrorActualValue;//60F4h   int32
    Uint32 PositionDemandValueInc;//60FCh     int32

}STR_POSITIONCONTROLFUNCTION2;



typedef struct{

    Uint32 VelocityDemandValue;//606Bh      int32
    Uint32 VelocityActualValue;//606Ch      int32
    Uint32 VelocityWindow;//606Dh          Uint16
    Uint32 VelocityWindowTime;//606Eh      Uint16
    Uint32 VelocityThreshold;//606Fh       Uint16
    Uint32 VelocityThresholdTime;//6070h   Uint16

}STR_PROFILEVELOCITYMODE1;

typedef struct{

    Uint32 TargetVelocity;//60FFh          int32

}STR_PROFILEVELOCITYMODE2;

//插补数据记录
typedef struct{
	Uint32 NumberOfEntrys; //00 点数，暂时固定为1个
	Uint32 SetPoint1st; //01 第1点
	
}STR_INTERPLTRECORD; 

//插补周期设定period = InterpltTimUnits * 10(InterpltTimIndex) second
//默认InterpltTimIndex=-3,即周期是10(-3)=1ms
typedef struct
{
	Uint32 NumberOfEntrys; 
	Uint32 InterpltTimUnits;  //默认1
	Uint32 InterpltTimIndex;  //默认-3
}STR_INTERPLTPERIOD;

//插补模式
typedef struct{
//	Uint32 InterpltModSelect;  //60C0 插补模式,暂时只支持直线
	STR_INTERPLTRECORD InterpltDataRecord; //60C1 插补数据	
	STR_INTERPLTPERIOD InterpltPeriod;     //60C2 插补周期
	
}STR_INTERPOLATIONMODE1;

//插补模式
typedef struct{
//	Uint32 InterpltModSelect;  //60C0 插补模式,暂时只支持直线
	Uint16             mappedRPDOx;         //60C1映射的PDO
	
}STR_INTERPOLATIONMODE2;


typedef struct{
	Uint32 TargetToq;       //6071 int16  Target Toruqe
	Uint32 MaxToq;          //6072 Uint16 Max Torque
	Uint32 ToqDemandValue;  //6074 int16  Torque demand value
	Uint32 ToqActualValue;  //6077 int16  Torque actual value
}STR_PROFILETORQUEMODE1;



typedef struct{
	Uint32 PosOff;  //60B0 int32  Position offset
	Uint32 VelOff;  //60B1 int32  Velocity offset
	Uint32 ToqOff;  //60B2 int32  Torque offset
}STR_CSTOFFSET;


typedef struct{
	Uint32 Func;  //60B8 Uint16  Func
	Uint32 Status;  //60B9 Uint16  Status
	Uint32 Pb1PosValatPE;  //60BA int32  pb1 position value at positive edge 
	Uint32 Pb1PosValatNE;  //60BB int32  pb1 position value at negative edge 
	Uint32 Pb2PosValatPE;  //60BC int32  pb2 position value at positive edge 
	Uint32 Pb2PosValatNE;  //60BD int32  pb2 position value at negative edge 
}STR_TOUCHPROBE;

typedef struct{

    Uint32 TP1PosEdgeCnt       ;//60D5h  Uint16 
    Uint32 TP1NegEdgeCnt       ;//60D6h  Uint16
    Uint32 TP2PosEdgeCnt       ;//60D7h  Uint16 
    Uint32 TP2NegEdgeCnt       ;//60D8h  Uint16

}STR_TOUCHPROBE2;

typedef struct{

    Uint32 MaxPosToq;//60E0h Uint16 Forward Direction Torque Limit Value
    Uint32 MaxNegToq;//60E1h Uint16 Reverse Direction Torque Limit Value

}STR_MAXTOQLIMIT;


typedef struct{
	Uint32 NumberOfEntrys; //Uint8 Supported Homing Method
	Uint32 Method_1; //Uint16 Supported Homing Method
	Uint32 Method_2; //Uint16 Supported Homing Method
	Uint32 Method_3; //Uint16 Supported Homing Method
	Uint32 Method_4; //Uint16 Supported Homing Method
	Uint32 Method_5; //Uint16 Supported Homing Method
	Uint32 Method_6; //Uint16 Supported Homing Method
	Uint32 Method_7; //Uint16 Supported Homing Method
	Uint32 Method_8; //Uint16 Supported Homing Method
	Uint32 Method_9; //Uint16 Supported Homing Method
	Uint32 Method_10; //Uint16 Supported Homing Method
	Uint32 Method_11; //Uint16 Supported Homing Method
	Uint32 Method_12; //Uint16 Supported Homing Method
	Uint32 Method_13; //Uint16 Supported Homing Method
	Uint32 Method_14; //Uint16 Supported Homing Method
	Uint32 Method_17; //Uint16 Supported Homing Method
	Uint32 Method_18; //Uint16 Supported Homing Method
	Uint32 Method_19; //Uint16 Supported Homing Method
	Uint32 Method_20; //Uint16 Supported Homing Method
	Uint32 Method_21; //Uint16 Supported Homing Method
	Uint32 Method_22; //Uint16 Supported Homing Method
	Uint32 Method_23; //Uint16 Supported Homing Method
	Uint32 Method_24; //Uint16 Supported Homing Method
	Uint32 Method_25; //Uint16 Supported Homing Method
	Uint32 Method_26; //Uint16 Supported Homing Method
	Uint32 Method_27; //Uint16 Supported Homing Method
	Uint32 Method_28; //Uint16 Supported Homing Method
	Uint32 Method_29; //Uint16 Supported Homing Method
	Uint32 Method_30; //Uint16 Supported Homing Method
	Uint32 Method_33; //Uint16 Supported Homing Method
	Uint32 Method_34; //Uint16 Supported Homing Method
	Uint32 Method_35; //Uint16 Supported Homing Method
}STR_SURHOMING; 


typedef struct{

    STR_SURHOMING SurHoming;//60E3h 
    Uint32 ActualPosCalMethod;//60E6h Uint8 Actual position calcuation method

}STR_HOMANDPOSCALMETHOD;


typedef struct{
    volatile STR_COMMONENTRYSUPPORTEDDRIVEMODES  ComEntrySurpDrivMod;//6502
    volatile STR_COMMONENTRYERRORCODE            ComEntryErrCode;    //603F
    volatile STR_DEVICECONTROL                   DeviceControl;      //6040~6061
    volatile STR_POSITIONCONTROLFUNCTION1        PosCtrlFunc1;       //6062~6068
	volatile STR_PROFILEVELOCITYMODE1            ProVelMode1;        //606B~6070
	volatile STR_PROFILETORQUEMODE1              ProToqMode1;        //6071~6077
    volatile STR_PROFILEPOSITIONMODE             ProPosMode;         //607A~6086
    volatile STR_PROFILETORQUEMODE2              ProToqMode2;        //6087
    volatile STR_FACTORGROUP                     FactorGroup;        //6091~6097
    volatile STR_HOMINGMODE                      HomingMode;         //6098~609A
    volatile STR_CSTOFFSET                       CstOffset;          //60B0~60B2
    volatile STR_TOUCHPROBE                      TouchProbe;         //60B8~60BD
	volatile STR_INTERPOLATIONMODE1              InterpltPosMode1;   //60C1 60C2
    volatile STR_MAXACCLIMIT                     MaxAcc;             //60C5 60C6
    volatile STR_TOUCHPROBE2                     TouchProbe2         ;//60D5~60D8
    volatile STR_MAXTOQLIMIT                     MaxToqLmt;          //60E0~60E1
    volatile STR_HOMANDPOSCALMETHOD              HomandPosCalMethod; //60E3~60E6
    volatile STR_POSITIONCONTROLFUNCTION2        PosCtrlFunc2;       //60F4~60FC
    volatile STR_COMMONENTRYDIDO                 ComEntryDIDO;       //60FD 60FE
    volatile STR_PROFILEVELOCITYMODE2            ProVelMode2;        //60FF
    volatile STR_INTERPOLATIONMODE2              InterpltPosMode2;   //60C1映射的PDO

    
}STR_OBJECTDICTIONARYSTANDARD;


typedef struct{
    Uint16 ZIndex:1;//omron
    Uint16 TouPro1:1;//omron
    Uint16 TouPro2:1;//3
    Uint16 TouPro3:1;//3
    Uint16 DI1Logic:1;//3
    Uint16 DI2Logic:1;//3
    Uint16 DI3Logic:1;//3
    Uint16 PCL:1;//3
    Uint16 NCL:1;//4
    Uint16 DIEmergStop:1;//4
    Uint16 BKOut:1;//4
    Uint16 STO1:1;//3
    Uint16 STO2:1;//4
    Uint16 EDMOut:1;//4
    Uint16 Rsvd:2;//
}STR_OMRONINSIDEDI_BIT;

typedef union{
    Uint16 all;
    volatile STR_OMRONINSIDEDI_BIT bit;
}UNION_OMRONINSIDEDI;


typedef struct{
    Uint32 DataType;     //数据类型
    Uint32 DataSize;     //数据长度
    Uint32 AccessType;   //可读写性检查
    Uint32 Active;       //生效时间检查，针对功能码
    Uint32 Mapping;		 //可映射性
	Uint32 SubindexCount;//子索引个数
	Uint32 ODBelong;     //OD在CANopen和EtherCAT中支持特性
    Uint32 ComSave;        //11 0:不可存储到EEPROM 1:可存储到EEPROM
}STR_ODATTRIB;

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */
typedef struct{            // bits   description
    Uint32  AccessType:2;   // 0-1     00:RW   01:WO    10:RO   11:RSVD
    Uint32  DataType:3;     // 2-4     010:有符号8位  011:有符号16位  100:有符号32位 101:有符号8位  110:有符号16位  111:有符号32位
    Uint32  DataSize:4;     // 5-8     0001:1个字节   0010:2个字节   0100:4个字节   1000:8个字节
    Uint32  PDOMapping:1;   // 9      0:不支持PDO映射  1:支持PDO映射
    Uint32  ODBelong:2;     //10-11      00:both 01:CANopen only 10:ECT only
    Uint32  ComSave:1;       //12 0:不可存储到EEPROM 1:可存储到EEPROM
    Uint32  Rsvd:19;        // 13-31   
}STR_OBJECCTDICTIONARY_ATTRIBUTE_BIT;

typedef union
{
    volatile Uint32 all;
    volatile STR_OBJECCTDICTIONARY_ATTRIBUTE_BIT     bit;
}UNI_OBJECCTDICTIONARY_ATTRIBUTE;


typedef struct{
    Uint32 SubindexCount;            //子索引个数
    Uint32 Value;                    //数值
    Uint32 LowerLmt;                 //下限
    Uint32 UpperLmt;                 //上限
    volatile UNI_OBJECCTDICTIONARY_ATTRIBUTE Attrib;      //对象字典属性
}STR_OBJECCTDICTIONARY_DEFAULT;



/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */
extern STR_SERVORERRORCODE  STR_SerErrCode;
extern STR_OBJECTDICTIONARYSTANDARD ObjectDictionaryStandard;
extern UNION_OMRONINSIDEDI       union_OmronInside;
extern const  STR_OBJECCTDICTIONARY_DEFAULT     ObjectDictionaryDefault[119];

extern UNI_CONTROLWORD ControlWord; 

/* 带参数的宏定义 */
//获取对象字典地址
#define     GetOD_Add(IndexOffset , Subindex)                     ((Uint32 *)ODStandardTable[(IndexOffset)] + (Subindex))
//获取对象字典子索引个数
#define     GetOD_SubindexCount(IndexOffset , Subindex)          (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].SubindexCount)
//获取对象字典默认值
#define     GetOD_Value(IndexOffset , Subindex)                   (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0))].Value)
//读对象字典值
#define     GetOD(IndexOffset , Subindex)                          ( *((Uint32 *)ODStandardTable[(IndexOffset)] + (Subindex)))
//写对象字典值
//获取对象字典上限 
#define     GetOD_LowerLmt(IndexOffset , Subindex)                (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].LowerLmt)
//获取对象字典下限
#define     GetOD_UpperLmt(IndexOffset , Subindex)                (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].UpperLmt)
//获取对象字典可读写性
#define     GetODAttrib_AccessType(IndexOffset , Subindex)       (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].Attrib.bit.AccessType)
//获取对象字典数据类型
#define     GetODAttrib_DataType(IndexOffset , Subindex)         (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].Attrib.bit.DataType)
//获取对象字典数据长度
#define     GetODAttrib_DataSize(IndexOffset , Subindex)         (ObjectDictionaryDefault[(GetOD_Add(IndexOffset , Subindex)-GetOD_Add(0 , 0)) ].Attrib.bit.DataSize)


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void AppDataToDefultValue(void);								
extern void ReadAppDataFromEeprom(void);								
extern void GetComValue(void);


#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of FUNC_GlobalVariable.h*/

/********************************* END OF FILE *********************************/ 


