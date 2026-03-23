#ifndef CANOPENPUBLIC_H
#define CANOPENPUBLIC_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif

#include "PUB_GlobalPrototypes.h"
/* Exported_Constants --------------------------------------------------------*/
/* 宏定义 常数类*/
#define PosCtrlLineInterpltDflts {0, 0,0, 0, 0, 0, \
                                  0, 0, 0, 0, 0, \
                                  0, 0, 0, 0, 0, \
                                  0, 0, 0, 0, 0, \
                                  0,0}
#define PPAMPBIT   (16)

/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */	

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 
typedef struct{
    Uint32 InPut;		
    Uint32 AccFactor6097_Numerator;
    Uint32 AccFactor6097_Denominator; 
    Uint64 Vel_AccFactor6097Q14;	     //6097
	Uint64 Pos_AccFactor6097Q14;
}STR_ACC_FACTOR;


typedef struct{
    
	int64   InPut;                        //速度编码器因子转换前速度指令(用户单位)
    Uint32  VelEncoder6094_Numerator;
    Uint32  VelEncoder6094_Denominator;   //6094 用户速度--驱动器速度
    int64  VelEncoder6094Q20;
}STR_VELOCITYENCODER_FACTOR;


typedef struct{
    
	int32   InPut;                      //速度转换因子前速度反馈(10000*rpm)
    Uint32  VelFactor6095_Numerator;
    Uint32  VelFactor6095_Denominator;  //6095倒数，用户速度单位-->rpm
    Uint64  VelFactor6095Q14;           //6095,rpm-->用户速度单位
}STR_VELOCITY_FACTOR;

typedef struct{
    
	int32   InPut;                      //位置转换因子前位置给定(增量)
    int64   PosRemainder;            //位置转换因子计算的余数
    Uint32  PosFactor6093_Numerator;
    Uint32  PosFactor6093_Denominator;  
}STR_POSITION_FACTOR;

typedef struct{
    
	int64   InPut;                      //位置转换因子前绝对位置反馈
    Uint32  PosFactor6093_Numerator;
    Uint32  PosFactor6093_Denominator;  
    Uint64  PosFactor6093_InverseQ16;   //6093倒数，用于计算用户位置反馈
}STR_POSITION_FACTOR_INVERSE;


typedef struct{
    int32 MinPositionLimit;//  int32
    int32 MaxPositionLimit;//  int32
    
    Uint32 MaxProfileVelocity;//607Fh
    
    Uint64 Vel_ProfAccQ16;//6083h
    Uint64 Vel_ProfDecQ16;//6084h
    Uint64 Vel_QuickStopDecQ16;//6085h
    
    Uint64 Pos_ProfAccQ16;//6083h
    Uint64 Pos_ProfDecQ16;//6084h
    Uint64 Pos_QuickStopDecQ16;//6085h

    
    Uint32 MaxAcceleration;//60C5h
    Uint32 MaxDeceleration;//60C6h
}STR_LIMITVARIBLES;

typedef struct{
    
	Uint8 Mode;    //控制模式标志
	Uint8 ModeSwitchFlag;//模式切换标志
	Uint8 preMode; // by huangxin201711_25 0x6060在sync中更新到preMode，以保证模式切换发生在sync之后的第一个位置环

}STR_CANSYSCONTR;

typedef struct
{
    
    Uint8 PPStatus;                  //0-插补完成，未进行插补; 1－正在插补
	Uint8 PPPlanDecAgain;            // 再规划减速段一次标志
	int64 PPStartPulseQ16;           //启动速度对应每个插补周期的脉冲个数, Q16
	int64 PPStopPulseQ16;            //停止速度对应每个插补周期的脉冲个数, Q16
	int64 PPUpPulseRevQ16;           //加速脉冲增量每个插补周期, Q16
	int64 PPAvergePulseQ16;          //匀速脉冲个数每个插补周期, Q16
		
    int64 PPDownPulseRevQ16;         //减速脉冲增量每个插补周期, Q16
	int64 PPLineLengthQ16;           //直线段总长度, Q16
	int64 PPUpLengthQ16;             //加速段长度, Q16
	int64 PPAvergeLengthQ16;         //匀速段长度, Q16
    int64 PPDownLengthQ16;           //减速段长度, Q16

	int64 PPLineRemainLengthQ16;     //当前线段剩余长度, Q16
	int16 PPLineDir;                 //插补方向 1＝正方向 －1＝负方向
 	int64 PPPlanValQ16;        //当前插补周期预计插补脉冲个数, Q16
	int32 PPPlanValueRemainQ16;      //当前插补周期脉冲剩余个数, Q16
	int32 PPRealVal;           //当前插补周期实际插补脉冲数

    int64 HaltDownPulseQ16;    //暂停减速度
    int64 HaltDownLengthQ16;    //暂停总位移
    int8  ZeroPassHandle;      //连续模式，当前段为正向指令，下一段为负向指令时的过零处理
    int64 ZeroPassLengthQ16;  //连续模式，减速到0的位移
    int64 ZeroPassDownPulseRevQ16;//连续模式，每个插补周期减速脉冲增量
    
    int8  ZeroPassOver;//过零完成标志
    int64 ZeroPassRealLengthQ16;//过零实际走的距离

} STR_POSCTRLLINEINTERPLT;

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */
extern STR_ACC_FACTOR                   STR_Acc_Factor;
extern STR_VELOCITYENCODER_FACTOR       STR_VelEnc_Factor;
extern STR_VELOCITY_FACTOR              STR_Vel_Factor;
extern STR_POSITION_FACTOR              STR_Pos_Factor;
extern STR_POSITION_FACTOR_INVERSE      STR_Pos_Factor_Inverse;
extern STR_LIMITVARIBLES                STR_LmtVar;
extern STR_CANSYSCONTR                  STR_CanSyscontrol;
extern STR_POSCTRLLINEINTERPLT* pCanopenIntplt[3];


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */
extern void FactorUpdate(void);
//用户位置转化为内部Q16位置单位
extern int64 UserPosUnit2IncpUnit(STR_POSITION_FACTOR *posFactor,int32 posInput);
//将驱动器位置(绝对位置反馈)转化为用户位置，仅在位置反馈用    1/6093
extern int32 IncpUnit2UserPosUnit(STR_POSITION_FACTOR_INVERSE *posFactor,int64 posInput);
//将用户速度指令单位转化成驱动器位置指令单位:p/位置环控制周期
extern int64 UserVelUnit2IncpsUnit(STR_VELOCITYENCODER_FACTOR *velFactor,Uint32 velInput);
//将用户加速度单位转化成驱动器单位:p/位置环控制周期  Q16
extern Uint64 UserAccUnit2Incps2Unit(STR_ACC_FACTOR *accFactor,Uint32 accInput);
//将用户速度单位转化成驱动器速度单位:rpm*10000 
extern int32 UserVelUnit2RpmUnit(STR_VELOCITYENCODER_FACTOR *velFactor,int64 velInput);
//将驱动器速度单位(rpm)转化成用户速度单位,只在反馈的时候用
extern int32 Rpm2UserVel(STR_VELOCITY_FACTOR *velFactor,int32 velInput);
//将用户加速度单位转化成驱动器速度控制中的加速度单位:10000*rpm/速度环周期
extern Uint64 UserAccUnit2Rpmps2Unit(STR_ACC_FACTOR *accFactor,Uint32 accInput);
// 对于速度控制----得出速度限幅后指令10000*rpm
extern  int32 CANopenVelLmt(int32 Vel);

//速度限幅，取Max1,Max2较小者限幅
extern int64 CanopenPosVelLmt(Uint32 Vel,Uint32 MaxProfileVel);
//位置模式下，段切换时，每周期位置指令增量更新
extern void CanopenPosAccLmt (Uint32 Acc,Uint32 Dec,Uint32 QStopDec);
//驱动器单位位置指令Software限幅
extern int32 CanopenPosLmt(int32 Pos);

//Canopen直线插补初始化函数
extern void CanopenLineIntpltInit(int64 StartPulseQ16, int64 RunPulseQ16, int64 StopPulseQ16,
                    int64 RisePulseQ16, int64 DownPulseQ16, int64 LineDist,
					STR_POSCTRLLINEINTERPLT *pAtb);
//Canopen直线插补函数
extern int32 CanopenLineIntpItCal(STR_POSCTRLLINEINTERPLT *pAtb);
extern void CanopenLineIntpltReset(STR_POSCTRLLINEINTERPLT *pAtb);
extern void RecordCanopenIntpltStruct(STR_POSCTRLLINEINTERPLT *pAtb, Uint16 index);
extern void ResetCanopenIntpltDownLength(STR_POSCTRLLINEINTERPLT *pAtb);

extern void SyncPhaseComp(Uint16* pSyncFlg); //同步相位调整

#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of CanopenPublic.h*/

/********************************* END OF FILE *********************************/
