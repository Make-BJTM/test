/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:    MTR_FUNCInterface.h
 创建人:    李浩                    创建日期：2012.03.26
 描述:
    1.
    2.
 修改记录：
    xx.xx.xx      XX
    1.      
    2. 
********************************************************************************/
#ifndef __MTR_FUNCINTERFACE_H
#define __MTR_FUNCINTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h" 

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义*/

#define     MTRTOFUNC_INITLIST_NUM         4       //初始化程序中由 MTR传到FUNC数据变量个数  (以32bit为基本单位)

#define     MTRTOFUNC_LIST_1kHz_NUM        5       //主循环(1K调度)程序中由 MTR传到FUNC数据变量个数  (以32bit为基本单位)

#define     MTRTOFUNC_LIST_16kHz_NUM       23      //中断(16K调度)程序中由 MTR传到FUNC数据变量个数  (以32bit为基本单位)

/* Exported_Macros -----------------------------------------------------------*/
/* 带参数的宏定义 */


/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */


/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void G_MTR_FUNCGetInitList(Uint32 * pFUNC_InitListHeadAddr);

extern void G_MTR_FUNCGetList_1kHz(Uint32 * pFUNC_List_1kHz_HeadAddr);

extern void G_MTR_FUNCGetList_16kHz(Uint32 * pFUNC_List_16kHz_HeadAddr);

extern void G_MTR_FUNCGetFSAList_16kHz(Uint32 * pFUNC_FSAList_16kHz_HeadAddr);

extern void SetEthercatSyncMode(void);
extern void SetSelfSyncMode(void);
extern void CompensatePwmPeriod(int8 CompVal);

extern Uint8 GetEcatActive(void);
extern Uint16 GetSyncPeriod(void);
extern int32 GetSyncLength(void);    

extern void AbsMod1_MultiTurnOffset(void);     //绝对线性模式原点回归后重新设置多圈偏置

extern void UpdateH0B_FPGA_State(void);        //H0B组FPGA状态信息更新

extern void AbsRom_BaudSet_2MHZ(void);	   //设置波特率为2Mhz
#ifdef __cplusplus
}
#endif

#endif /* __MTR_FUNCINTERFACE_H */

/********************************* END OF FILE *********************************/
