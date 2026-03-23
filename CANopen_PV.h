#ifndef CANOPENPROFILEVELOCITY_H
#define CANOPENPROFILEVELOCITY_H

#ifdef  __cplusplus                     //C++和C语言可兼容要求
extern "C" {
#endif

#include "PUB_GlobalPrototypes.h"
//#include "CanopenObjectDictionary.h"

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 


/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern  void InitCanopenPVMode(void);
extern void ECTPVAccLmt (void);
extern void CANopenSpdMonitor (void);
extern void CanSpdCtrlUpdate(void);
extern void ECTInteruptSpdShow(void);


#ifdef __cplusplus
}
#endif /* extern "C"*/ 

#endif /*end of FUNC_GlobalVariable.h*/

/********************************* END OF FILE *********************************/








