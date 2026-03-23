/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:    PUB_RccDriver.h  
 创建人：   李浩                   创建日期：XXXX.XX.XX                     
 修改人：   XXXXXX                 修改日期：XXXX.XX.XX 
 描述： 
    1.
    2.
 修改记录：  
    XXXX.XX.XX  XXXXXXX
    1.
    2.
********************************************************************************/ 
#ifndef __PUB_RCCDRIVER_H
#define __PUB_RCCDRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h"

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */ 


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
extern void PUB_SystemInit(void);
extern void PUB_PeripheralClockConfig(void);
#ifdef __cplusplus
}
#endif

#endif /* __PUB_RCCDRIVER_H */

/********************************* END OF FILE *********************************/


