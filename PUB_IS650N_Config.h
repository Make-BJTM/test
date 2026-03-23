/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:    PUB_IS650N_Config.h  
 创建人：   王治国                 创建日期：2014.08.01
 描述： 
    1. 
 修改记录：  
    20xx.xx.xx   xxx
    1. xxxx
********************************************************************************/ 
#ifndef __PUB_IS650N_CONFIG_H
#define __PUB_IS650N_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */


/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */

/* 伺服软件版本号命名规则(10进制)
 * XY.A 
 * X  驱动器型号0-64
 * Y  归档版本号0-99
 * A  市场临时版本号0-9
*/

//标准软件通用版本号Y.A
#define VERSION_STANDARD        7

//H0100 DSP软件通用版本号
#define VERSION_H0100         ((11 * 1000) + VERSION_STANDARD)

/* 软件内部版本号命名规则(16进制)
 * B.D 
 * B  提交测试部版本号0-99
 * D  研发内部版本号0-99
*/
//提交测试版本号
#define VERSION_B               0

//内部版本号
#define VERSION_D               1

//H0150 DSP软件内部版本号
#define VERSION_H0150         ((VERSION_B * 100) + VERSION_D)

//H0151 FPGA软件内部版本号
#define VERSION_H0151         ((VERSION_B * 100) + 0)

/* 非标版本号命名规则(16进制)
 * FFF.AB 
 * FFF  非标号
 * A   非标通用版本
 * B   非标临时版本
*/ 
//直线电机非标         
#define LINEARMOT   0x001
//600P非标
#define IS600P      0x216   
      
//非标号
#define NONSTANDARD_PROJECT    0x721

//非标软件版本号
#if NONSTANDARD_PROJECT != 0
    #define VERSION_NONSTANDARD     0x01 
#else
    #define VERSION_NONSTANDARD     0x00 
#endif

#define VERSION_H0002      ((Uint16)((NONSTANDARD_PROJECT << 8) + VERSION_NONSTANDARD)) 
#define VERSION_H0003      (NONSTANDARD_PROJECT >> 8) 

//CAN功能使能开关
//0--代表不使能CAN功能；1--代表使能CAN功能
#define  CAN_ENABLE_SWITCH      0 

//EtherCAT功能使能开关
//0--代表不使能ECT功能；1--代表使能ECT功能
#define  ECT_ENABLE_SWITCH      1 

//海德汉编码器开关
#define  HDH_ENCODER_SW         0 

//汇川编码器开关
#define  HC_ENC_SW              1 

//尼康编码器开关
#define  NOKIN_ENC_SW           1 

//多摩川编码器开关
#define  TAMAGAWA_ENC_SW        1  


//电子标签宏定义  借用620N
//厂商
#define EL_OEM               1
//产品线
#define EL_PRODUCT_LINE       13
//产品型号
#define EL_MODEL             0x108
//产品版本
#define EL_PRODUCT_VER       VERSION_H0100
//CAN通信版本
#define EL_CAN_VER           310
//ECAT通信版本
#define EL_ECAT_VER          3000
//BOOT版本
#define EL_BOOT_VER          0
//芯片型号
#define EL_CHIP_MODEL        0x130
//非标号
#define EL_NONSTANDARD_VER   VERSION_NONSTANDARD


#ifdef __cplusplus
}
#endif

#endif /* __PUB_IS650N_CONFIG_H */

/********************************* END OF FILE *********************************/
