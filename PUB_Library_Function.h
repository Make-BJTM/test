/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名: PUB_Library_Function.h       库函数                                                 
 创建人：童文邹、马仕贤         创建日期：2008.08.10
 修改人：朱祥华                 修改日期：2012.02.23 
         姚虹 					
 描述： 数学计算函数库
 修改记录：  
    1. 朱祥华      2012.02.23

                 
    2. 姚虹         2012.09.18
       变更内容： 加入快速计算算术平方根的算法
********************************************************************************/
#ifndef PUB_LIBRARY_FUNCTION_H
#define PUB_LIBRARY_FUNCTION_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */  
#include "PUB_GlobalPrototypes.h" 


/* Exported_Constants --------------------------------------------------------*/
/* 函数声明*/

Static_Inline Uint16  qsqrt(Uint32 Input);
Static_Inline Uint32 qsqrt64(Uint64 dwNumber);
Static_Inline Uint16 qsqrt32(Uint32 dwNumber);


/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */


/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 
//暂无

/* Inline Function --------------------------------------------------------*/
/* 内联函数定义 */
/*******************************************************************************
  函数名: 
  输入:   无 
  输出:   无 
  子函数: 无
    1.  
    2.
********************************************************************************/
Static_Inline Uint16  qsqrt(Uint32 Input)
{
    int8      i;
    Uint32    SquareRoot;

    if(Input == 0)                return (0);

    if(Input <= 4194304)          SquareRoot = (Input>>10) + 63;

    else if (Input <= 134217728)  SquareRoot = (Input>>12) + 255;

    else                          SquareRoot = (Input>>14) + 1023;

    for (i=0;i<5;i++)
    {
        SquareRoot=(SquareRoot+Input/SquareRoot)>>1;
    }

    return((Uint16)SquareRoot);    
}


/*******************************************************************************
  函数名: 
  输入:   要开方的64位数 
  输出:   求取的算术平方根 
  子函数: 无
  函数说明：此函数只允许输入64位数，如果一个32位数输入，则会延长计算时间
********************************************************************************/
Static_Inline Uint32 qsqrt64(Uint64 dwNumber)
{
    Uint32  dwSquareRoot = 0;   //最终得到的算术平方根
	Uint16  i;
	Uint32  Residual;	    //计算的余数
	Uint32  Temp;	        //用于递推的比较数

    union 
    {
        Uint32 Bit32[2];
        Uint64 Bit64;
    }ShiftNum;

	if(dwNumber < 2)             //输入小于2时不计算
	{
	    return(dwNumber);
	}
	
    ShiftNum.Bit64 = dwNumber;

	Residual  = ((ShiftNum.Bit32[1]) >> 30) & 0x03;
    ShiftNum.Bit32[1] = (ShiftNum.Bit32[1] << 2) | ((ShiftNum.Bit32[0] >> 30) & 0x03);     //去除64位数高两位
    ShiftNum.Bit32[0] = ShiftNum.Bit32[0] << 2;

    if(Residual > 0)
    {
        dwSquareRoot = 1;				//得到开平方的最高位
        Residual = Residual - dwSquareRoot;    //计算剩余的余数
    }

    i = 0;
    while(i < 31)
    {
        i++;
        Residual = Residual<<2;
        Residual += (((ShiftNum.Bit32[1]) >> 30) & 0x03);    //再取开平方数的高两位，得到新的余数值
        dwSquareRoot = dwSquareRoot<<1;      //平方根也进行移位以获取下一位的数值
        Temp = (dwSquareRoot<<1) + 1;        //假定平方根的下一位为1；

        if(Residual >= Temp)     //如果新的低位为1即假设成立，则需要再次计算余数
        {
            dwSquareRoot++;
            Residual -= Temp;
        }

        ShiftNum.Bit32[1] = (ShiftNum.Bit32[1] << 2) | ((ShiftNum.Bit32[0] >> 30) & 0x03);     //去除64位数高两位
        ShiftNum.Bit32[0] = ShiftNum.Bit32[0] << 2;
    } 
	
	return(dwSquareRoot);     //返回计算的算术平方根 
}

/*******************************************************************************
  函数名: 
  输入:   要开方的64位数 
  输出:   求取的算术平方根 
  子函数: 无
  函数说明：此函数只允许输入64位数，如果一个32位数输入，则会延长计算时间
********************************************************************************/
Static_Inline Uint16 qsqrt32(Uint32 dwNumber)
{
    Uint16  dwSquareRoot = 0;   //最终得到的算术平方根
	Uint16  i;
	Uint32  Residual;	    //计算的余数
	Uint32  Temp;	        //用于递推的比较数
	Uint32  ShiftNum;		//原数值进行移位后的数

	if(dwNumber < 2)             //输入小于2时不计算
	{
	    return(dwNumber);
	}
	
	Residual = dwNumber>>30;   //得到输入的最高两位
	ShiftNum = dwNumber<<2;   //移位去除最高两位
	
	if(Residual > 0)
	{
	    dwSquareRoot = 1;				//得到开平方的最高位
		Residual = Residual - dwSquareRoot;    //计算剩余的余数
	}

	for(i=0; i<15; i++)
	{
	    Residual = Residual<<2;
		Residual += ShiftNum>>30;   //再取开平方数的高两位，得到新的余数值
		dwSquareRoot = dwSquareRoot<<1;      //平方根也进行移位以获取下一位的数值
		Temp = (dwSquareRoot<<1) + 1;        //假定平方根的下一位为1；

		if(Residual >= Temp)		//如果新的低位为1即假设成立，则需要再次计算余数   
		{
		    dwSquareRoot++;
			Residual -= Temp; 	  
		}

		ShiftNum  = ShiftNum<<2;
	} 
	
	return(dwSquareRoot);     //返回计算的算术平方根
}


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 




#ifdef __cplusplus
}
#endif

#endif /*PUB_Library_Function.h*/    

/********************************* END OF FILE *********************************/
