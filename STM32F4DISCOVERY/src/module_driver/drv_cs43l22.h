/**
  ******************************************************************************
  * @file    drv_cs43l22.h
  * @author  keitwo
  * @date    2020/08/07
  * @brief   オーディオコーデック制御プログラム
  ******************************************************************************
  * @attention
  * このファイルは、オーディオコーデックCS43L22のドライバソースファイルです。
  * I2C、I2Sを制御します。
  *
  * @lisence <h2><center>&copy; Copyright (c) 2020 keitwo
  *          All rights reserved.</center></h2><br> 
  *          BSD 2 Clause License as below.
  ****************************************************************************** 
  * Copyright (c) 2020, keitwo
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this
  *    list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The views and conclusions contained in the software and documentation are those
  * of the authors and should not be interpreted as representing official policies,
  * either expressed or implied, of the FreeBSD Project.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRV_CS43L22_H
#define DRV_CS43L22_H

/* Includes ------------------------------------------------------------------*/


/** @addtogroup drv_cs43l22Group
  * @{
  */

/** @addtogroup drv_cs43l22
  * @{
  */

/* Exported Defines ----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Exported_Defines drv_cs43l22 Exported Defines
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Exported Defines */

/* Exported Macros -----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Exported_Macros drv_cs43l22 Exported Macros
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Exported Macros */

/* Exported Constants -------------------------------------------------------*/
/** @defgroup drv_cs43l22_Exported_Constants drv_cs43l22 Exported Constants
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Exported Constants */

/* Exported Variables --------------------------------------------------------*/
/** @defgroup drv_cs43l22_Exported_Variables drv_cs43l22 Exported Variables
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Exported Variables */

/* Exported Functions ----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Exported_Functions drv_cs43l22 Exported Functions
  * @{
  */

extern void DRV_CS43L22_Init(uint32_t SamplingFreq, void* audioBuf, int audioBufSampleSize);
extern uint8_t DRV_CS43L22_ReadRegisterOneByte(uint8_t internalAddr);
extern void DRV_CS43L22_ReadRegisters(uint8_t internalAddr, uint8_t* data, int size);
extern void DRV_CS43L22_WriteRegisterOneByte(uint8_t internalAddr, uint8_t data);
extern void DRV_CS43L22_WriteRegisters(uint8_t internalAddr, uint8_t* data, int size);
extern int DRV_CS43L22_I2CTest(void);

/**
  * @}
  */ /* End drv_cs43l22 Exported Functions */

/**
  * @}
  */ /* End addtogroup drv_cs43l22 */

/**
  * @}
  */ /* End addtogroup drv_cs43l22Group */

#endif /* DRV_CS43L22_H */
/************************ (C) COPYRIGHT keitwo *****END OF FILE****/
