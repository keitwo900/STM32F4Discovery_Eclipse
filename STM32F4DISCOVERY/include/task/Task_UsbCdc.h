/**
  ******************************************************************************
  * @file    Task_UsbCdc.h
  * @author  keitwo
  * @brief   ヘッダファイルテンプレート
  ******************************************************************************
  * @attention
  * このファイルは、USB CDCの管理をするタスクです。
  * 例として、周期的にデータ受信をチェックし、受信があったらエコーバックする仕様にしています。
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
#ifndef TENPLATE_H
#define TENPLATE_H

/* Includes ------------------------------------------------------------------*/


/** @addtogroup task
  * @{
  */

/** @addtogroup Task_UsbCdc
  * @{
  */

/* Exported Defines ----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Exported_Defines Task_UsbCdc Exported Defines
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Exported Defines */

/* Exported Macros -----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Exported_Macros Task_UsbCdc Exported Macros
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Exported Macros */

/* Exported Constatnts -------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Exported_Constatnts Task_UsbCdc Exported Constatnts
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Exported Constatnts */

/* Exported Variables --------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Exported_Variables Task_UsbCdc Exported Variables
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Exported Variables */

/* Exported Functions ----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Exported_Functions Task_UsbCdc Exported Functions
  * @{
  */
extern void T_UsbCdc_Init(void);
extern void Task_UsbCdc(void);
/**
  * @}
  */ /* End Task_UsbCdc Exported Functions */

/**
  * @}
  */ /* End addtogroup Task_UsbCdc */

/**
  * @}
  */ /* End addtogroup task */

#endif /* TENPLATE_H */
/************************ (C) COPYRIGHT keitwo *****END OF FILE****/
