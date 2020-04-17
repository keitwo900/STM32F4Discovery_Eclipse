/**
  ******************************************************************************
  * @file    Task_UsbCdc.c
  * @author  keitwo
  * @brief   ソースファイルテンプレート
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

/* Includes ------------------------------------------------------------------*/
#include "usb_device.h"
#include "usbd_cdc_if.h"

//header of itself
#include "Task_UsbCdc.h"

/** @addtogroup task
  * @{
  */

/** @addtogroup Task_UsbCdc
  * @{
  */

/* Private Defines ----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Private_Defines Task_UsbCdc Private Defines
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Private Defines */

/* Private Macros -----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Private_Macros Task_UsbCdc Private Macros
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Private Macros */

/* Private Constatnts -------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Private_Constatnts Task_UsbCdc Private Constatnts
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Private Constatnts */

/* Private Variables --------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Private_Variables Task_UsbCdc Private Variables
  * @{
  */
static uint8_t txData[256];
/**
  * @}
  */ /* End Task_UsbCdc Private Variables */

/* Private Functions ----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Private_Functions Task_UsbCdc Private Functions
  * @{
  */

/**
  * @}
  */ /* End Task_UsbCdc Private Functions */

/* Public Functions ----------------------------------------------------------*/
/** @defgroup Task_UsbCdc_Public_Functions Task_UsbCdc Public Functions
  * @{
  */

/*******************************************************************************/
/** @par	T_UsbCdc_Init
  * @param  なし
  * @retval なし
  * @brief	USB-CDCタスクを初期化します。
  * @author keitwo
********************************************************************************/
void T_UsbCdc_Init(void)
{
	MX_USB_DEVICE_Init();
}

/*******************************************************************************/
/** @par	Task_UsbCdc
  * @param  なし
  * @retval なし
  * @brief	USB-CDC周期タスクです。
  * @author keitwo
********************************************************************************/
void Task_UsbCdc(void)
{
	/* 受信データエコーバック */
	int32_t data;
	int i = 0;
	while((data = CDC_GetDataFromRxRing()) != CDC_RX_NODATA)
	{
		txData[i++] = data & 0xFF;
	}
	if(i != 0)
	{
		CDC_Transmit_FS(txData, i); /* txData内のデータは送信完了まで保持される必要があるので、高速に連続にコールされる場合はリングバッファに変更すること */
	}
}

/**
  * @}
  */ /* End Task_UsbCdc Public Functions */

/**
  * @}
  */ /* End addtogroup Task_UsbCdc */

/**
  * @}
  */ /* End addtogroup task */

/************************ (C) COPYRIGHT keitwo *****END OF FILE****/
