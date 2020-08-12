/**
  ******************************************************************************
  * @file    drv_cs43l22.h
  * @author  keitwo
  * @date    2020/08/12
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

/** @defgroup drv_cs43l22_Private_Types drv_cs43l22 Private Types
  * @{
  */

typedef void (*I2C_COMP_FUNC)(void); /*!< I2C通信完了時のコールバック関数の型 */

/*! @enum I2C_DIRECTION_E
    @brief  I2Cの送受信方向(※受信のみは動作未確認です)
*/
typedef enum
{
    I2C_DIR_TX,                     /*!< 送信のみ */
    I2C_DIR_RX,                     /*!< 受信のみ */
    I2C_DIR_RX_WITH_ADDR_SELECT,    /*!< 内部アドレスを送信後、データを受信 */

    NUM_OF_I2C_DIR,
} I2C_DIRECTION_E;

/*! @struct I2C_TRANS_INFO_S
    @brief  I2C通信情報フォーマット
*/
typedef struct
{
    uint8_t slaveAddress;       /*!< デバイスのスレーブアドレス(上詰め) */
    uint8_t internalAddress;    /*!< コーデックの内部レジスタアドレス */
    I2C_DIRECTION_E dir;        /*!< 送受信方向 */
    uint8_t* pBuf;              /*!< 送受信データのポインタ */
    int bufSize;                /*!< 送受信データサイズ */
    I2C_COMP_FUNC i2cCompFunc;  /*!< I2C通信完了時にコールしてほしい関数 */
} I2C_TRANS_INFO_S;

/**
  * @}
  */ /* End drv_cs43l22 Private Types */

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
extern uint8_t DRV_CS43L22_ReadRegOneByteWithWait(uint8_t internalAddr);
extern void DRV_CS43L22_ReadRegsWithWait(uint8_t internalAddr, uint8_t* data, int size);
extern void DRV_CS43L22_WriteRegOneByteWithWait(uint8_t internalAddr, uint8_t data);
extern void DRV_CS43L22_WriteRegsWithWait(uint8_t internalAddr, uint8_t* data, int size);
extern void DRV_CS43L22_RequestI2c(I2C_TRANS_INFO_S i2cTransInfo);
extern void DRV_CS43L22_ExecuteI2cPeriodic(void);
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
