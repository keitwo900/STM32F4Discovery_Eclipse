/**
  ******************************************************************************
  * @file    drv_cs43l22.c
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

/* Includes ------------------------------------------------------------------*/

//standard libraries
#include <stdbool.h>

//peripheral drivers
#include "stm32f407xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"

//header of itself
#include "drv_cs43l22.h"

/** @addtogroup drv_cs43l22Group
  * @{
  */

/** @addtogroup drv_cs43l22
  * @{
  */

/* Private Defines ----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Private_Defines drv_cs43l22 Private Defines
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Private Defines */

/* Private Macros -----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Private_Macros drv_cs43l22 Private Macros
  * @{
  */

/* ----------------------------------------------------
ハードウェア関連定義
---------------------------------------------------- */

/* I2C関連 */
#define AUDIO_RESET_GPIO    GPIOD
#define AUDIO_RESET_PIN     LL_GPIO_PIN_4

#define AUDIO_I2C_GPIO  GPIOB
#define AUDIO_SCL_PIN   LL_GPIO_PIN_6
#define AUDIO_SDA_PIN   LL_GPIO_PIN_9
#define AUDIO_I2C       I2C1
#define DRV_CS43L22_I2cEventHandler I2C1_EV_IRQHandler

#define CODEC_SLAVE_ADDRESS 0x94  /* b1001010r/w */
#define I2C_READ_BIT 0x01 /* リードはLSBが1 */
#define CODEC_MAP_INCR_BIT 0x80 /* 内部アドレスバイトのMSBに1を立てると、インクリメント書き込み、読み出しが可能になる */

/* I2S関連 */
#define AUDIO_I2S_GPIO1     GPIOC /* Master Clock, Bit Clock, Serial Data */
#define AUDIO_I2S_GPIO2     GPIOA /* Word Clock */
#define AUDIO_I2S_MCK_PIN   LL_GPIO_PIN_7 /* PC7 */
#define AUDIO_I2S_BCK_PIN   LL_GPIO_PIN_10 /* PC10 */
#define AUDIO_I2S_SD_PIN    LL_GPIO_PIN_12 /* PC12 */
#define AUDIO_I2S_WCK_PIN   LL_GPIO_PIN_4 /* PA4 */
#define AUDIO_I2S           SPI3

/* I2S DMA関連 */
#define AUDIO_I2S_DMA               DMA1
#define AUDIO_I2S_DMA_STREAM        LL_DMA_STREAM_7
#define AUDIO_I2S_DMA_CH            LL_DMA_CHANNEL_0
#define DRV_CS43L22_I2sDmaHandler   DMA1_Stream7_IRQHandler

/* ----------------------------------------------------
ソフトウェア関連定義
---------------------------------------------------- */
#define DEFAULT_VOLUME  -96 /* 最初に設定するマスターボリューム */
#define I2C_QUEUE_SIZE  16 /* I2C通信要求を貯められる最大サイズ。2のべき乗にすること! */

/**
  * @}
  */ /* End drv_cs43l22 Private Macros */

/* Private Constants -------------------------------------------------------*/

/** @defgroup drv_cs43l22_Private_Types drv_cs43l22 Private Types
  * @{
  */

/*! @struct DRV_CS43L22_CLASS
    @brief  オーディオコーデックドライバクラス
*/
typedef struct
{
    I2C_TRANS_INFO_S i2cTransInfo[I2C_QUEUE_SIZE];
    int i2cTransRi;
    int i2cTransWi;
    bool i2cMtx; /* I2Cが通信中か否か。通信中ならtrue */
    int i2cTransCnt;
} DRV_CS43L22_CLASS;

/**
  * @}
  */ /* End drv_cs43l22 Private Types */

/* Private Constants -------------------------------------------------------*/
/** @defgroup drv_cs43l22_Private_Constants drv_cs43l22 Private Constants
  * @{
  */

/**
  * @}
  */ /* End drv_cs43l22 Private Constants */

/* Private Variables --------------------------------------------------------*/
/** @defgroup drv_cs43l22_Private_Variables drv_cs43l22 Private Variables
  * @{
  */

DRV_CS43L22_CLASS drvCs43l22;

/**
  * @}
  */ /* End drv_cs43l22 Private Variables */

/* Private Functions ----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Private_Functions drv_cs43l22 Private Functions
  * @{
  */

/*******************************************************************************/
/** @par    DRV_CS43L22_Reset
  * @param  なし
  * @retval なし
  * @brief  オーディオコーデックをリセットし、時間待ちした後リセット解除します。
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_Reset(void)
{
    volatile int i;
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* オーディオコーデックリセットピン GPIO初期化 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); /* GPIOD クロック有効化 */
    GPIO_InitStruct.Pin = AUDIO_RESET_PIN; /* PD4 */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; /* オルタネート(I2C)で使用 */
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; /* プッシュプル出力 */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* プルアップ/プルダウンともしない */
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW; /* リセットピンなので低速で良い */
    LL_GPIO_ResetOutputPin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN); /* まずリセット */
    LL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);
    
    for(i=0; i < 0x4FFF; i++){} /* リセット時間確保(時間は適当) */
    LL_GPIO_SetOutputPin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN); /* リセット解除 */
}

/*******************************************************************************/
/** @par	DRV_CS43L22_InitI2c
  * @param  なし
  * @retval なし
  * @brief	オーディオコーデックを制御するI2Cを初期化します。
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_InitI2c(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    
    /* I2C GPIO初期化 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); /* GPIOB クロック有効化 */
    GPIO_InitStruct.Pin = AUDIO_SCL_PIN | AUDIO_SDA_PIN; /* PB6(SCL), PB9(SDA) */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE; /* オルタネート(I2C)で使用 */
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN; /* オープンドレイン出力 */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* プルアップ/プルダウンともしない */
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; /* シリアルインタフェースなので高速 */
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4; /* オルタネート機能4(I2C SCL/SDA) */
    LL_GPIO_Init(AUDIO_I2C_GPIO, &GPIO_InitStruct);
    
    /* I2Cペリフェラル初期化 */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1); /* I2Cペリフェラルへのクロック有効化 */
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 400000;
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2C1, 0);
    
    /* I2C1 割り込み初期化 */
    NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(I2C1_ER_IRQn);
}


/*******************************************************************************/
/** @par    DRV_CS43L22_VolumeCtrl
  * @param  uint8_t vol_dB: オーディオコーデックに設定するマスターボリューム(dB値)
  *                         範囲-102dB ~ 12dBで0.5dB刻みです。設定したい値を2倍して
  *                         入力してください。
  *                         例: -102dB -> -204, 12dB -> 24, 4.5dB -> 9
  * @retval なし
  * @brief  I2Cを使用してオーディオコーデックの音量を設定します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_VolumeCtrl(int16_t vol_dB)
{
    vol_dB = (vol_dB >= 0) ? vol_dB : 0x0100 + vol_dB; /* 負のdBの変換方法はオーディオコーデックのマニュアル参照 */
    
    /* Set the Master volume */
    DRV_CS43L22_WriteRegOneByteWithWait(0x20, (uint8_t)vol_dB); 
    DRV_CS43L22_WriteRegOneByteWithWait(0x21, (uint8_t)vol_dB);
}

/*******************************************************************************/
/** @par    DRV_CS43L22_InitCodec
  * @param  なし
  * @retval なし
  * @brief  I2Cを使用してオーディオコーデックを初期化します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_InitCodec(void)
{
    DRV_CS43L22_WriteRegOneByteWithWait(0x02, 0x01); /* コーデックパワーオフ */
    DRV_CS43L22_WriteRegOneByteWithWait(0x04, 0xAF); /* スピーカーオフ、ヘッドフォンON*/
    DRV_CS43L22_WriteRegOneByteWithWait(0x05, 0x81); /* クロック設定: 自動検出 */
    DRV_CS43L22_WriteRegOneByteWithWait(0x06, 0x04); /* I2S規格: I2Sフィリップス標準*/
    
    DRV_CS43L22_VolumeCtrl(DEFAULT_VOLUME); /* ボリューム初期値設定 */
    
    DRV_CS43L22_WriteRegOneByteWithWait(0x02, 0x9E); /* コーデックパワーオン */
    DRV_CS43L22_WriteRegOneByteWithWait(0x0A, 0x00); /* アナログソフトランプ無効化 */
    DRV_CS43L22_WriteRegOneByteWithWait(0x0E, 0x04); /* デジタルソフトランプ無効化*/
    DRV_CS43L22_WriteRegOneByteWithWait(0x27, 0x00); /* アタックレベルのリミッタ無効か */
    DRV_CS43L22_WriteRegOneByteWithWait(0x1F, 0x0F); /* 低音域、高音域調整 */
    DRV_CS43L22_WriteRegOneByteWithWait(0x1A, 0x0A); /* PCMボリュームレベル調整 */
    DRV_CS43L22_WriteRegOneByteWithWait(0x1B, 0x0A); /* PCMボリュームレベル調整 */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_InitI2s
  * @param  uint32_t SamplingFreq: サンプリング周波数
  * @param  void* audioBuf: 楽音バッファの先頭ポインタ
  * @param  int audioBufSampleSize: 楽音バッファのサンプル数
  * @retval なし
  * @brief  I2Sを設定してオーディオ信号経路を開通します。
  *         DMAのサーキュラーモードを使用し、楽音バッファの前半、後半で
  *         ダブルバッファを構成してソフトウェア信号処理をするシステムにします。
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_InitI2s(uint32_t SamplingFreq, void* audioBuf, int audioBufSampleSize)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_I2S_InitTypeDef I2S_InitStruct = {0};
    LL_DMA_InitTypeDef DMA_InitStruct = {0};
    
    /* I2S GPIO初期化 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOC); /* GPIOA,C クロック有効化 */
    GPIO_InitStruct.Pin = AUDIO_I2S_MCK_PIN | AUDIO_I2S_BCK_PIN | AUDIO_I2S_SD_PIN; /* PC7(MCK), PC10(BCK), PC12(SD) */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE; /* オルタネート(I2S)で使用 */
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; /* プッシュプル出力 */
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* プルアップ/プルダウンともしない */
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; /* シリアルインタフェースなので高速 */
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6; /* オルタネート機能6(I2C I2S3_MCK/I2S3_CK/I2S3_SD/I2S3_WS) */
    LL_GPIO_Init(AUDIO_I2S_GPIO1, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = AUDIO_I2S_WCK_PIN; /* PA4(WCK) */
    LL_GPIO_Init(AUDIO_I2S_GPIO2, &GPIO_InitStruct);
    
    /* I2Sペリフェラル初期化 */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3); /* I2Sペリフェラルへのクロック有効化 */
    I2S_InitStruct.Mode = LL_I2S_MODE_MASTER_TX; /* マスター、送信で使用 */
    I2S_InitStruct.Standard = LL_I2S_STANDARD_PHILIPS; /* I2SのモードはI2Sフィリップス標準 */
    I2S_InitStruct.DataFormat = LL_I2S_DATAFORMAT_16B; /* 16bit */
    I2S_InitStruct.MCLKOutput = LL_I2S_MCLK_OUTPUT_ENABLE; /* マスタークロックを出力する */
    I2S_InitStruct.AudioFreq = SamplingFreq; /* サンプリング周波数 */
    I2S_InitStruct.ClockPolarity = LL_I2S_POLARITY_LOW; /* クロック極性は定常状態(テータが出ていないとき)でLow */
    LL_I2S_Init(AUDIO_I2S, &I2S_InitStruct);
    
    /* I2S DMA 初期化 */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1); /* DMA1 クロック有効化 */
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)(&(AUDIO_I2S->DR)); /* I2Sデータレジスタのアドレス */
    DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)audioBuf; /* オーディオバッファのアドレス */
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH; /* 方向はメモリからペリフェラル(出力) */
    DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR; /* サーキュラーモード */
    DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT; /* ペリフェラルはレジスタなのでインクリメントなし */
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT; /* メモリは楽音データを更新するのでインクリメント */
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD; /* 16bit */
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD; /* 16bit */
    DMA_InitStruct.NbData = audioBufSampleSize; /* オーディオバッファのサンプルサイズ */
    DMA_InitStruct.Channel = AUDIO_I2S_DMA_CH; /* Channel 0 (マニュアルより) */
    DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH; /* オーディオは途切れてはいけないので優先度高め */
    DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE; /* FIFOは使わない */
    // DMA_InitStruct.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_2; /* FIFOは使わないので不要 */
    DMA_InitStruct.MemBurst = LL_DMA_MBURST_SINGLE; /* バースト転送しない */
    DMA_InitStruct.PeriphBurst = LL_DMA_PBURST_SINGLE; /* バースト転送しない */
    LL_DMA_Init(AUDIO_I2S_DMA, AUDIO_I2S_DMA_STREAM, &DMA_InitStruct);
    
    /* I2S DMA 割り込み有効化 */
    LL_DMA_EnableIT_HT(AUDIO_I2S_DMA, AUDIO_I2S_DMA_STREAM); /* DMA半分転送割り込み */
    LL_DMA_EnableIT_TC(AUDIO_I2S_DMA, AUDIO_I2S_DMA_STREAM); /* DMA転送完了割り込み */
    NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    
    /* I2S DMA 有効化 */
    LL_I2S_EnableDMAReq_TX(AUDIO_I2S); /* I2Sペリフェラル側での、DMA送信有効化 */
    LL_DMA_EnableStream(AUDIO_I2S_DMA, AUDIO_I2S_DMA_STREAM);
    
    /* I2S有効化 */
    LL_I2S_Enable(AUDIO_I2S);
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2cPostFunction
  * @param  なし
  * @retval なし
  * @brief  I2C通信完了時にコールされる関数です。
  *         本関数は割り込みハンドラからコールされます。
  * @author keitwo
********************************************************************************/
static void DRV_CS43L22_I2cPostFunction(void)
{
    drvCs43l22.i2cMtx = false;
    drvCs43l22.i2cTransRi++;
    drvCs43l22.i2cTransRi &= I2C_QUEUE_SIZE - 1; /* リミッタ */
}

/**
  * @}
  */ /* End drv_cs43l22 Private Functions */

/* Public Functions ----------------------------------------------------------*/
/** @defgroup drv_cs43l22_Public_Functions drv_cs43l22 Public Functions
  * @{
  */

/*******************************************************************************/
/** @par	DRV_CS43L22_Init
  * @param  uint32_t SamplingFreq: サンプリング周波数
  * @param  void* audioBuf: 楽音バッファの先頭ポインタ
  * @param  int audioBufSampleSize: 楽音バッファのサンプル数
  * @retval なし
  * @brief	CS43L22を初期化します。
  *         CS43L22につながるI2C、I2Sピンと、ペリフェラルも初期化します。
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_Init(uint32_t SamplingFreq, void* audioBuf, int audioBufSampleSize)
{
    DRV_CS43L22_Reset();
    DRV_CS43L22_InitI2c();
    DRV_CS43L22_InitCodec();
    DRV_CS43L22_InitI2s(SamplingFreq, audioBuf, audioBufSampleSize);
    
#if 0
    /* I2C関数テスト(テストするとオーディオコーデックの設定が変わるため、音が鳴らなくなります。) */
    if(DRV_CS43L22_I2CTest())
    {
        while(1); /* テスト失敗 */
    }
#endif
}

/*******************************************************************************/
/** @par    DRV_CS43L22_ReadRegOneByteWithWait
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @retval uint8_t: 読み出しデータ
  * @brief  I2Cでオーディオコーデックのレジスタからデータを1byte読み出します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
uint8_t DRV_CS43L22_ReadRegOneByteWithWait(uint8_t internalAddr)
{
    uint8_t result;
    
    while(LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)); /* 前の転送完了待ち */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* スタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* スタートコンディション発行完了待ち */
    
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS); /* スレーブアドレス送信 */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, internalAddr); /* オーディオコーデック内部アドレス送信 */
    while(!LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */

    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* リスタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* リスタートコンディション発行完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS | I2C_READ_BIT); /* スレーブアドレス送信(受信) */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* アドレス送信完了待ち(受信) */
    
    
    LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え(1byte受信の時は、ADDRフラグを落とす前!) */
    
    (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
    
    LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
    while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
    
    while(!LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C)); /* データ受信待ち */
    result = LL_I2C_ReceiveData8(AUDIO_I2C); /* データ受信 */
    
    //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
    
    LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。 */
        
    return result;
}

/*******************************************************************************/
/** @par    DRV_CS43L22_ReadRegsWithWait
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t* data: 読み出しデータを格納する配列の先頭ポインタ
  * @param  int size: データサイズ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタからデータを読み出します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_ReadRegsWithWait(uint8_t internalAddr, uint8_t* data, int size)
{
    int i;
    
    if(size <= 0) return; 
    
    while(LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)); /* 前の転送完了待ち */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* スタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* スタートコンディション発行完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS); /* スレーブアドレス送信 */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, internalAddr | CODEC_MAP_INCR_BIT); /* オーディオコーデック内部アドレス送信(連続読み出しBit On) */
    while(!LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
            || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* リスタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* リスタートコンディション発行完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS | I2C_READ_BIT); /* スレーブアドレス送信(受信) */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* アドレス送信完了待ち(受信) */
    
    switch(size)
    {
    case 1:
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え(1byte受信の時は、ADDRフラグを落とす前!) */
            
        (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
            
        LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
        
        while(!LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C)); /* データ受信待ち */
        *data = LL_I2C_ReceiveData8(AUDIO_I2C); /* データ受信 */
           
        //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
            
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
        
        break;
        
    case 2:
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え */
        LL_I2C_EnableBitPOS(AUDIO_I2C); /* POS をHighに切り替え(現在のシフトレジスタのデータの次のバイトに対しACK bitの内容を適用する) */
        
        (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
        
        while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* データ受信完了待ち(DRにbyte 0、シフトレジスタにbyte 1が来るのを待つ) */
        
        LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
         
        *data++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 0格納 */
        *data = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 1格納 */
        
        //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
        
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
        LL_I2C_DisableBitPOS(AUDIO_I2C); /* POS をLowに戻す */
        
        break;
    
    default: /* 3byte以上 */
        
        (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
        
        for(i = 0; i < size-3; i++)
        {
            while(!LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C)); /* データ受信待ち */
            *data++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* データ格納 */
        }
        
        while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* データ受信完了待ち(DRにbyte (N-3)、シフトレジスタにbyte (N-2)が来るのを待つ) */
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え */
        
        *data++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte (N-3)格納 */ 
        
        while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* データ受信完了待ち(DRにbyte (N-2)、シフトレジスタにbyte (N-1)が来るのを待つ) */
        
        LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
        
        *data++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte (N-2)格納 */
        *data = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte (N-1)格納 */
        
        //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
        
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
        
        break;
    }
}

/*******************************************************************************/
/** @par    DRV_CS43L22_WriteRegOneByteWithWait
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t data: 書き込みデータ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタにデータを1byte書き込みます。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_WriteRegOneByteWithWait(uint8_t internalAddr, uint8_t data)
{
    while(LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)); /* 前の転送完了待ち */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* スタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* スタートコンディション発行完了待ち */
    
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS); /* スレーブアドレス送信 */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, internalAddr); /* オーディオコーデック内部アドレス送信 */
    while(!LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
            || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */

    LL_I2C_TransmitData8(AUDIO_I2C, data); /* データ送信 */
    while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* データ送信完了待ち */
    
    LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
    //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_WriteRegsWithWait
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t* data: データの先頭ポインタ
  * @param  int size: データサイズ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタにデータを書き込みます。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_WriteRegsWithWait(uint8_t internalAddr, uint8_t* data, int size)
{
    int i;
    
    while(LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)); /* 前の転送完了待ち */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* スタートコンディション発行 */
    while(!LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)); /* スタートコンディション発行完了待ち */
    
    
    LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS); /* スレーブアドレス送信 */
    while(!LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
        || !LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    LL_I2C_TransmitData8(AUDIO_I2C, internalAddr | CODEC_MAP_INCR_BIT); /* オーディオコーデック内部アドレス送信(連続書き込みBit On) */
    while(!LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
            || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    
    for(i = 0; i < size-1; i++)
    {
        LL_I2C_TransmitData8(AUDIO_I2C, *data++); /* データ送信 */
        while(!LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
            || !LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
            || !LL_I2C_GetTransferDirection(AUDIO_I2C)); /* アドレス送信完了待ち */
    }
    
    LL_I2C_TransmitData8(AUDIO_I2C, *data); /* 最終データ送信 */
    while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* 最終データ送信完了待ち */
    
    LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
    //while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_RequestI2c
  * @param  I2C_TRANS_INFO_S i2cTransInfo: I2C通信情報
  * @retval なし
  * @brief  I2C通信を要求します。
  *         本関数がコールされ終わっても、通信は開始されません。
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_RequestI2c(I2C_TRANS_INFO_S i2cTransInfo)
{
    if(i2cTransInfo.bufSize <= 0) return;
    
    drvCs43l22.i2cTransInfo[drvCs43l22.i2cTransWi] = i2cTransInfo; /* I2C通信情報を積む */

    drvCs43l22.i2cTransWi++;
    drvCs43l22.i2cTransWi &= I2C_QUEUE_SIZE - 1; /* リミッタ */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_ExecuteI2cPeriodic
  * @param  なし
  * @retval なし
  * @brief  I2C通信要求があった場合、通信を実行します。
  *         本関数は周期的にコールしてください。
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_ExecuteI2cPeriodic(void)
{
    if(drvCs43l22.i2cMtx) return;  /* I2C通信が空いていなかったらここで抜ける */
    
    if(drvCs43l22.i2cTransWi == drvCs43l22.i2cTransRi) return; /* 要求がない場合は何もしない */
    
    drvCs43l22.i2cMtx = true;
    
    while(LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)); /* 前の転送完了待ち(一応) */
    
    LL_I2C_GenerateStartCondition(AUDIO_I2C); /* スタートコンディション発行 */
    
    LL_I2C_EnableIT_TX(AUDIO_I2C);/* I2C割り込み許可(EVT, BUF) */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2cEventHandler
  * @param  なし
  * @retval なし
  * @brief  I2Cイベントの割り込みハンドラです。

  * @author keitwo
********************************************************************************/
void DRV_CS43L22_I2cEventHandler(void)
{
    I2C_TRANS_INFO_S* i2cXferInfo = &drvCs43l22.i2cTransInfo[drvCs43l22.i2cTransRi];
    
    /* スタートコンディション発行完了 */
    if(LL_I2C_IsActiveFlag_SB(AUDIO_I2C) 
     &&LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
     &&LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
     && (i2cXferInfo->dir == I2C_DIR_TX || i2cXferInfo->dir == I2C_DIR_RX_WITH_ADDR_SELECT))
    {
        LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS); /* スレーブアドレス送信 */
        return;
    }
    
    /* スレーブアドレス送信完了 */
    if(LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C) 
    && LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
    && LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
    && LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
    && LL_I2C_GetTransferDirection(AUDIO_I2C)
    && (i2cXferInfo->dir == I2C_DIR_TX || i2cXferInfo->dir == I2C_DIR_RX_WITH_ADDR_SELECT))
    {
        LL_I2C_TransmitData8(AUDIO_I2C, i2cXferInfo->internalAddress | CODEC_MAP_INCR_BIT); /* オーディオコーデック内部アドレス送信(連続読み出しBit On) */
        drvCs43l22.i2cTransCnt = i2cXferInfo->bufSize;
        return;
    }
    
    if(i2cXferInfo->dir == I2C_DIR_TX) /* 送信*/
    {
        /* データ送信 */
        if(LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        && LL_I2C_GetTransferDirection(AUDIO_I2C)
        && drvCs43l22.i2cTransCnt != 0)
        {
            LL_I2C_TransmitData8(AUDIO_I2C, *i2cXferInfo->pBuf++);
            drvCs43l22.i2cTransCnt--;
            return;
        }
        
        /* 最終データ送信完了 */
        if(LL_I2C_IsActiveFlag_BTF(AUDIO_I2C))
        {
            LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
            LL_I2C_DisableIT_TX(AUDIO_I2C); /* TX割り込みOFF */
            if(i2cXferInfo->i2cCompFunc != NULL) i2cXferInfo->i2cCompFunc();
            DRV_CS43L22_I2cPostFunction();
        }
    }
    else /* 受信 */
    {
        i2cXferInfo->dir = I2C_DIR_RX; /* ここから受信に切り替え */
        /* オーディオコーデック内部アドレス送信完了 */
        if(LL_I2C_IsActiveFlag_TXE(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_MSL(AUDIO_I2C)
        && LL_I2C_GetTransferDirection(AUDIO_I2C) )
        {
            LL_I2C_GenerateStartCondition(AUDIO_I2C); /* リスタートコンディション発行 */
            return;
        }
        
        /* リスタートコンディション発行完了 */
        if(LL_I2C_IsActiveFlag_SB(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_MSL(AUDIO_I2C))
        {
            LL_I2C_TransmitData8(AUDIO_I2C, CODEC_SLAVE_ADDRESS | I2C_READ_BIT); /* スレーブアドレス送信(受信) */
            return;
        }
        
        /* スレーブアドレス送信完了(受信) */
        if(LL_I2C_IsActiveFlag_ADDR(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_BUSY(AUDIO_I2C)
        && LL_I2C_IsActiveFlag_MSL(AUDIO_I2C))
        {
            switch(i2cXferInfo->bufSize)
            {
            case 1:
                LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え(1byte受信の時は、ADDRフラグを落とす前!) */  
                (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。*/
                LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
                break;
            case 2:
                LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え */
                LL_I2C_EnableBitPOS(AUDIO_I2C); /* POS をHighに切り替え(現在のシフトレジスタのデータの次のバイトに対しACK bitの内容を適用する) */
                (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。*/
                break;
            default:
                (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
                break;
            }
            return;
        }
        
        /* 通信完了(RXNE判定より前!) */
        if(LL_I2C_IsActiveFlag_BTF(AUDIO_I2C))
        {
            switch(i2cXferInfo->bufSize)
            {
            case 1:
                break;
            case 2:
                LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
                *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 0格納 */
                *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 1格納 */
                LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
                LL_I2C_DisableBitPOS(AUDIO_I2C); /* POS をLowに戻す */
                LL_I2C_DisableIT_RX(AUDIO_I2C); /* RX割り込みOFF */
                if(i2cXferInfo->i2cCompFunc != NULL) i2cXferInfo->i2cCompFunc();
                DRV_CS43L22_I2cPostFunction();
                break;

            default:
                if(drvCs43l22.i2cTransCnt == 3)
                {
                    LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え */
                    *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte(N-3)格納 */
                    drvCs43l22.i2cTransCnt--;
                }
                else if(drvCs43l22.i2cTransCnt == 2)
                {
                    LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
                    *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte(N-2)格納 */
                    *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte(N-1)格納 */
                    LL_I2C_DisableIT_RX(AUDIO_I2C); /* RX割り込みOFF */
                    LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
                    if(i2cXferInfo->i2cCompFunc != NULL) i2cXferInfo->i2cCompFunc();
                    DRV_CS43L22_I2cPostFunction();
                }
                break;
            }
            return;
        }
        
        /* データ受信 */
        if(LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C))
        {
            switch(i2cXferInfo->bufSize)
            {
            case 1:
                *i2cXferInfo->pBuf = LL_I2C_ReceiveData8(AUDIO_I2C); /* データ格納 */
                LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
                LL_I2C_DisableIT_RX(AUDIO_I2C); /* RX割り込みOFF */
                if(i2cXferInfo->i2cCompFunc != NULL) i2cXferInfo->i2cCompFunc();
                DRV_CS43L22_I2cPostFunction();
                break;

            case 2:
                break;

            default:
                if(drvCs43l22.i2cTransCnt > 3)
                {
                    *i2cXferInfo->pBuf++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* データ格納 */
                    drvCs43l22.i2cTransCnt--;
                }
                break;
            }
            return;
        }
    }
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2CTestCallback
  * @param  なし
  * @retval なし
  * @brief  I2C読み書き関数のコールバックテスト関数です。
  * @author keitwo
********************************************************************************/
static bool testI2cComp; /*<! テスト関数内で割り込み処理を全て待つための変数 */
void DRV_CS43L22_I2CTestCallback(void)
{
    testI2cComp = true;
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2CTest
  * @param  なし
  * @retval int: テスト成功なら0, 失敗なら!0が返る
  * @brief  I2C読み書き関数のテスト関数です。
  *         ★オーディオコーデックのリセット解除およびI2Cペリフェラル初期化後にテストしてください。
  * @author keitwo
********************************************************************************/
static uint8_t readTest[4];
static uint8_t writeTest[4];
int DRV_CS43L22_I2CTest(void)
{
    int result = 0;
    int i;
    uint8_t codecReg;
    I2C_TRANS_INFO_S i2cTranInfo;
    
    /* --------------------------------
     ブロッキングI2C通信テスト 
    --------------------------------- */
    
    /* 1byte読み書きテスト */
    DRV_CS43L22_WriteRegOneByteWithWait(0x02, 0x9E);
    codecReg = DRV_CS43L22_ReadRegOneByteWithWait(0x02);

    /* 送受信データ比較 */
    if(codecReg != 0x9E)
    {
        result = !0;
    }
    
    /* 連続読み書きテスト */
    writeTest[0] = 0xCC;
    writeTest[1] = 0xDD;
    writeTest[2] = 0xEE;
    writeTest[3] = 0xFF;
    DRV_CS43L22_WriteRegsWithWait(0x20, writeTest, 4);
    readTest[0] = 0x00;
    readTest[1] = 0x00;
    readTest[2] = 0x00;
    readTest[3] = 0x00;
    DRV_CS43L22_ReadRegsWithWait(0x20, readTest, 4);
    
    /* 送受信データ比較 */
    for(i = 0; i < 4; i++)
    {
        if(writeTest[i] != readTest[i])
        {
            result = !0;
        }
    }
    
    /* --------------------------------
     ノンブロッキングI2C通信テスト 
    --------------------------------- */

    /* 送信テスト */
    writeTest[0] = 0x55;
    writeTest[1] = 0x66;
    writeTest[2] = 0x77;
    writeTest[3] = 0x88;
    i2cTranInfo.bufSize = 4;
    i2cTranInfo.dir = I2C_DIR_TX;
    i2cTranInfo.i2cCompFunc = DRV_CS43L22_I2CTestCallback;
    i2cTranInfo.internalAddress = 0x20;
    i2cTranInfo.pBuf = &writeTest[0];
    i2cTranInfo.slaveAddress = CODEC_SLAVE_ADDRESS;
    testI2cComp = false;
    DRV_CS43L22_RequestI2c(i2cTranInfo);
    DRV_CS43L22_ExecuteI2cPeriodic(); /* テストのためここで即時実行 */
    while(!testI2cComp); /* コールバック完了待ち */

    /* 受信テスト(1バイト) */
    readTest[0] = 0x00;
    i2cTranInfo.bufSize = 1;
    i2cTranInfo.dir = I2C_DIR_RX_WITH_ADDR_SELECT;
    i2cTranInfo.i2cCompFunc = DRV_CS43L22_I2CTestCallback;
    i2cTranInfo.internalAddress = 0x20;
    i2cTranInfo.pBuf = &readTest[0];
    i2cTranInfo.slaveAddress = CODEC_SLAVE_ADDRESS;
    testI2cComp = false;
    DRV_CS43L22_RequestI2c(i2cTranInfo);
    DRV_CS43L22_ExecuteI2cPeriodic(); /* テストのためここで即時実行 */
    while(!testI2cComp); /* コールバック完了待ち */
   
    /* 送受信データ比較 */
    for(i = 0; i < 1; i++)
    {
        if(writeTest[i] != readTest[i])
        {
            result = !0;
        }
    }
    
    /* 受信テスト(2バイト) */
    readTest[0] = 0x00;
    readTest[1] = 0x00;
    i2cTranInfo.bufSize = 2;
    i2cTranInfo.dir = I2C_DIR_RX_WITH_ADDR_SELECT;
    i2cTranInfo.i2cCompFunc = DRV_CS43L22_I2CTestCallback;
    i2cTranInfo.internalAddress = 0x20;
    i2cTranInfo.pBuf = &readTest[0];
    i2cTranInfo.slaveAddress = CODEC_SLAVE_ADDRESS;
    testI2cComp = false;
    DRV_CS43L22_RequestI2c(i2cTranInfo);
    DRV_CS43L22_ExecuteI2cPeriodic(); /* テストのためここで即時実行 */
    while(!testI2cComp); /* コールバック完了待ち */
    
    /* 送受信データ比較 */
    for(i = 0; i < 2; i++)
    {
        if(writeTest[i] != readTest[i])
        {
            result = !0;
        }
    }

    /* 受信テスト(3バイト以上) */
    readTest[0] = 0x00;
    readTest[1] = 0x00;
    readTest[2] = 0x00;
    readTest[3] = 0x00;
    i2cTranInfo.bufSize = 4;
    i2cTranInfo.dir = I2C_DIR_RX_WITH_ADDR_SELECT;
    i2cTranInfo.i2cCompFunc = DRV_CS43L22_I2CTestCallback;
    i2cTranInfo.internalAddress = 0x20;
    i2cTranInfo.pBuf = &readTest[0];
    i2cTranInfo.slaveAddress = CODEC_SLAVE_ADDRESS;
    testI2cComp = false;
    DRV_CS43L22_RequestI2c(i2cTranInfo);
    DRV_CS43L22_ExecuteI2cPeriodic(); /* テストのためここで即時実行 */
    while(!testI2cComp); /* コールバック完了待ち */

    /* 送受信データ比較 */
    for(i = 0; i < 4; i++)
    {
        if(writeTest[i] != readTest[i])
        {
            result = !0;
        }
    }

    return result;
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2sDmaHandler
  * @param  なし
  * @retval なし
  * @brief  I2S DMAの割り込みハンドラです。

  * @author keitwo
********************************************************************************/
void DRV_CS43L22_I2sDmaHandler(void)
{
    /* 半分転送完了 */
    if(LL_DMA_IsActiveFlag_HT7(AUDIO_I2S_DMA))
    {
        LL_DMA_ClearFlag_HT7(AUDIO_I2S_DMA);
        /* ここに楽音合成処理を記載To Do */
    }
    
    /* 転送完了 */
    if(LL_DMA_IsActiveFlag_TC7(AUDIO_I2S_DMA))
    {
        LL_DMA_ClearFlag_TC7(AUDIO_I2S_DMA);
        /* ここに楽音合成処理を記載To Do */
    }
}

/**
  * @}
  */ /* End drv_cs43l22 Public Functions */

/**
  * @}
  */ /* End addtogroup drv_cs43l22 */

/**
  * @}
  */ /* End addtogroup drv_cs43l22Group */

/************************ (C) COPYRIGHT keitwo *****END OF FILE****/
