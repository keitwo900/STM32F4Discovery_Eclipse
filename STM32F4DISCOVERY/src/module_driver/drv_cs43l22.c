/**
  ******************************************************************************
  * @file    drv_cs43l22.c
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

/* Includes ------------------------------------------------------------------*/

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
#define AUDIO_RESET_GPIO    GPIOD
#define AUDIO_RESET_PIN     LL_GPIO_PIN_4

#define AUDIO_I2C_GPIO  GPIOB
#define AUDIO_SCL_PIN   LL_GPIO_PIN_6
#define AUDIO_SDA_PIN   LL_GPIO_PIN_9
#define AUDIO_I2C       I2C1

#define AUDIO_I2S_GPIO1     GPIOC /* Master Clock, Bit Clock, Serial Data */
#define AUDIO_I2S_GPIO2     GPIOA /* Word Clock */
#define AUDIO_I2S_MCK_PIN   LL_GPIO_PIN_7 /* PC7 */
#define AUDIO_I2S_BCK_PIN   LL_GPIO_PIN_10 /* PC10 */
#define AUDIO_I2S_SD_PIN    LL_GPIO_PIN_12 /* PC12 */
#define AUDIO_I2S_WCK_PIN   LL_GPIO_PIN_4 /* PA4 */
#define AUDIO_I2S           SPI3

#define AUDIO_I2S_DMA               DMA1
#define AUDIO_I2S_DMA_STREAM        LL_DMA_STREAM_7
#define AUDIO_I2S_DMA_CH            LL_DMA_CHANNEL_0
#define DRV_CS43L22_I2sDmaHandler   DMA1_Stream7_IRQHandler

#define CODEC_SLAVE_ADDRESS 0x94  /* b1001010r/w */
#define I2C_READ_BIT 0x01 /* リードはLSBが1 */
#define CODEC_MAP_INCR_BIT 0x80 /* 内部アドレスバイトのMSBに1を立てると、インクリメント書き込み、読み出しが可能になる */

#define DEFAULT_VOLUME  -96 /* 最初に設定するマスターボリューム */

/**
  * @}
  */ /* End drv_cs43l22 Private Macros */

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
    //NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    //NVIC_EnableIRQ(I2C1_EV_IRQn);
    //NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    //NVIC_EnableIRQ(I2C1_ER_IRQn);
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
    DRV_CS43L22_WriteRegisterOneByte(0x20, (uint8_t)vol_dB); 
    DRV_CS43L22_WriteRegisterOneByte(0x21, (uint8_t)vol_dB);
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
    DRV_CS43L22_WriteRegisterOneByte(0x02, 0x01); /* コーデックパワーオフ */
    DRV_CS43L22_WriteRegisterOneByte(0x04, 0xAF); /* スピーカーオフ、ヘッドフォンON*/
    DRV_CS43L22_WriteRegisterOneByte(0x05, 0x81); /* クロック設定: 自動検出 */
    DRV_CS43L22_WriteRegisterOneByte(0x06, 0x04); /* I2S規格: I2Sフィリップス標準*/
    
    DRV_CS43L22_VolumeCtrl(DEFAULT_VOLUME); /* ボリューム初期値設定 */
    
    DRV_CS43L22_WriteRegisterOneByte(0x02, 0x9E); /* コーデックパワーオン */
    DRV_CS43L22_WriteRegisterOneByte(0x0A, 0x00); /* アナログソフトランプ無効化 */
    DRV_CS43L22_WriteRegisterOneByte(0x0E, 0x04); /* デジタルソフトランプ無効化*/
    DRV_CS43L22_WriteRegisterOneByte(0x27, 0x00); /* アタックレベルのリミッタ無効か */
    DRV_CS43L22_WriteRegisterOneByte(0x1F, 0x0F); /* 低音域、高音域調整 */
    DRV_CS43L22_WriteRegisterOneByte(0x1A, 0x0A); /* PCMボリュームレベル調整 */
    DRV_CS43L22_WriteRegisterOneByte(0x1B, 0x0A); /* PCMボリュームレベル調整 */
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
    
    
    //DRV_CS43L22_I2CTest();
}

/*******************************************************************************/
/** @par    DRV_CS43L22_ReadRegisterOneByte
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @retval uint8_t: 読み出しデータ
  * @brief  I2Cでオーディオコーデックのレジスタからデータを1byte読み出します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
uint8_t DRV_CS43L22_ReadRegisterOneByte(uint8_t internalAddr)
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
    
    while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
    
    LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。 */
        
    return result;
}

/*******************************************************************************/
/** @par    DRV_CS43L22_ReadRegisters
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t* data: 読み出しデータを格納する配列の先頭ポインタ
  * @param  int size: データサイズ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタからデータを読み出します。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_ReadRegisters(uint8_t internalAddr, uint8_t* data, int size)
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
           
        while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
            
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
        
        break;
        
    case 2:
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_NACK); /* NACK送信に切り替え */
        LL_I2C_EnableBitPOS(AUDIO_I2C); /* POS をHighに切り替え(現在のシフトレジスタのデータの次のバイトに対しACK bitの内容を適用する) */
        
        (void)AUDIO_I2C->SR2; /* SR1に続いてSR2を読みだすと、ADDRがクリアされる。ADDRがクリアされることによって、受信が開始される。 */
        
        while(!LL_I2C_IsActiveFlag_BTF(AUDIO_I2C)); /* データ受信完了待ち(DRにbyte 0、シフトレジスタにbyte 1が来るのを待つ) */
        
        LL_I2C_GenerateStopCondition(AUDIO_I2C); /* ストップコンディション発行 */
         
        *data++ = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 0格納 */
        while(!LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C)); /* データ受信待ち */
        *data = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte 1格納 */
        
        while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
        
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
        while(!LL_I2C_IsActiveFlag_RXNE(AUDIO_I2C)); /* データ受信待ち */
        *data = LL_I2C_ReceiveData8(AUDIO_I2C); /* byte (N-1)格納 */
        
        while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。)受信の場合はデータ転送後に確認! */
        
        LL_I2C_AcknowledgeNextData(AUDIO_I2C, LL_I2C_ACK); /* ACK送信に戻す。*/
        
        break;
    }
}


/*******************************************************************************/
/** @par    DRV_CS43L22_WriteRegisterOneByte
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t data: 書き込みデータ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタにデータを1byte書き込みます。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_WriteRegisterOneByte(uint8_t internalAddr, uint8_t data)
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
    while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_WriteRegisters
  * @param  uint8_t internalAddr: オーディオコーデック内部アドレス
  * @param  uint8_t* data: データの先頭ポインタ
  * @param  int size: データサイズ
  * @retval なし
  * @brief  I2Cでオーディオコーデックのレジスタにデータを書き込みます。
  *         割り込みを使用しないで制御します。(ブロッキング処理)
  * @author keitwo
********************************************************************************/
void DRV_CS43L22_WriteRegisters(uint8_t internalAddr, uint8_t* data, int size)
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
    while(AUDIO_I2C->CR1 & I2C_CR1_STOP); /* ストップコンディション発行完了待ち(CR1であることに注意! SR1のSTOPビットは意味が違う。) */
}

/*******************************************************************************/
/** @par    DRV_CS43L22_I2CTest
  * @param  なし
  * @retval int: テスト成功なら0, 失敗なら!0が返る
  * @brief  I2C読み書き関数のテスト関数です。
  *         ★オーディオコーデックのリセットを解除後にテストしてください。
  * @author keitwo
********************************************************************************/
int DRV_CS43L22_I2CTest(void)
{
    int result = 0;
    int i;
    uint8_t codecReg;
    uint8_t readTest[4];
    uint8_t writeTest[4];
    
    /* 1byte読み書きテスト */
    codecReg = DRV_CS43L22_ReadRegisterOneByte(0x02);
    DRV_CS43L22_WriteRegisterOneByte(0x02, 0x9E);
    codecReg = DRV_CS43L22_ReadRegisterOneByte(0x02);

    if(codecReg != 0x9E)
    {
        result = !0;
    }
    
    /* 連続読み出しテスト(1byte, 2byte, 3byte) */
    DRV_CS43L22_ReadRegisters(0x01, readTest, 1);
    DRV_CS43L22_ReadRegisters(0x03, readTest, 2);
    DRV_CS43L22_ReadRegisters(0x01, readTest, 3);
    
    /* 連続読み書きテスト */
    DRV_CS43L22_ReadRegisters(0x20, readTest, 4);
    writeTest[0] = 0xCC;
    writeTest[1] = 0xDD;
    writeTest[2] = 0xEE;
    writeTest[3] = 0xFF;
    DRV_CS43L22_WriteRegisters(0x20, writeTest, 4);
    DRV_CS43L22_ReadRegisters(0x20, readTest, 4);
    
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
