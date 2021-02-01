/**
* @file    S2LP_Interface.h
* @author  CLAB
* @version V1.0.0
* @date    03-April-2017
* @brief   Interface functions for S2LP.
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __S2LP_INTERFACE_H
#define __S2LP_INTERFACE_H


/* Includes ------------------------------------------------------------------*/
#include "S2LP_Gpio.h"
#include "S2LP_Radio.h"
#include "S2LP_Util.h"
#include "S2LP_Regs.h"
#include "S2LP_Types.h"
#ifdef __cplusplus
  "C" {
#endif


/**
 * @addtogroup S2LP_INTERFACE
 * @{
 */
    
typedef struct 
{
  uint16_t nS2LP_Version;
}S2LP_VersionMap;
   
#define CUT_MAX_NO 3
#define CUT_2_1v3 0x0103
#define CUT_2_1v4 0x0104
#define CUT_3_0   0x0130


/**
 * @addgroup S2LP_INTERFACE_FUNCTIONS
 * @{
 */
void S2LP_ManagementIdentificationRFBoard(void);

RangeExtType S2LP_ManagementGetRangeExtender(void);
void S2LP_ManagementSetRangeExtender(RangeExtType xRangeType);
void S2LP_ManagementRangeExtInit(void);
void S2LP_ManagementSetBand(uint8_t value);
uint8_t S2LP_ManagementGetBand(void);

uint8_t SdkEvalGetHasEeprom(void);

void S2LP_InterfaceInit(void);
void S2LPInterfaceInit(void);
void S2LPGpioIrqInit(S2LPIrqs *pGpioIRQ);
void S2LP_RadioInit(SRadioInit *pRadioInit);
void S2LP_PacketConfig(void);
void S2LP_SetPayloadlength(uint8_t length);
void S2LP_SetDestinationAddress(uint8_t address);
void S2LP_EnableTxIrq(void);
void S2LP_EnableRxIrq(void);
void S2LP_DisableIrq(void);
void S2LP_SetRxTimeout(float cRxTimeOut);
void S2LP_EnableSQI(void);
void S2LP_SetRssiTH(int dbmValue);
float S2LP_GetRssiTH(void);
void S2LP_StartRx(void);
void S2LP_GetRxPacket(uint8_t *buffer, uint8_t *size );
void S2LP_StartTx(uint8_t *buffer, uint8_t size);


void SdkEvalSetHasEeprom(uint8_t eeprom);
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#ifdef __cplusplus
}
#endif


#endif


 /******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

