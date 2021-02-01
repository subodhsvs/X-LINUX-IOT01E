/**
  ******************************************************************************
  * @file    SPIRIT_Types.c
  * @author  AMG - RF Application team
  * @version 3.2.4
  * @date    26-September-2016
  * @brief   File for SPIRIT types.
  * @details
  *
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

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Types.h"
#include "MCU_Interface.h"


/** @addtogroup SPIRIT_Libraries
 * @{
 */


/** @addtogroup SPIRIT_Types
 * @{
 */


/** @defgroup Types_Private_TypesDefinitions    Types Private Types Definitions
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Defines             Types Private Defines
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Macros               Types Private Macros
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Variables             Types Private Variables
 * @{
 */

/**
 * @brief  Spirit Status global variable.
 *         This global variable of @ref SpiritStatus type is updated on every SPI transaction
 *         to maintain memory of Spirit Status.
 */

volatile SpiritStatus g_xStatus;

/**
 * @}
 */



/** @defgroup Types_Private_FunctionPrototypes       Types Private FunctionPrototypes
 * @{
 */



/**
 * @}
 */



/** @defgroup Types_Private_Functions                 Types Private Functions
 * @{
 */

#ifdef  SPIRIT_USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file  pointer to the source file name
 * @param line  assert_param error line source number
 * @retval : None
 */
void s_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#elif SPIRIT_USE_VCOM_ASSERT

#include "SDK_EVAL_VC_General.h"

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line  assert_param error line source number
 * @param expression: string representing the assert failed expression
 * @retval : None
 */
void s_assert_failed(uint8_t* file, uint32_t line, char* expression)
{

  printf("\n\rVCOM DEBUG: Incorrect parameter. Please reboot.\n\r");
  printf("%s:%d \n\r",file,line);
  printf("The expression %s returned FALSE.\n\r", expression);

  /* Infinite loop */
  while (1)
  {
  }
}

#elif SPIRIT_USE_FRAME_ASSERT

#include "SdkUsbProtocol.h"

/**
 * @brief Sends a notify frame with a payload indicating the name
 *        of the assert failed.
 * @param expression: string representing the assert failed expression
 * @retval : None
 */
void s_assert_failed(char* expression)
{
  char pcPayload[100];
  uint16_t i;

  for(i = 0 ; expression[i]!='(' ; i++);
  expression[i]='\0';

  strcpy(pcPayload, &expression[3]);

  //sprintf(pcPayload, "The expression %s returned FALSE.\n\r", expression);
  SpiritNotifyAssertFailed(pcPayload);

}

#endif


/**
 * @brief  Updates the gState (the global variable used to maintain memory of Spirit Status)
 *         reading the MC_STATE register of SPIRIT.
 * @param  None
 * @retval None
 */
void SpiritRefreshStatus(void)
{
  uint8_t tempRegValue[2];
  //uint16_t *ptr = (uint16_t*)&g_xStatus;

  /* Read the status both from register and from SPI header and exit when they match.
      This will protect against possible transition state changes */

  do
  {
    /* Reads the MC_STATUS register to update the g_xStatus */
    g_xStatus = SpiritSpiReadRegisters(MC_STATE1_BASE, 2, &tempRegValue[0]);
    printf("\nLOG : First time g_xStatus.MC_STATE = %d  \n",(uint8_t)g_xStatus.MC_STATE);
  }
  while(!((((uint8_t*)&g_xStatus)[0])==tempRegValue[1] &&
          (((uint8_t*)&g_xStatus)[1]&0x0F)==tempRegValue[0]));

  printf("\nLOG : Second time g_xStatus.MC_STATE = %d  \n",(uint8_t)g_xStatus.MC_STATE);

/*svs: Only for testing
  SpiritStatus LocalStatus[10];
  uint8_t LocalTempRegValue[10] = {0};
  printf("\nSUBODH Start Reading \n");
  LocalStatus[0] = SpiritSpiReadRegisters(0xC2, 1, &LocalTempRegValue[0]);
  LocalStatus[1] = SpiritSpiReadRegisters(0xC3, 1, &LocalTempRegValue[1]);
  LocalStatus[2] = SpiritSpiReadRegisters(0xC4, 1, &LocalTempRegValue[2]);
  LocalStatus[3] = SpiritSpiReadRegisters(0xC5, 1, &LocalTempRegValue[3]);
  LocalStatus[4] = SpiritSpiReadRegisters(0xC6, 1, &LocalTempRegValue[4]);

  LocalStatus[5] = SpiritSpiReadRegisters(0xC7, 1, &LocalTempRegValue[5]);
  LocalStatus[6] = SpiritSpiReadRegisters(0xC8, 1, &LocalTempRegValue[6]);
  LocalStatus[7] = SpiritSpiReadRegisters(0xC9, 1, &LocalTempRegValue[7]);
  LocalStatus[8] = SpiritSpiReadRegisters(0xCA, 1, &LocalTempRegValue[8]);
  LocalStatus[9] = SpiritSpiReadRegisters(0xCB, 1, &LocalTempRegValue[9]);

  for(volatile uint32_t index =0; index <= 9; index++)
  {
    printf("\n Reg Address : 0xC%d | LocalStatus = 0x%x | LocalTempRegValue = 0x%x%x",index+2, LocalStatus[index],(uint16_t)LocalTempRegValue[index]);
  }

  printf("\nSUBODH End Reading \n");
  //while(1);
  */

}

/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
