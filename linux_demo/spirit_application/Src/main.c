/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include <stdio.h>
#include <unistd.h>
#include <linux/time.h>
#include "platform.h"
#include "st_errno.h"
//#include "rfal_analogConfig.h"
//#include "rfal_nfca.h"
//#include "example_poller.h"
//#include "test.c"

/*svs start*/
//#include "cube_hal.h"
#include "radio_shield_config.h"
#include "radio_appli.h"
#include "p2p_lib.h"
/*svs end */

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/*svs Start */
#define USE_STM32F4XX_NUCLEO
#define USE_HAL_DRIVER
#define STM32F401xE
#define P2P_DEMO
#define SPIRIT1_ST_SHIELD
#define USE_SPIRIT1_DEFAULT
#define STM32F4
#define NO_EEPROM
#define X_NUCLEO_IDS01A4
#define USE_STDPERIPH_DRIVER
#define USE_SYSTICK_DELAY
/*svs End */

#define logUsart	printf
/*svs Start*/
#define TX_BUFFER_SIZE   20
#define RX_BUFFER_SIZE   96

uint8_t TxLength = TX_BUFFER_SIZE;
uint8_t RxLength = 0;
uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,\
  16,17,18,19,20};
uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};
/*svs End */


/*
 ******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************
 */
int main(void)
{
	//logUsart("Welcome to the ST25R3916 NFC Poller Demo on Linux..\n");
	//set terminal to line buffer so as to update charecter by charecter

	setlinebuf(stdout);
	int ret = 0;

	/* Initialize the platform */
	/* Initialize GPIO */
	ret = gpio_init();
	if(ret != ERR_NONE)
  {
    printf("\nSVSL LOG error in gpio_init()\n ");
    goto error;
  }
  else
    printf("\nSVSL LOG: gpio_init() successful\n ");

/*svs: initialize the GPIO for SDN, INT */

	/* Initialize SPI */
	ret = spi_init();

	if(ret != ERR_NONE)
  {
    printf("\nSVS LOG error in spi_init()\n ");
    goto error;
  }
  else
    printf("\nSVS LOG: spi_init() successful\n ");

	/* Initialize interrupt mechanism */
	ret = interrupt_init();
	if (ret != ERR_NONE)
  {
    printf("\nSVSL LOG error in interrupt_init()\n ");
    goto error;
  }
  else
    printf("\nSVSL LOG: interrupt_init() successful\n ");
	/* Initialize rfal and run example code for NFC */

/*	exampleRfalPollerRun(); */ /*svs :	commented for disabling NFC and adding
																			SPIRIT1 application code */

  HAL_Radio_Init(); // svs: calls only the RadioInterfaceInit()
  printf("\n  LOG: after HAL_Radio_Init() \n");
	P2P_Init();
  printf("\n  LOG: after P2P_Init() \n");
  uint32_t index = 0;
	while (1)
	{
    printf("\n inside while(1) #%d\n",index++);

    usleep(1000*1000); /*svs: Check this as well, does it affect the User State Machine  */

    /* svs: Logging data for testing purpose only . . . */
		P2P_Process(aTransmitBuffer, TxLength, aReceiveBuffer, RxLength);
	}

error:
	return 0;
}
