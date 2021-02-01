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
/*! \file
 *
 *  \author
 *
 *  \brief Platform header file. Defining platform independent functionality.
 *
 */

/*
 *      PROJECT:
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file platform.h
 *
 *  \author Gustavo Patricio
 *
 *  \brief Platform specific definition layer
 *
 *  This should contain all platform and hardware specifics such as
 *  GPIO assignment, system resources, locks, IRQs, etc
 *
 *  Each distinct platform/system/board must provide this definitions
 *  for all SW layers to use
 *
 *  Modified for Linux platform.
 *
 */

#ifndef PLATFORM_H
#define PLATFORM_H

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "st_errno.h"
#include "pltf_timer.h"
#include "pltf_spi.h"
#include "pltf_gpio.h"
//#include "st25r3911_interrupt.h"

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
#define LED_NFCA_PIN                1    // MCU_LED3 PE1
#define LED_NFCB_PIN                10   // MCU_LED2 PE10
#define LED_NFCF_PIN                13   // MCU_LED1 PF13
#define LED_NFCV_PIN                3    // MCU_LED4 PC3
#define LED_AP2P_PIN                12   // MCU_LED5 PF12

#define LED_NFCA_PORT               4
#define LED_NFCB_PORT               4
#define LED_NFCF_PORT               5
#define LED_NFCV_PORT               2

#if 0
#define LED_FIELD_Pin               49   // MCU_LED6 PD1
#else
  #define LED_FIELD_Pin             1   // MCU_LED6 PD1
#endif
#define LED_FIELD_GPIO_Port         3

#define ST25R_SS_PIN                                                /*!< GPIO pin used for ST25R391X SPI SS */
#define ST25R_SS_PORT                                               /*!< GPIO port used for ST25R391X SPI SS port */

#define ST25R_INT_PIN               10 /*Pin 10 */                 /*!< GPIO pin used for SPIRIT1_GPIO3 Interrupt PB10*/
#define ST25R_INT_PORT              1 /*Port B */                 /*!< GPIO port used for SPIRIT1_GPIO3 Interrupt - PB10*/

#ifdef LED_FIELD_Pin
  #define PLATFORM_LED_FIELD_PIN          LED_FIELD_Pin             /*!< GPIO pin used as field LED */
#endif

#ifdef LED_FIELD_GPIO_Port
  #define PLATFORM_LED_FIELD_PORT         LED_FIELD_GPIO_Port       /*!< GPIO port used as field LED */
#endif


/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define ST25R_COM_SINGLETXRX                                                    /*!< Enable single SPI frame transmission */

#define platformProtectST25RComm()            pltf_protect_com()
#define platformUnprotectST25RComm()          pltf_unprotect_com()

#define platformProtectWorker()               pltf_protect_worker()             /*!< Protect RFAL Worker/Task/Process from concurrent execution on multi thread platforms   */
#define platformUnprotectWorker()             pltf_unprotect_worker()           /*!< Unprotect RFAL Worker/Task/Process from concurrent execution on multi thread platforms */

#define platformProtectST25RIrqStatus()       pltf_protect_interrupt_status()   /*!< Acquire the lock for safe access of RFAL interrupt status variable */
#define platformUnprotectST25RIrqStatus()     pltf_unprotect_interrupt_status() /*!< Release the lock aquired for safe accessing of RFAL interrupt status variable */

#define platformSpiSelect()                                                     /*!< SPI SS\CS: Chip|Slave Select */
#define platformSpiDeselect()                                                   /*!< SPI SS\CS: Chip|Slave Deselect */

#define platformIsr()                         st25r3911Isr()

#define platformGpioSet(port, pin)            gpio_set(port, pin)                                                                           /*!< Turns the given GPIO High */
#define platformGpioClear(port, pin)          gpio_clear(port, pin)                                                                         /*!< Turns the given GPIO Low  */
#define platformGpioToggle(port, pin)         (platformGpioIsHigh(port, pin) ? platformGpioClear( port, pin) : platformGpioSet(port, pin))  /*!< Toogles the given GPIO    */
#define platformGpioIsHigh(port, pin)         (gpio_readpin(port, pin) == GPIO_PIN_SET)                                                     /*!< Checks if the given GPIO is High */
#define platformGpioIsLow(port, pin)          (!platformGpioIsHigh(port, pin))                                                              /*!< Checks if the given GPIO is Low  */

#define platformLedsInitialize()                                                /*!< Initializes the pins used as LEDs to outputs*/
#define platformLedOff(port, pin)              // platformGpioClear(port, pin)  /*!< Turns the given LED Off */
#define platformLedOn(port, pin)               // platformGpioSet(port, pin)    /*!< Turns the given LED On  */

#define platformTimerCreate(t)                timerCalculateTimer(t)    /*!< Create a timer with the given time (ms)     */
#define platformTimerIsExpired(timer)         timerIsExpired(timer)     /*!< Checks if the given timer is expired        */
#define platformDelay(t)                      timerDelay(t)             /*!< Performs a delay for the given time (ms)    */
#define platformGetSysTick()                  platformGetSysTick_linux()/*!< Get System Tick ( 1 tick = 1 ms)            */

#define platformSpiTxRx(txBuf, rxBuf, len)    spiTxRx(txBuf, rxBuf, len)/*!< SPI transceive */


/*
******************************************************************************
* RFAL FEATURES CONFIGURATION
******************************************************************************
*/

#define RFAL_FEATURE_LISTEN_MODE               false      /*!< Enable/Disable RFAL support for Listen Mode                               */
#define RFAL_FEATURE_WAKEUP_MODE               true       /*!< Enable/Disable RFAL support for the Wake-Up mode                          */
#define RFAL_FEATURE_NFCA                      true       /*!< Enable/Disable RFAL support for NFC-A (ISO14443A)                         */
#define RFAL_FEATURE_NFCB                      true       /*!< Enable/Disable RFAL support for NFC-B (ISO14443B)                         */
#define RFAL_FEATURE_NFCF                      true       /*!< Enable/Disable RFAL support for NFC-F (FeliCa)                            */
#define RFAL_FEATURE_NFCV                      true       /*!< Enable/Disable RFAL support for NFC-V (ISO15693)                          */
#define RFAL_FEATURE_T1T                       true       /*!< Enable/Disable RFAL support for T1T (Topaz)                               */
#define RFAL_FEATURE_T2T                       true       /*!< Enable/Disable RFAL support for T2T                                       */
#define RFAL_FEATURE_T4T                       true       /*!< Enable/Disable RFAL support for T4T                                       */
#define RFAL_FEATURE_ST25TB                    true       /*!< Enable/Disable RFAL support for ST25TB                                    */
#define RFAL_FEATURE_DYNAMIC_ANALOG_CONFIG     true       /*!< Enable/Disable Analog Configs to be dynamically updated (RAM)             */
#define RFAL_FEATURE_DPO                       false      /*!< Enable/Disable RFAL dynamic power support                                 */
#define RFAL_FEATURE_ISO_DEP                   true       /*!< Enable/Disable RFAL support for ISO-DEP (ISO14443-4)                      */
#define RFAL_FEATURE_ISO_DEP_POLL              true       /*!< Enable/Disable RFAL support for Poller mode (PCD) ISO-DEP (ISO14443-4)    */
#define RFAL_FEATURE_ISO_DEP_LISTEN            false      /*!< Enable/Disable RFAL support for Listen mode (PICC) ISO-DEP (ISO14443-4)   */
#define RFAL_FEATURE_NFC_DEP                   true       /*!< Enable/Disable RFAL support for NFC-DEP (NFCIP1/P2P)                      */


/*svs: adding code from Gustavo Patricio*/
#define RFAL_FEATURE_ISO_DEP_IBLOCK_MAX_LEN    256U       /*!< ISO-DEP I-Block max length. Please use values as defined by rfalIsoDepFSx */
#define RFAL_FEATURE_NFC_DEP_BLOCK_MAX_LEN     254U       /*!< NFC-DEP Block/Payload length. Allowed values: 64, 128, 192, 254           */
#define RFAL_FEATURE_NFC_RF_BUF_LEN            258U       /*!< RF buffer length used by RFAL NFC layer                                   */
#define RFAL_FEATURE_ISO_DEP_APDU_MAX_LEN      512U       /*!< ISO-DEP APDU max length.                                                  */
#define RFAL_FEATURE_NFC_DEP_PDU_MAX_LEN       512U       /*!< NFC-DEP PDU max length.                                                   */
/*svs: end addition*/




/*
 ******************************************************************************
 * RFAL OPTIONAL MACROS            (Do not modify)
 ******************************************************************************
 */

#ifndef platformProtectST25RIrqStatus
    #define platformProtectST25RIrqStatus()            /*!< Protect unique access to IRQ status var - IRQ disable on single thread environment (MCU) ; Mutex lock on a multi thread environment */
#endif /* platformProtectST25RIrqStatus */

#ifndef platformUnprotectST25RIrqStatus
    #define platformUnprotectST25RIrqStatus()          /*!< Unprotect the IRQ status var - IRQ enable on a single thread environment (MCU) ; Mutex unlock on a multi thread environment         */
#endif /* platformUnprotectST25RIrqStatus */

#ifndef platformProtectWorker
    #define platformProtectWorker()                    /* Protect RFAL Worker/Task/Process from concurrent execution on multi thread platforms   */
#endif /* platformProtectWorker */

#ifndef platformUnprotectWorker
    #define platformUnprotectWorker()                  /* Unprotect RFAL Worker/Task/Process from concurrent execution on multi thread platforms */
#endif /* platformUnprotectWorker */

#ifndef platformIrqST25RPinInitialize
    #define platformIrqST25RPinInitialize()            /*!< Initializes ST25R IRQ pin                     */
#endif /* platformIrqST25RPinInitialize */

#ifndef platformIrqST25RSetCallback
    #define platformIrqST25RSetCallback( cb )          /*!< Sets ST25R ISR callback                       */
#endif /* platformIrqST25RSetCallback */

#ifndef platformLedsInitialize
    #define platformLedsInitialize()                   /*!< Initializes the pins used as LEDs to outputs  */
#endif /* platformLedsInitialize */

#ifndef platformLedOff
    #define platformLedOff( port, pin )                /*!< Turns the given LED Off                       */
#endif /* platformLedOff */

#ifndef platformLedOn
    #define platformLedOn( port, pin )                 /*!< Turns the given LED On                        */
#endif /* platformLedOn */

#ifndef platformLedToogle
    #define platformLedToogle( port, pin )             /*!< Toggles the given LED                         */
#endif /* platformLedToogle */

#ifndef platformGetSysTick
    #define platformGetSysTick()                       /*!< Get System Tick ( 1 tick = 1 ms)              */
#endif /* platformGetSysTick */

#ifndef platformTimerDestroy
    #define platformTimerDestroy( timer )              /*!< Stops and released the given timer            */
#endif /* platformTimerDestroy */

//#ifndef platformLog
//    #define platformLog(...)                           /*!< Log method                                    */
//#endif /* platformLog */

#ifndef platformAssert
    #define platformAssert( exp )                      /*!< Asserts whether the given expression is true */
#endif /* platformAssert */

#ifndef platformErrorHandle
    #define platformErrorHandle()                      /*!< Global error handler or trap                 */
#endif /* platformErrorHandle */

#ifdef RFAL_USE_I2C

    #ifndef platformSpiTxRx
        #define platformSpiTxRx( txBuf, rxBuf, len )   /*!< SPI transceive                               */
    #endif /* platformSpiTxRx */

#else /* RFAL_USE_I2C */

    #ifndef platformI2CTx
        #define platformI2CTx( txBuf, len, last, txOnly ) /*!< I2C Transmit                               */
    #endif /* platformI2CTx */

    #ifndef platformI2CRx
        #define platformI2CRx( txBuf, len )            /*!< I2C Receive                                  */
    #endif /* platformI2CRx */

    #ifndef platformI2CStart
        #define platformI2CStart()                     /*!< I2C Start condition                          */
    #endif /* platformI2CStart */

    #ifndef platformI2CStop
        #define platformI2CStop()                      /*!< I2C Stop condition                           */
    #endif /* platformI2CStop */

    #ifndef platformI2CRepeatStart
        #define platformI2CRepeatStart()               /*!< I2C Repeat Start                             */
    #endif /* platformI2CRepeatStart */

    #ifndef platformI2CSlaveAddrWR
        #define platformI2CSlaveAddrWR(add)            /*!< I2C Slave address for Write operation        */
    #endif /* platformI2CSlaveAddrWR */

    #ifndef platformI2CSlaveAddrRD
        #define platformI2CSlaveAddrRD(add)            /*!< I2C Slave address for Read operation         */
    #endif /* platformI2CSlaveAddrRD */

#endif /* RFAL_USE_I2C */

/******************************************************************************/


#endif /* PLATFORM_H */
