#ifndef _P2P_LIB_H_
#define _P2P_LIB_H_
#include "MCU_Interface.h"

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#include "SPIRIT1_Util.h"
#endif
#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
#include "s2lp_interface.h"
#include "S2LP_Util.h"
#include "S2LP_Config.h"
#endif
/*---------appli.h-----*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
//#define       RADIO_Csma.h      SPIRIT_Csma

#endif
#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)
//#define      RADIO_Csma      S2LP_Csma


#endif



#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)

#define        HAL_Radio_Init(void)          HAL_Spirit1_Init(void)

#define        RadioInterfaceInit             Spirit1InterfaceInit
#define        RadioGpioIrqInit               Spirit1GpioIrqInit
#define        RadioRadioInit                 Spirit1RadioInit
#define        RadioSetPower                  Spirit1SetPower
#define        RadioPacketConfig             Spirit1PacketConfig
#define        RadioSetPayloadlength         Spirit1SetPayloadlength
#define        RadioSetDestinationAddress    Spirit1SetDestinationAddress
#define        RadioEnableTxIrq              Spirit1EnableTxIrq
#define        RadioEnableRxIrq              Spirit1EnableRxIrq
#define        RadioDisableIrq               Spirit1DisableIrq
#define        RadioSetRxTimeout             Spirit1SetRxTimeout
#define        RadioEnableSQI                Spirit1EnableSQI
#define        RadioSetRssiTH                 Spirit1SetRssiTH//
#define        RadioClearIRQ                  Spirit1ClearIRQ//
#define        RadioStartRx                     Spirit1StartRx
#define        RadioStartTx                     Spirit1StartTx
#define        RadioGetRxPacket                 Spirit1GetRxPacket

#define    RADIO_GPIO_MODE_DIGITAL_OUTPUT_LP      SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP
#define    RADIO_GPIO_DIG_OUT_IRQ                 SPIRIT_GPIO_DIG_OUT_IRQ

#define    RadioCsmaInit             CsmaInit

#define    RadioIrqs                 SpiritIrqs

#define    RadioGetReceivedDestinationAddress()       SpiritPktCommonGetReceivedDestAddress()

#define    Radio_PktStackAddressesInit                SpiritPktStackAddressesInit

#define    Radio_PktStackLlpInit                SpiritPktStackLlpInit
#define    RadioPktBasicAddressesInit                SpiritPktBasicAddressesInit

#define    RadioPktStackFilterOnSourceAddress                SpiritPktStackFilterOnSourceAddress
#define    RadioPktStackSetRxSourceMask                      SpiritPktStackSetRxSourceMask
#define    RadioPktStackSetSourceReferenceAddress            SpiritPktStackSetSourceReferenceAddress

#define    RadioPktStackInit                  SpiritPktStackInit
#define    RadioPktBasicInit                  SpiritPktBasicInit

#define    RadioCmdStrobeReady                SpiritCmdStrobeReady
#define    RadioRefreshStatus                 SpiritRefreshStatus
#define    RadioEnterShutdown()               SpiritEnterShutdown()
#define    RadioCmdStrobeStandby()            SpiritCmdStrobeStandby()
#define    RadioCmdStrobeSleep()              SpiritCmdStrobeSleep()
#define    RadioCmdStrobeSabort()             SpiritCmdStrobeSabort()
#define    RadioCmdStrobeRx()                 SpiritCmdStrobeRx()

#define    RadioGpioIrqGetStatus              SpiritIrqGetStatus

#define    RadioCsma                         SpiritCsma
#define    RadioRadioPersistenRx             SpiritRadioPersistenRx
#define    RadioRadioCsBlanking              SpiritRadioCsBlanking

#define    RadioQiSetRssiThresholddBm       SpiritQiSetRssiThresholddBm


#endif





#if defined(X_NUCLEO_S2868A1) || defined(X_NUCLEO_S2915A1)

#define        HAL_Radio_Init(void)          HAL_S2LP_Init(void)

//#define        RadioPacketConfig                S2LP_PacketConfig
#define        RadioSetPayloadlength            S2LP_SetPayloadlength
#define        RadioEnableTxIrq                 S2LP_EnableTxIrq
#define        RadioEnableRxIrq                 S2LP_EnableRxIrq
#define        RadioSetDestinationAddress       S2LP_SetDestinationAddress
#define        RadioSetRxTimeout                S2LP_SetRxTimeout
#define        RadioEnableSQI                   S2LP_EnableSQI
#define        RadioStartRx                     S2LP_StartRx
#define        RadioStartTx                     S2LP_StartTx
#define        RadioGetRxPacket                 S2LP_GetRxPacket


#define    RADIO_GPIO_MODE_DIGITAL_OUTPUT_LP      S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP
#define    RADIO_GPIO_DIG_OUT_IRQ                 S2LP_GPIO_DIG_OUT_IRQ

#define    RadioCsmaInit                 SCsmaInit

#define    RadioIrqs                 S2LPIrqs

#define    RadioGetReceivedDestinationAddress()    S2LPGetReceivedDestinationAddress()

#define    Radio_PktStackAddressesInit             S2LP_PktStackAddressesInit

#define    Radio_PktStackLlpInit                S2LP_PktStackLlpInit
#define    RadioPktBasicAddressesInit            S2LPPktBasicAddressesInit


#define    Radio_PktStackFilterOnSourceAddress                S2LP_PktStackFilterOnSourceAddress
#define    Radio_PktStackSetRxSourceMask                    S2LP_PktStackSetRxSourceMask
#define    Radio_PktStackSetSourceReferenceAddress          S2LP_PktStackSetSourceReferenceAddress

#define    RadioPktStackInit                S2LPPktStackInit
#define    RadioPktBasicInit                S2LPPktBasicInit

#define    RadioCmdStrobeReady                S2LPCmdStrobeReady
#define    RadioRefreshStatus                 S2LPRefreshStatus
//#define    RadioEnterShutdown()               S2LPEnterShutdown()
#define    RadioCmdStrobeStandby()            S2LPCmdStrobeStandby()
#define    RadioCmdStrobeSleep()              S2LPCmdStrobeSleep()
#define    RadioCmdStrobeSabort()              S2LPCmdStrobeSabort()
#define    RadioCmdStrobeRx()              S2LPCmdStrobeRx()

#define    RadioGpioIrqGetStatus              S2LPGpioIrqGetStatus


#define    RadioCsma                         S2LPCsma
#define    RadioRadioPersistenRx             S2LPPacketHandlerSetRxPersistentMode
#define    RadioRadioCsBlanking              S2LPRadioCsBlanking

#define    RadioQiSetRssiThresholddBm       S2LPRadioSetRssiThreshdBm


#endif


#endif //_P2P_LIB_H_
