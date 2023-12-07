/******************************************************************************

 @file radio.h

 @brief radio Header
 *
 *  Created on: Jun 21, 2022
 *
 *****************************************************************************/
#ifndef RADIO_RADIO_H_
#define RADIO_RADIO_H_

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "ti_drivers_config.h"
#include <ti_radio_config.h>

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             254 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

typedef void (*App_postEvent_t)(void);
/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
/**
 *  @brief                      Initializes all default settings to radio params, PropTx and PropRx.
 *  @param   funcPtr            A function pointer
 *  @return                     Pointer to a buffer.
 */
uint8_t * Radio_Init(App_postEvent_t funcPtr);

/**
 *  @brief                      This is a modular function that transmits a single packet.
 *  @param   txpacket           Takes in a buffer to transmit
 *  @param   payload_length     Length of the buffer
 *  @return                     Pointer to a buffer.
 */
void Radio_txPacket(uint8_t * txpacket, uint16_t payload_length);

/**
 *  @brief                      This is a modular function that receives a single packet.
 *
 *  @return                     Nothing.
 */
void Radio_rxPacket(void);

/**
 *  @brief                      This is a modular function that receives a single packet.
 *  @param   h                  A handle that is returned by to RF_open().
 *  @param   ch                 Command handle that is returned by RF_postCmd().
 *  @param   e                  Possible event flags are listed in @ref RF_Core_Events and @ref RF_Driver_Events.
 *  @return                     Nothing.
 */
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

#endif /* RADIO_RADIO_H_ */
