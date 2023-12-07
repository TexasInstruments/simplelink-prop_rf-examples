/******************************************************************************

 @file rfClient.h

 @brief rfClient Header
 *
 *  Created on: Jun 23, 2022
 *
 *****************************************************************************/
#ifndef RFCLIENT_H_
#define RFCLIENT_H_

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <oad/native_oad/oad_storage.h>

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
#define OADClient_BLOCK_REQ_POLL_DELAY 80
#define OADClient_BLOCK_REQ_RATE 60
#define OADClient_MAX_RETRIES 3

/* node events */
#define CLIENT_EVENT_ALL                 0xFFFFFFFF
#define CLIENT_EVENT_NEW_OAD_MSG         (uint32_t)(1 << 0)
#define CLIENT_EVENT_STATUS_UPDATE       (uint32_t)(1 << 1)
#define CLIENT_EVENT_OAD_REQ             (uint32_t)(1 << 2)

/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
/**
 *  @brief          Initializes the OAD client and OAD protocol parameters.
 *
 *  @return         Nothing.
 */
void OAD_Init(void);

/**
 *  @brief          This function initializes the display.
 *
 *  @return         Nothing.
 */
void Display_setInitialization(void);

/**
 *  @brief          This function initializes the green and red LEDs.
 *
 *  @return         Nothing.
 */
void GPIO_setInitialization(void);

/**
 *  @brief          This function gets called in the radio module to
 *                  post an event for a new OAD message.
 *
 *  @return         Nothing.
 */
void rfClient_postNewOADMsg(void);

/**
 *  @brief                          This function keeps track of the block, total blocks and retries
 *
 *  @param      newOadBlock         The new counter value for the received OAD block.
 *  @param      oadBNumBlocks       Total amount of OAD blocks.
 *  @param      retries             The number of retries during the OAD progress.
 *  @return                         Nothing.
 */
void rfClient_displayOadBlockUpdate(uint16_t newOadBlock, uint16_t oadBNumBlocks, uint32_t retries);

/**
 *  @brief                          This function prints messages relating to the OAD process.
 *
 *  @param      status              Takes an OADStorage_Status_t value that indicate which state the OAD process is at.
 *  @return                         Nothing.
 */
void rfClient_displayOadStatusUpdate(OADStorage_Status_t status);

/**
 *  @brief                          This function saves a string in the storage and goes into a system reset.
 *
 *  @param      pDstAddr            Destination address to send
 *  @return                         Nothing.
 */
void rfClient_resetUserApp(void* pDstAddr);

/**
 *  @brief                          This function prints messages relating to the OAD process to UART.
 *             
 *  @return                         Nothing.
 */
void rfClient_printUpdate(void);

#endif /* RFCLIENT_H_ */
