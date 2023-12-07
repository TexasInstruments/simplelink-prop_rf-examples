/******************************************************************************

 @file rfServer.h

 @brief rfServer Header
 *
 *  Created on: Jun 23, 2022
 *
 *****************************************************************************/
#ifndef RFSERVER_H_
#define RFSERVER_H_

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
/* Concentrator node events */
#define SERVER_EVENT_ALL                          0xFFFFFFFF
#define SERVER_EVENT_SELECT_ACTION                (uint32_t)(1 << 0)
#define SERVER_EVENT_UPDATE_DISPLAY               (uint32_t)(1 << 1)
#define SERVER_EVENT_OAD_MSG                      (uint32_t)(1 << 2)
#define SERVER_EVENT_UART_FW_UPDATE               (uint32_t)(1 << 3)

/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/
/* Brief UI actions */
typedef enum
{
    Server_Actions_UpdateAvailableFw = 0,
    Server_Actions_FwVerReq = 1,
    Server_Actions_UpdateClientFw = 2,
} Server_Actions_t;

/* Concentrator OAD status codes */
typedef enum {
    NodeOadStatus_NotInProgress = 0, ///< No OAD in progress
    NodeOadStatus_ResetComplete = 1, ///< OAD ready
    NodeOadStatus_InProgress = 2,    ///< OAD in progress
    NodeOadStatus_Completed = 3,     ///< OAD complete
    NodeOadStatus_Aborted = 4,       ///< OAD abort
} NodeOadStatus_t;

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
 *  @brief          This function initializes the GPIO pins for button 1 and button 2.
 *                  It also initializes the green and red LEDs.
 *
 *  @return         Nothing.
 */
void GPIO_setInitialization(void);

/**
 *  @brief          This is a callback function when a button gets pressed.
 *
 *  @param index    Button 1 or Button 2.
 *  @return         Nothing.
 */
void buttonCallbackFunction(uint_least8_t index);

/**
 *  @brief          This function prints messages from OAD events and button presses.
 *
 *  @return         Nothing.
 */
void ServerStatusUpdate(void);

/**
 *  @brief          This function initializes the display.
 *
 *  @return         Nothing.
 */
void Display_setInitialization(void);

/**
 *  @brief                  This function gets called from the OAD client to
 *                          update the current client FW.
 *  @param fwVersionStr     Takes pointer to a char buffer.
 *  @return                 Nothing.
 */
void rfServer_updateCurrentClientFWVer(char * fwVersionStr);

/**
 *  @brief                  This function will post an event for
 *                          any incoming OAD messages.
 *
 *  @return                 Nothing.
 */
void rfServer_postNewOadMsg(void);

/**
 *  @brief                  This function changes a local variable's status
 *                          to indicate which state the OAD progress is in.
 *
 *  @return                 Nothing.
 */
void rfServer_updateNodeOadStatus(NodeOadStatus_t status);

/**
 *  @brief                  This is a callback function that gets called after
 *                          receiving a reset response from the client. It is
 *                          needed to change a state of a variable so it can
 *                          proceed with updating the client firmware.
 *
 *  @return                 Nothing.
 */
void rfServer_setNodeOadStatusReset(void);

/**
 *  @brief                  This function updates the current block counter so
 *                          it can be displayed on the UART terminal.
 *
 *  @return                 Nothing.
 */
void rfServer_updateNodeOadBlock(uint16_t block);

/**
 *  @brief                  This function is used for the UART firmware download.
 *
 *  @return                 Nothing.
 */
void rfServer_updateAvailableFWVer(uint8_t status);

#endif /* RFOADCLIENTAPP_H_ */
