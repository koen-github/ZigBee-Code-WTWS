/**************************************************************************************************
  Filename:       DemoCoordinator.c

  Description:    Coordinator application for the sensor demo utilizing Simple API.

                  The coordinator node functions as a gateway. The node accepts
                  incoming reports from the sensor nodes (router and end device)
                  and can send the reports via the UART to a PC tool.


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_uart.h"
#include "DemoApp.h"

/******************************************************************************
 * CONSTANTS
 */
// General UART frame offsets
#define FRAME_SOF_OFFSET                    0
#define FRAME_LENGTH_OFFSET                 1
#define FRAME_CMD0_OFFSET                   2
#define FRAME_CMD1_OFFSET                   3
#define FRAME_DATA_OFFSET                   4

// ZB_RECEIVE_DATA_INDICATION offsets
#define ZB_RECV_SRC_OFFSET                  0
#define ZB_RECV_CMD_OFFSET                  2
#define ZB_RECV_LEN_OFFSET                  4
#define ZB_RECV_DATA_OFFSET                 6
#define ZB_RECV_FCS_OFFSET                  8

// ZB_RECEIVE_DATA_INDICATION frame length
#define ZB_RECV_LENGTH                      15

// PING response frame length and offset
#define SYS_PING_RSP_LENGTH                 7
#define SYS_PING_CMD_OFFSET                 1

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007
#else
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define CPT_SOP                             0xFE
#define SYS_PING_REQUEST                    0x0021
#define SYS_PING_RESPONSE                   0x0161
#define ZB_RECEIVE_DATA_INDICATION          0x8746

// Application osal event identifiers
#define MY_START_EVT                        0x0001
#define MY_FIND_COLLECTOR_EVT               0x0002
#define MY_DOOR_STATUS_EVT                  0x0004


/******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16              source;
  uint16              parent;
  uint8               temp;
  uint8               voltage;
} gtwData_t;

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 appState =             APP_INIT;
static uint8 myStartRetryDelay =    10;          // milliseconds

static bool lampIsActivated = false;
static uint8 pollingDoorStatusInterval =      1000;
bool isDoorCurrentlyOpen = false; 

//define toggl lamp
static void setLampValue(bool value);
//define lamp status messag send
static void sendLampStatusMessage(void);
static void sendCurrentDoorStatus(void);

static void openDoor(bool openDoor);


/******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 calcFCS(uint8 *pBuf, uint8 len);
static void sysPingReqRcvd(void);
static void sysPingRsp(void);


/******************************************************************************
 * GLOBAL VARIABLES
 */
// Inputs and Outputs for Collector device
#define NUM_OUT_CMD_COLLECTOR           2
#define NUM_IN_CMD_COLLECTOR            2

// List of output and input commands for Collector device
const cId_t zb_OutCmdList[NUM_OUT_CMD_COLLECTOR] =
{
  LED_REPORT_CMD_ID,
  DOOR_STATUS_CMD_ID
};
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  LDR_REPORT_CMD_ID,
  KEYLOCK_CMD_ID
};


// Define SimpleDescriptor for Collector device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList              //  Output Command List
};

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{
  if( event & SYS_EVENT_MSG )
  {
  }

  if( event & ZB_ENTRY_EVENT )
  {
    // Initialise UART
    initUart(uartRxCB);

    // blind LED 1 to indicate starting/joining a network
    HalLedBlink ( HAL_LED_1, 0, 50, 500 );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );

    // Start the device
    appState = APP_START;
    zb_StartRequest();
    
    MCU_IO_DIR_OUTPUT(0,5); //set Lamp value.
    MCU_IO_DIR_OUTPUT(0,6); //set Door value
    MCU_IO_SET_HIGH(0,5); //set lamp on default
    
    MCU_IO_INPUT(0,4,MCU_IO_PULLUP); 
    isDoorCurrentlyOpen = MCU_IO_GET(0,4);
      
    osal_start_timerEx(osal_self(),MY_DOOR_STATUS_EVT,pollingDoorStatusInterval);
    
    
    lampIsActivated = true;
  }
  
  if(event & MY_DOOR_STATUS_EVT){
    osal_start_reload_timer(osal_self(),MY_DOOR_STATUS_EVT,pollingDoorStatusInterval);
    if(MCU_IO_GET(0,4) != isDoorCurrentlyOpen)
    {
     if(MCU_IO_GET(0,4)){
        isDoorCurrentlyOpen = MCU_IO_GET(0,4);
      }
      else{
        isDoorCurrentlyOpen = MCU_IO_GET(0,4);
      }
    
      sendCurrentDoorStatus();
    }
  }

  if ( event & MY_START_EVT )
  {
    zb_StartRequest();
  }
  if ( event & MY_FIND_COLLECTOR_EVT )
  {
    // blink LED 2 to indicate discovery and binding
    HalLedBlink ( HAL_LED_2, 0, 50, 200 );

    // Find and bind to a collector device
    zb_BindDevice( TRUE, DOOR_STATUS_CMD_ID, (uint8 *)NULL );
  }
}

/******************************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  static uint8 allowBind = FALSE;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      if ( appState == APP_RUN )
      {
        allowBind ^= 1;
        if ( allowBind )
        {
          // Turn ON Allow Bind mode infinitly
          zb_AllowBind( 0xFF );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
        }
        else
        {
          // Turn OFF Allow Bind mode infinitly
          zb_AllowBind( 0x00 );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
        }
      }
    }
    if ( keys & HAL_KEY_SW_2 )
    {
      //bind second time with sensor.
      zb_BindDevice(true,DOOR_STATUS_CMD_ID,(uint8 *)NULL);
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
}

static void sendLampStatusMessage(void){
   //send back to let the user know the lamp is on.
   uint8 pData[1];
   pData[0]= (uint8)lampIsActivated;
   zb_SendDataRequest( ZB_BINDING_ADDR, LED_REPORT_CMD_ID, 1, pData, 0, AF_MSG_ACK_REQUEST, 0 );

}

static void sendCurrentDoorStatus(void){
  uint8 pData[1];
  pData[0]= (bool)isDoorCurrentlyOpen;
  zb_SendDataRequest( ZB_BINDING_ADDR, DOOR_STATUS_CMD_ID, 1, pData, 0, AF_MSG_ACK_REQUEST, 0 );
}

static void setLampValue(bool value){
  if(value){
    MCU_IO_SET_HIGH(0,5); //set lamp HIGH
    lampIsActivated = true;
  }
  else{
    MCU_IO_SET_LOW(0,5); //set lamp LOW
    lampIsActivated = false;
  }
   sendLampStatusMessage();
}

static void openDoor(bool openDoor){
  if(openDoor){
    MCU_IO_SET_HIGH(0,6);
  }
  else{
    MCU_IO_SET_LOW(0,6);
  }
}



/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )
  {
    // Set LED 1 to indicate that node is operational on the network
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );

    // Change application state
    appState = APP_RUN;
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee stack after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  (void)handle;
  (void)status;
}

/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )
  {
    if(commandId == LED_REPORT_CMD_ID){
      HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      sendLampStatusMessage(); //send current lamp status.
      appState = APP_RUN; //finsihed binding.
    }
    else if(commandId == DOOR_STATUS_CMD_ID){
      HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      sendCurrentDoorStatus(); //send current door status.
      appState = APP_RUN; //finsihed binding.
    }
  }
}

/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
   (void)source;
    
    static bool lampisBinded=false;

  if(!lampisBinded){ //first bootup the router
    //connect door
    zb_BindDevice(true,DOOR_STATUS_CMD_ID,(uint8 *)NULL);
    lampisBinded=true;
  }
  else{
    //connect lamp
    zb_BindDevice(true,LED_REPORT_CMD_ID,(uint8 *)NULL);
  }
    

}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
  (void)searchType;
  (void)searchKey;
  (void)result;
}

/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
  if(command == LDR_REPORT_CMD_ID){
     setLampValue(pData[0]);
  }
  else if(command == KEYLOCK_CMD_ID){
     openDoor(pData[0]);
  }
}

/******************************************************************************
 * @fn          uartRxCB
 *
 * @brief       Callback function for UART
 *
 * @param       port - UART port
 *              event - UART event that caused callback
 *
 * @return      none
 */
void uartRxCB( uint8 port, uint8 event )
{
  (void)port;

  uint8 pBuf[RX_BUF_LEN];
  uint16 cmd;
  uint16 len;

  if ( event != HAL_UART_TX_EMPTY )
  {
    // Read from UART
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );

    if ( len > 0 )
    {
      cmd = BUILD_UINT16(pBuf[SYS_PING_CMD_OFFSET + 1], pBuf[SYS_PING_CMD_OFFSET]);

      if( (pBuf[FRAME_SOF_OFFSET] == CPT_SOP) && (cmd == SYS_PING_REQUEST) )
      {
        sysPingReqRcvd();
      }
    }
  }
}

/******************************************************************************
 * @fn          sysPingReqRcvd
 *
 * @brief       Ping request received
 *
 * @param       none
 *
 * @return      none
 */
static void sysPingReqRcvd(void)
{
   sysPingRsp();
}

/******************************************************************************
 * @fn          sysPingRsp
 *
 * @brief       Build and send Ping response
 *
 * @param       none
 *
 * @return      none
 */
static void sysPingRsp(void)
{
  uint8 pBuf[SYS_PING_RSP_LENGTH];

  // Start of Frame Delimiter
  pBuf[FRAME_SOF_OFFSET] = CPT_SOP;

  // Length
  pBuf[FRAME_LENGTH_OFFSET] = 2;

  // Command type
  pBuf[FRAME_CMD0_OFFSET] = LO_UINT16(SYS_PING_RESPONSE);
  pBuf[FRAME_CMD1_OFFSET] = HI_UINT16(SYS_PING_RESPONSE);

  // Stack profile
  pBuf[FRAME_DATA_OFFSET] = LO_UINT16(STACK_PROFILE);
  pBuf[FRAME_DATA_OFFSET + 1] = HI_UINT16(STACK_PROFILE);

  // Frame Check Sequence
  pBuf[SYS_PING_RSP_LENGTH - 1] = calcFCS(&pBuf[FRAME_LENGTH_OFFSET], (SYS_PING_RSP_LENGTH - 2));

  // Write frame to UART
  HalUARTWrite(HAL_UART_PORT_0,pBuf, SYS_PING_RSP_LENGTH);
}

/******************************************************************************
 * @fn          calcFCS
 *
 * @brief       This function calculates the FCS checksum for the serial message
 *
 * @param       pBuf - Pointer to the end of a buffer to calculate the FCS.
 *              len - Length of the pBuf.
 *
 * @return      The calculated FCS.
 ******************************************************************************
 */
static uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while ( len-- )
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}
