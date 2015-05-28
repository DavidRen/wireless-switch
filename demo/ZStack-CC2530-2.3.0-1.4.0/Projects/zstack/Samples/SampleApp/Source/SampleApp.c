/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "Hal_flash.h"
/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "OSAL_Nv.h"
#include <string.h>
#include "zigbee_wrt_protocol.h"

#define TEST_NV 0x0202

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 Calc_CRC8(uint8 *PData, uint32 Len);
// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

//uint8 SampleApp_TransID;  // This is the unique message ID (counter)
uint8 globa_run_num = 0;

//afAddrType_t SampleApp_Periodic_DstAddr;
//afAddrType_t SampleApp_Flash_DstAddr;
//afAddrType_t SampleApp_P2P_DstAddr;

aps_Group_t SampleApp_Group;

//uint8 SampleAppPeriodicCounter = 0;
//uint8 SampleAppFlashCounter = 0;
uint8 uart_data_buffer[50];

//static  uint8     user_endpoint;//端口
//static  uint16    user_profile_id;//应用工程id 两个zigbee要一样才可以通讯
//static  uint16    user_panid;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void  AF_To_R_From_C(afAddrType_t *srcAddr,char *pdata, uint16 len);
int  pack_msg_ack_param(Msg_WRT_Zg_Set_Get_info *mzwt, char *pout);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void  AF_To_R_From_C(afAddrType_t *srcAddr,char *pdata, uint16 len)
{
	if ( AF_DataRequest( srcAddr, &SampleApp_epDesc,
						 3,						
						 len,
						 (uint8 *)pdata,
						 &globa_run_num++,
						 AF_DISCV_ROUTE,
						 AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
		HalLedBlink(HAL_LED_2, 2, 50, 1000);
	}
	else
	{
	  // Error occurred in request to send.
	}
}

#define      PANID_PROFILE_ENDPOINT_ADDR     0x0202

void   Set_Zgb_param(Msg_WRT_Zg_Set_Get_info *pinfo)
{
	uint8   tempbuf[6];
	
	tempbuf[0] = pinfo->panid;
	tempbuf[1] = pinfo->panid >> 8;

	tempbuf[2] = pinfo->profileid;
	tempbuf[3] = pinfo->profileid >> 8;

	tempbuf[4] = pinfo->endpoint;

	
	if ( osal_nv_item_init(PANID_PROFILE_ENDPOINT_ADDR, 5, NULL) == ZSUCCESS)
	{
		if(osal_nv_write(PANID_PROFILE_ENDPOINT_ADDR,0, 5, tempbuf) == ZSUCCESS)
		{
			//ack
		}	    
	}

}
// return wrt zgb param
void  Get_Zgb_param(void)
{
	Msg_WRT_Zg_Set_Get_info param;
	uint16  datalen;
	uint8   tempbuf[6];
    osal_memset(tempbuf,0,6);
	 
	if(osal_nv_read(PANID_PROFILE_ENDPOINT_ADDR,0, 2, &tempbuf) == ZSUCCESS)
	{
		param.panid = tempbuf[0] + (tempbuf[1]<<8);
		param.profileid = tempbuf[2] + (tempbuf[3]<<8);
		param.endpoint = tempbuf[4];	
		
		datalen = pack_msg_ack_param(&param,  uart_data_buffer);
		HalUARTWrite(0, uart_data_buffer, datalen);
	}			
}

static void SerialApp_CallBack(uint8 port, uint8 event)
{
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
   {
       // HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf+SerialApp_TxLen+1,
        //                                            SERIAL_APP_TX_MAX-SerialApp_TxLen);	
        
	  uint8 crc_t; 
      uint8 *data;
      Zg_Header_Struct *ph;
      Msg_Zg_WRT_transfer *mzwt;
//	  HalLedSet(HAL_LED_2, HAL_LED_MODE_TOGGLE);
      HalUARTRead(0, uart_data_buffer,sizeof(uart_data_buffer));
	  ph  = (Zg_Header_Struct *)uart_data_buffer;
	  crc_t = Calc_CRC8(uart_data_buffer + sizeof(Zg_Header_Struct), ph->msg_len);
	 
	 if(ph->crc8 == crc_t)
	 {
		switch(ph->msg_cmd)
		{
			case WRT_ZGB_DATA_CMD:
				mzwt = (Msg_Zg_WRT_transfer *)(uart_data_buffer + sizeof(Zg_Header_Struct));
                data = (uint8 *)(uart_data_buffer + sizeof(Zg_Header_Struct) +sizeof(Msg_Zg_WRT_transfer));			
				AF_To_R_From_C(&mzwt->end_dev_addr,(char*)data, ph->msg_len - sizeof(Msg_Zg_WRT_transfer));
				break;

			case WRT_ZGB_SET_INFO_CMD:
				Set_Zgb_param((Msg_WRT_Zg_Set_Get_info *)(uart_data_buffer + sizeof(Zg_Header_Struct)));
				break;
			case WRT_ZGB_GET_INFO_CMD:
				Get_Zgb_param();
				break;
			default:
				break;
		}
	}
   }

}

int pack_msg_ack_param(Msg_WRT_Zg_Set_Get_info *mzwt, char *pout)
{
	char *pt;
    pt = pout;
	Zg_Header_Struct hd;
	
	hd.head = 0x7e7e;
	hd.msg_serial_num = globa_run_num++;
	hd.msg_cmd = ZGB_WRT_ACK_INFO_CMD;
	hd.msg_len = sizeof(Msg_WRT_Zg_Set_Get_info) ;
	pt += sizeof(Zg_Header_Struct);
	
	osal_memcpy(pt, (char *)mzwt, sizeof(Msg_WRT_Zg_Set_Get_info));
	pt += sizeof(Msg_WRT_Zg_Set_Get_info);

	hd.crc8 = Calc_CRC8((uint8 *)pout+sizeof(hd), hd.msg_len);
	osal_memcpy(pout,(char *)&hd, sizeof(Zg_Header_Struct));

	return hd.msg_len + sizeof(Zg_Header_Struct);       	
}

int  pack_msg_transfer(Msg_Zg_WRT_transfer *mzwt,uint8 *pdata,uint16 dataLen, char *pout)
{
    char *pt;
    pt = pout;
	Zg_Header_Struct hd;
	
	hd.head = 0x7e7e;
	hd.msg_serial_num = globa_run_num++;
	hd.msg_cmd = ZGB_WRT_DATA_CMD;
	hd.msg_len = sizeof(Msg_Zg_WRT_transfer) + dataLen;// point size is 2 byte
	pt += sizeof(Zg_Header_Struct);
	
	osal_memcpy(pt,(char *)mzwt, sizeof(Msg_Zg_WRT_transfer));
	pt += sizeof(Msg_Zg_WRT_transfer);
	osal_memcpy(pt,pdata, dataLen);	
	pt += dataLen;

	hd.crc8 = Calc_CRC8((uint8 *)pout+sizeof(hd), hd.msg_len);
	osal_memcpy(pout,(char *)&hd, sizeof(Zg_Header_Struct));

	return hd.msg_len + sizeof(Zg_Header_Struct);                       
}

/*
init from flash
*/
int  init_from_flash(void)
{

	uint8   tempbuf[6];
    osal_memset(tempbuf,0,6);
	 
	if(osal_nv_read(PANID_PROFILE_ENDPOINT_ADDR,0, 2, &tempbuf) == ZSUCCESS)
	{
//		user_panid = tempbuf[0] + (tempbuf[1] << 8);

		SampleApp_SimpleDesc.AppProfId = tempbuf[2] + (tempbuf[3] << 8);

		SampleApp_epDesc.endPoint=tempbuf[4];
	
	}
}
/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
//  SampleApp_TransID = 0;
  
  //------------------------配置串口---------------------------------
  //MT_UartInit();                    //串口初始化
  //MT_UartRegisterTaskID(task_id);   //注册串口任务
   //HalUARTWrite(0,"UartInit OK\n", sizeof("UartInit OK\n"));//串口发送
  //-----------------------------------------------------------------
    
  
  halUARTCfg_t uartConfig ;
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = 64; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (0, &uartConfig);
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
//  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
//  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
//  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
//  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  //P2P initialize
//  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
//  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;  
//  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;//发给协调器



  // Fill out the endpoint description.
  uint8   tempbuf[6];
  osal_memset(tempbuf,0,6);
   
  if(osal_nv_read(PANID_PROFILE_ENDPOINT_ADDR,0, 6, &tempbuf) == ZSUCCESS)
  {
//	  user_panid = tempbuf[0] + (tempbuf[1] << 8);
	  SampleApp_SimpleDesc.AppProfId = tempbuf[2] + (tempbuf[3] << 8);
	  SampleApp_SimpleDesc.EndPoint=tempbuf[4];
	  SampleApp_epDesc.endPoint[4];
  }

  
//  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
//  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group);

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
  
//  osal_start_timerEx( SampleApp_TaskID, SYS_EVENT_MSG_USER_TM,5000);

   /*
  	flash block
  	short int value_read = 0; 
	char str_read[8] ;
	short int value = 0x0304;
	char *str = "abcdefg";
	int temp = strlen(str);
	osal_nv_item_init(TEST_NV,7,NULL);//NULL表示初始化的时候，item数据部分为空
	int len = osal_nv_item_len(TEST_NV);
	osal_nv_write(TEST_NV,0,1,&value);
	int len2 = osal_nv_item_len(TEST_NV);
	osal_nv_read(TEST_NV,0,2,&value_read);
    */

}



/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
char  buf[64];
int   buflen;
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              //|| (SampleApp_NwkState == DEV_ROUTER)
              //|| (SampleApp_NwkState == DEV_END_DEVICE) 
              )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
            osal_stop_timerEx(SampleApp_TaskID,SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    HalLedSet(HAL_LED_3,HAL_LED_MODE_TOGGLE);
   // SampleApp_SendPeriodicMessage();
	  //增加点播的发送函数 
    // Setup to send message again in normal period (+ a little jitter)


    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  
  //user specific event
  if ( events & SYS_EVENT_MSG_USER_TM )
  {  

	
	  
	  return (events ^ SYS_EVENT_MSG_USER_TM);
  }
 
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;  
  Msg_Zg_WRT_transfer rpst;
	  osal_memset(buf, 0, sizeof(buf));
	  rpst.src_type = 1;				//for test
	  rpst.len = pkt->cmd.DataLength;	//for test
	  rpst.end_dev_addr = pkt->srcAddr;	//for test
	  buflen = pack_msg_transfer(&rpst,pkt->cmd.Data,pkt->cmd.DataLength, buf);
	  HalUARTWrite(0, (uint8*)buf, buflen);//串口发送
	  HalLedSet(HAL_LED_3,HAL_LED_MODE_TOGGLE);
	  return;
  switch ( pkt->clusterId )
  {
  	HalLedSet(HAL_LED_4,HAL_LED_MODE_TOGGLE);
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
	case SAMPLEAPP_R2C_CLUSTERID://
	  
      break;
	case SAMPLEAPP_C2R_CLUSTERID://收到协调器发给路由的数据
      HalUARTWrite(0, pkt->cmd.Data,  pkt->cmd.DataLength);//串口发送
      break;
  }
}



void SampleApp_SendFlashMessage(uint16 flashTime)
{

}
/*********************************************************************
*********************************************************************/
