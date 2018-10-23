/******************************************************************************

 @file       simple_beacon.c

 @brief This file contains the Simple Eddystone Beacon sample application for use
        with the CC2640R2F Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_02_30_00_28
 Release Date: 2018-10-15 15:51:38
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>
#include <driverlib/aon_batmon.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <icall.h>
#include "util.h"
#include "att_rsp.h"

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "ll_common.h"

#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board_key.h"

#include "Board.h"
#include "eddystoneURLCfg.h"
#include "simple_beacon.h"
#include "UTC_clock.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr;  // event header.
    uint8_t *pData;  // event data
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static eddystoneAdvData_t eddystoneAdv =
{
    // Flags; this sets the device to use general discoverable mode
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // Complete list of 16-bit Service UUIDs
    0x03,   // length of this data including the data type byte
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_UINT16(EDDYSTONE_SERVICE_UUID),
    HI_UINT16(EDDYSTONE_SERVICE_UUID),

    // Service Data
    0x03, // to be set properly later
    GAP_ADTYPE_SERVICE_DATA,
    LO_UINT16(EDDYSTONE_SERVICE_UUID),
    HI_UINT16(EDDYSTONE_SERVICE_UUID)
};

eddystoneUID_t   eddystoneUID;
eddystoneURL_t   eddystoneURL;
eddystoneTLM_t   eddystoneTLM;

static eddystoneCfgAdvData_t eddystoneCfgAdv =
{
    // Flags; this sets the device to use general discoverable mode
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // Complete list of 128-bit Service UUIDs
    0x11,   // length of this data
    GAP_ADTYPE_128BIT_COMPLETE,
    {EDDYSTONE_BASE_UUID_128(URLCFGSVC_SVC_UUID)},

    // Power Level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    4  // To be set properly later to be changed
};

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x16,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'S',
    'i',
    'm',
    'p',
    'l',
    'e',
    'E',
    'd',
    'd',
    'y',
    's',
    't',
    'o',
    'n',
    'e',
    'B',
    'e',
    'a',
    'c',
    'o',
    'n',

    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
};

// Array of URL Scheme Prefices
static char* eddystoneURLPrefix[EDDYSTONE_URL_PREFIX_MAX] =
{
    "http://www.",
    "https://www.",
    "http://",
    "https://"
};

// Array of URLs to be encoded
static char* eddystoneURLEncoding[EDDYSTONE_URL_ENCODING_MAX] =
{
    ".com/",
    ".org/",
    ".edu/",
    ".net/",
    ".info/",
    ".biz/",
    ".gov/",
    ".com/",
    ".org/",
    ".edu/",
    ".net/",
    ".info/",
    ".biz/",
    ".gov/"
};

static uint32 advCount = 0;

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Beacon";

// Globals used for ATT Response retransmission
//static gattMsgEvent_t *pAttRsp = NULL;
//static uint8_t rspTxRetry = 0;

// Eddystone frame type currently used
static uint8 currentFrameType = EDDYSTONE_FRAME_TYPE_URL;

// URL Configuration mode
static uint8 URLCfgMode = FALSE;

// Connection status
static uint8 ConnectedInCfgMode = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleEddystoneBeacon_init( void );
static void SimpleEddystoneBeacon_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleEddystoneBeacon_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleEddystoneBeacon_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleEddystoneBeacon_processAppMsg(sbpEvt_t *pMsg);
static void SimpleEddystoneBeacon_processStateChangeEvt(gaprole_States_t newState);
static void SimpleEddystoneBeacon_processCharValueChangeEvt(uint8_t paramID);
static void SimpleEddystoneBeacon_performPeriodicTask(void);
static void SimpleEddystoneBeacon_clockHandler(UArg arg);

static void SimpleEddystoneBeacon_processAdvCompleteEvt(void);

static void SimpleEddystoneBeacon_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleEddystoneBeacon_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SimpleEddystoneBeacon_processPairState(uint8_t state, uint8_t status);
static void SimpleEddystoneBeacon_processPasscode(uint8_t uiOutputs);

static void SimpleEddystoneBeacon_stateChangeCB(gaprole_States_t newState);
static void SimpleEddystoneBeacon_charValueChangeCB(uint8_t paramID);

static void SimpleEddystoneBeacon_updateTLM(void);
static void SimpleEddystoneBeacon_initUID(void);
static void SimpleEddystoneBeacon_initConfiguration(void);
static void SimpleEddystoneBeacon_applyConfiguration(void);
static void SimpleEddystoneBeacon_selectFrame(uint8 frameType);
static void SimpleEddystoneBeacon_startRegularAdv(void);

static uint8_t SimpleEddystoneBeacon_enqueueMsg(uint8_t event, uint8_t state,
                                              uint8_t *pData);

static void SimpleEddystoneBeacon_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimpleEddystoneBeacon_processConnEvt(Gap_ConnEventRpt_t *pReport);



/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t SimpleEddystoneBeacon_gapRoleCBs =
{
    SimpleEddystoneBeacon_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t SimpleEddystoneBeacon_BondMgrCBs =
{
    (pfnPasscodeCB_t) SimpleEddystoneBeacon_passcodeCB, // Passcode callback
    SimpleEddystoneBeacon_pairStateCB                   // Pairing / Bonding state Callback
};

// Eddystone URL Configuration Service Callbacks
static urlCfgSvcCBs_t SimpleEddystoneBeacon_urlCfgCBs =
{
    SimpleEddystoneBeacon_charValueChangeCB // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
    NOT_REGISTER       = 0,
    FOR_AOA_SCAN       = 1,
    FOR_ATT_RSP        = 2,
    FOR_AOA_SEND       = 4,
    FOR_TOF_SEND       = 8
}connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleEddystoneBeacon_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    // in case  there is no registration for the connection event, make the registration
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        status = GAP_RegisterConnEventCb(SimpleEddystoneBeacon_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
    }

    if(status == SUCCESS)
    {
        //add the reason bit to the bitamap.
        CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
    }

    return(status);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimpleEddystoneBeacon_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

    CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
    // in case  there is no more registration for the connection event than unregister
    if (!CONNECTION_EVENT_IS_REGISTERED)
    {
        GAP_RegisterConnEventCb(SimpleEddystoneBeacon_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
    }

    return(status);
}

 /*********************************************************************
 * @fn      SimpleEddystoneBeacon_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleEddystoneBeacon_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = sbpTaskStack;
    taskParams.stackSize = SBP_TASK_STACK_SIZE;
    taskParams.priority = SBP_TASK_PRIORITY;

    Task_construct(&sbpTask, SimpleEddystoneBeacon_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    Board_shutDownExtFlash();

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&periodicClock, SimpleEddystoneBeacon_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

    dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

    // Set GAP Parameters: After a connection was established, delay in seconds
    // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
    // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
    // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
    // For current defaults, this has no effect.
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

    // Setup the Peripheral GAPRole Profile. For more information see the User's
    // Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html/
    {
        // Device starts advertising upon initialization of GAP
        uint8_t initialAdvertEnable = FALSE;
        uint8_t initialNonConnAdvEnable = FALSE;

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until re-enabled by the application
        uint16_t advertOffTime = 0;

        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Initialize UID frame
        SimpleEddystoneBeacon_initUID();

        // Start Clock for TLM
        UTC_init();

        // Set the Peripheral GAPRole Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &initialAdvertEnable);
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                     &initialNonConnAdvEnable);

        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                             &advertOffTime);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);

        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                             &enableUpdateRequest);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMinInterval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMaxInterval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                             &desiredSlaveLatency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                             &desiredConnTimeout);
    }

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Register an event for adv completion
    HCI_EXT_AdvEventNoticeCmd(selfEntity, SBP_ADV_COMPLETE_EVT);

    // Setup the GAP Bond Manager. For more information see the section in the
    // User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html/
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t mitm = TRUE;
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8_t bonding = TRUE;
        uint8_t replaceBonds = FALSE;

        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                                &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
        GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(uint8_t), &replaceBonds);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
    DevInfo_AddService();                        // Device Information Service
    VOID URLCfgSvc_AddService();                 // URL Configuration Service

    // Setup the URL Configuration Characteristic Values
    SimpleEddystoneBeacon_initConfiguration();

    // Register callback with SimpleGATTprofile
    URLCfgSvc_RegisterAppCBs(&SimpleEddystoneBeacon_urlCfgCBs);

    // Start the Device
    VOID GAPRole_StartDevice(&SimpleEddystoneBeacon_gapRoleCBs);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&SimpleEddystoneBeacon_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html
    GAP_RegisterForMsgs(selfEntity);
    
    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    //Set default values for Data Length Extension
    {
        //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
        #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        //This API is documented in hci.h
        //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
        //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
        //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
    // Get the currently set local supported LE features
    // The HCI will generate an HCI event that will get received in the main
    // loop
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

    Display_print0(dispHandle, 0, 0, "Simple Beacon");
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_initUID
 *
 * @brief   initialize UID frame
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_initUID(void)
{
    // Set Eddystone UID frame with meaningless numbers for example.
    // This need to be replaced with some algorithm-based formula
    // for production.
    eddystoneUID.namespaceID[0] = 0x00;
    eddystoneUID.namespaceID[1] = 0x01;
    eddystoneUID.namespaceID[2] = 0x02;
    eddystoneUID.namespaceID[3] = 0x03;
    eddystoneUID.namespaceID[4] = 0x04;
    eddystoneUID.namespaceID[5] = 0x05;
    eddystoneUID.namespaceID[6] = 0x06;
    eddystoneUID.namespaceID[7] = 0x07;
    eddystoneUID.namespaceID[8] = 0x08;
    eddystoneUID.namespaceID[9] = 0x09;

    eddystoneUID.instanceID[0] = 0x04;
    eddystoneUID.instanceID[1] = 0x51;
    eddystoneUID.instanceID[2] = 0x40;
    eddystoneUID.instanceID[3] = 0x00;
    eddystoneUID.instanceID[4] = 0xB0;
    eddystoneUID.instanceID[5] = 0x00;
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_encodeURL
 *
 * @brief   Encodes URL in accordance with Eddystone URL frame spec
 *
 * @param   urlOrg - Plain-string URL to be encoded
 *          urlEnc - Encoded URL. Should be URLCFGSVC_CHAR_URI_DATA_LEN-long.
 *
 * @return  0 if the prefix is invalid
 *          The length of the encoded URL including prefix otherwise
 */
uint8 SimpleEddystoneBeacon_encodeURL(char* urlOrg, uint8* urlEnc)
{
    uint8 i, j;
    uint8 urlLen;
    uint8 tokenLen;

    urlLen = (uint8) strlen(urlOrg);

    // search for a matching prefix
    for (i = 0; i < EDDYSTONE_URL_PREFIX_MAX; i++)
    {
        tokenLen = strlen(eddystoneURLPrefix[i]);
        if (strncmp(eddystoneURLPrefix[i], urlOrg, tokenLen) == 0)
        {
            break;
        }
    }

    if (i == EDDYSTONE_URL_PREFIX_MAX)
    {
        return 0;       // wrong prefix
    }

    // use the matching prefix number
    urlEnc[0] = i;
    urlOrg += tokenLen;
    urlLen -= tokenLen;

    // search for a token to be encoded
    for (i = 0; i < urlLen; i++)
    {
        for (j = 0; j < EDDYSTONE_URL_ENCODING_MAX; j++)
        {
            tokenLen = strlen(eddystoneURLEncoding[j]);
            if (strncmp(eddystoneURLEncoding[j], urlOrg + i, tokenLen) == 0)
            {
                // matching part found
                break;
            }
        }

        if (j < EDDYSTONE_URL_ENCODING_MAX)
        {
            memcpy(&urlEnc[1], urlOrg, i);
            // use the encoded byte
            urlEnc[i + 1] = j;
            break;
        }
    }

    if (i < urlLen)
    {
        memcpy(&urlEnc[i + 2],
               urlOrg + i + tokenLen, urlLen - i - tokenLen);
        return urlLen - tokenLen + 2;
    }

    memcpy(&urlEnc[1], urlOrg, urlLen);
    return urlLen + 1;
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_updateTLM
 *
 * @brief   Update TLM elements
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_updateTLM(void)
{
    uint32 time100MiliSec;
    uint32 batt;

    // Battery voltage (bit 10:8 - integer, but 7:0 fraction)
    batt = AONBatMonBatteryVoltageGet();
    batt = (batt * 125) >> 5; // convert V to mV
    eddystoneTLM.vBatt[0] = HI_UINT16(batt);
    eddystoneTLM.vBatt[1] = LO_UINT16(batt);
    // Temperature - 19.5 (Celcius) for example
    eddystoneTLM.temp[0] = 19;
    eddystoneTLM.temp[1] = 256 / 2;
    // advertise packet cnt;
    eddystoneTLM.advCnt[0] = BREAK_UINT32(advCount, 3);
    eddystoneTLM.advCnt[1] = BREAK_UINT32(advCount, 2);
    eddystoneTLM.advCnt[2] = BREAK_UINT32(advCount, 1);
    eddystoneTLM.advCnt[3] = BREAK_UINT32(advCount, 0);
    // running time
    time100MiliSec = UTC_getClock() * 10; // 1-second resolution for now
    eddystoneTLM.secCnt[0] = BREAK_UINT32(time100MiliSec, 3);
    eddystoneTLM.secCnt[1] = BREAK_UINT32(time100MiliSec, 2);
    eddystoneTLM.secCnt[2] = BREAK_UINT32(time100MiliSec, 1);
    eddystoneTLM.secCnt[3] = BREAK_UINT32(time100MiliSec, 0);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_initConfiguration
 *
 * @brief   set all URL Configuration characteristics to default values
 *
 * @param   none
 *
 * @return  none
 */
void SimpleEddystoneBeacon_initConfiguration(void)
{
    uint8 tempURLEnc[URLCFGSVC_CHAR_URI_DATA_LEN];
    uint8 temp8;
    uint16 temp16;
    uint8 tempLock[16] = URLCFG_CHAR_LOCK_DEFAULT;

    // set URI Data
    temp8 = SimpleEddystoneBeacon_encodeURL(URLCFG_CHAR_URI_DATA_DEFAULT,
                                          tempURLEnc);
    URLCfgSvc_SetParameter(URLCFGSVC_URI_DATA, temp8, tempURLEnc);

    // set Flags
    temp8 = URLCFG_CHAR_FLAGS_DEFAULT;
    URLCfgSvc_SetParameter(URLCFGSVC_FLAGS, 1, &temp8);

    // set TX Power Mode
    temp8 = URLCFG_CHAR_TX_POWER_MODE_DEFAULT;
    URLCfgSvc_SetParameter(URLCFGSVC_TX_POWER_MODE, 1, &temp8);

    // set Beacon Period
    temp16 = URLCFG_CHAR_BEACON_PERIOD_DEFAULT;
    URLCfgSvc_SetParameter(URLCFGSVC_BEACON_PERIOD, 2, &temp16);

    // set Lock Code
    URLCfgSvc_SetParameter(URLCFGSVC_LOCK, 16, tempLock);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_applyConfiguration
 *
 * @brief   Apply the changes maded in URL Configuration mode
 *
 * @param   none
 *
 * @return  none
 */
void SimpleEddystoneBeacon_applyConfiguration(void)
{
    int8 tempPwrLvls[4];
    uint8 tempPowerMode;
    int8 tempPower;
    uint16 tempPeriod;

    // update URL frame
    URLCfgSvc_GetParameter(URLCFGSVC_URI_DATA, eddystoneURL.encodedURL);

    // update TX power
    URLCfgSvc_GetParameter(URLCFGSVC_ADV_TX_PWR_LVLS, tempPwrLvls);
    URLCfgSvc_GetParameter(URLCFGSVC_TX_POWER_MODE, &tempPowerMode);
    tempPower = tempPwrLvls[tempPowerMode];
    HCI_EXT_SetTxPowerCmd(tempPower);
    eddystoneUID.rangingData = tempPower;
    eddystoneURL.txPower = tempPower;

    // update adv period
    URLCfgSvc_GetParameter(URLCFGSVC_BEACON_PERIOD, &tempPeriod);

    if (tempPeriod != 0)
    {
        // convert into multiple of 0.625us
        tempPeriod = (uint16) (tempPeriod * 8L / 5);

        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, tempPeriod);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, tempPeriod);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, tempPeriod);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, tempPeriod);
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_selectFrame
 *
 * @brief   Selecting the type of frame to be put in the service data
 *
 * @param   frameType - Eddystone frame type
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_selectFrame(uint8 frameType)
{
    if (frameType == EDDYSTONE_FRAME_TYPE_UID ||
        frameType == EDDYSTONE_FRAME_TYPE_URL ||
        frameType == EDDYSTONE_FRAME_TYPE_TLM)
    {

    eddystoneFrame_t*   pFrame;
    uint8               frameSize;
    uint8               temp;

    eddystoneAdv.length = EDDYSTONE_SVC_DATA_OVERHEAD_LEN;
    // Fill with 0s first
    memset((uint8*) &eddystoneAdv.frame, 0x00, sizeof(eddystoneFrame_t));

    switch (frameType)
    {
        case EDDYSTONE_FRAME_TYPE_UID:
            eddystoneUID.frameType = EDDYSTONE_FRAME_TYPE_UID;
            frameSize = sizeof(eddystoneUID_t);
            pFrame = (eddystoneFrame_t *) &eddystoneUID;
        break;

        case EDDYSTONE_FRAME_TYPE_URL:
            eddystoneURL.frameType = EDDYSTONE_FRAME_TYPE_URL;
            URLCfgSvc_GetParameter(URLCFGSVC_URI_DATA_LEN, &temp);
            frameSize = sizeof(eddystoneURL_t) - EDDYSTONE_MAX_URL_LEN + temp;
            pFrame = (eddystoneFrame_t *) &eddystoneURL;
        break;

        case EDDYSTONE_FRAME_TYPE_TLM:
            eddystoneTLM.frameType = EDDYSTONE_FRAME_TYPE_TLM;
            frameSize = sizeof(eddystoneTLM_t);
            SimpleEddystoneBeacon_updateTLM();
            pFrame = (eddystoneFrame_t *) &eddystoneTLM;
        break;

        default:
        //do nothing
        break;
    }

    memcpy((uint8 *) &eddystoneAdv.frame, (uint8 *) pFrame, frameSize);
    eddystoneAdv.length += frameSize;

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA,
                         EDDYSTONE_FRAME_OVERHEAD_LEN + eddystoneAdv.length,
                         &eddystoneAdv);
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_startRegularAdv
 *
 * @brief   Start regular advertise.
 *          If configuration mode was on going, stop it.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_startRegularAdv(void)
{
    uint8 advertEnabled = FALSE;
    uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;
    uint8 tempPeriod;

    SimpleEddystoneBeacon_applyConfiguration();

    // Stop connectable advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &advertEnabled);

    // if BeaconPeriod is 0, don't advertise.
    URLCfgSvc_GetParameter(URLCFGSVC_BEACON_PERIOD, &tempPeriod);

    if (tempPeriod != 0)
    {
        advertEnabled = TRUE;
    }

    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);

    // Select UID or URL frame as adv data initially
    SimpleEddystoneBeacon_selectFrame(currentFrameType);

    GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                         &advertEnabled);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_startConfigAdv
 *
 * @brief   Start advertising in configuration mode
 *          If regular advertising was on going, stop it.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_startConfigAdv(void)
{
    uint8 advertEnabled;
    uint8 advType;
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
    uint8 pwrLvls[4];

    advertEnabled = FALSE;
    GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                         &advertEnabled);

    advType = GAP_ADTYPE_ADV_IND;
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &advType);

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    // update TX power
    URLCfgSvc_GetParameter(URLCFGSVC_ADV_TX_PWR_LVLS, pwrLvls);
    HCI_EXT_SetTxPowerCmd(pwrLvls[TX_POWER_MODE_MEDIUM]);

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(eddystoneCfgAdv),
                         &eddystoneCfgAdv);

    advertEnabled = TRUE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &advertEnabled);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    SimpleEddystoneBeacon_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                    if (pEvt->signature == 0xffff)
                    {
                        if (pEvt->event_flag & SBP_ADV_COMPLETE_EVT)
                        {
                            SimpleEddystoneBeacon_processAdvCompleteEvt();
                        }
                    }
                    else
                    {
                        // Process inter-task message
                        safeToDealloc = SimpleEddystoneBeacon_processStackMsg((ICall_Hdr *)pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & SBP_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message.
                        SimpleEddystoneBeacon_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }

            if (events & SBP_PERIODIC_EVT)
            {
                // Perform periodic application task
                SimpleEddystoneBeacon_performPeriodicTask();
            }
        }
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleEddystoneBeacon_processStackMsg(ICall_Hdr *pMsg)
{
    uint8_t safeToDealloc = TRUE;

    switch (pMsg->event)
    {
        case GATT_MSG_EVENT:
            // Process GATT message
            safeToDealloc = SimpleEddystoneBeacon_processGATTMsg((gattMsgEvent_t *)pMsg);
        break;

        case HCI_GAP_EVENT_EVENT:
        {

            // Process HCI message
            switch(pMsg->status)
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                // Process HCI Command Complete Event
                {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
                    // This code will disable the use of the LL_CONNECTION_PARAM_REQ
                    // control procedure (for connection parameter updates, the
                    // L2CAP Connection Parameter Update procedure will be used
                    // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
                    // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE
                    // The L2CAP Connection Parameter Update procedure is used to
                    // support a delta between the minimum and maximum connection
                    // intervals required by some iOS devices.

                    // Parse Command Complete Event for opcode and status
                    hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
                    uint8_t   pktStatus = command_complete->pReturnParam[0];

                    //find which command this command complete is for
                    switch (command_complete->cmdOpcode)
                    {
                        case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                        {
                            if (pktStatus == SUCCESS)
                            {
                                uint8_t featSet[8];

                                // Get current feature set from received event (bits 1-9
                                // of the returned data
                                memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                                // Clear bit 1 of byte 0 of feature set to disable LL
                                // Connection Parameter Updates
                                CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                                // Update controller with modified features
                                HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                            }
                        }
                        break;

                        default:
                        //do nothing
                        break;
                    }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

            case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
                AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

            default:
            break;
            }
        }
        break;

        default:
        // do nothing
        break;

    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleEddystoneBeacon_processGATTMsg(gattMsgEvent_t *pMsg)
{
    // See if GATT server was unable to transmit an ATT response
    if (attRsp_isAttRsp(pMsg))
    {
        // No HCI buffer was available. Let's try to retransmit the response
        // on the next connection event.
        if( SimpleEddystoneBeacon_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
        {
            // Don't free the response message yet
            return (FALSE);
        }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        Display_print1(dispHandle, 5, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleEddystoneBeacon_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

    if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
    {
        // The GATT server might have returned a blePending as it was trying
        // to process an ATT Response. Now that we finished with this
        // connection event, let's try sending any remaining ATT Responses
        // on the next connection event.
        // Try to retransmit pending ATT Response (if any)
        if (attRsp_sendAttRsp() == SUCCESS)
        {
            // Disable connection event end notice
            SimpleEddystoneBeacon_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);
        }
    }

}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processAdvCompleteEvt
 *
 * @brief   Notification of a compleletion of advertise packet transmission.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_processAdvCompleteEvt(void)
{
    advCount++;

    if (URLCfgMode != TRUE)
    {
        if ((advCount % 10) == 0)
        {
            // Send TLM frame every 100 advertise packets
            SimpleEddystoneBeacon_selectFrame(EDDYSTONE_FRAME_TYPE_TLM);
        }
        else
        {
            // Send UID or URL
            SimpleEddystoneBeacon_selectFrame(currentFrameType);
        }
    }
}


/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_processAppMsg(sbpEvt_t *pMsg)
{
    switch (pMsg->hdr.event)
    {
        case SBP_STATE_CHANGE_EVT:
        {
            SimpleEddystoneBeacon_processStateChangeEvt((gaprole_States_t)pMsg->
                                                    hdr.state);
        }
        break;

        case SBP_CHAR_CHANGE_EVT:
        {
            SimpleEddystoneBeacon_processCharValueChangeEvt(pMsg->hdr.state);
        }
        break;

        // Pairing event
        case SBP_PAIRING_STATE_EVT:
        {
            SimpleEddystoneBeacon_processPairState(pMsg->hdr.state, *pMsg->pData);

            ICall_free(pMsg->pData);
            break;
        }

        // Passcode event
        case SBP_PASSCODE_NEEDED_EVT:
        {
            SimpleEddystoneBeacon_processPasscode(*pMsg->pData);

            ICall_free(pMsg->pData);
            break;
        }

        case SBP_CONN_EVT:
        {
            SimpleEddystoneBeacon_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));

            ICall_free(pMsg->pData);
            break;
        }

        default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_stateChangeCB(gaprole_States_t newState)
{
    SimpleEddystoneBeacon_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_processStateChangeEvt(gaprole_States_t newState)
{
    switch ( newState )
    {
        case GAPROLE_STARTED:
        {
            uint8_t ownAddress[B_ADDR_LEN];
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = ownAddress[0];
            systemId[1] = ownAddress[1];
            systemId[2] = ownAddress[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = ownAddress[5];
            systemId[6] = ownAddress[4];
            systemId[5] = ownAddress[3];

            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

            // Display device address
            Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
            Display_print0(dispHandle, 2, 0, "Initialized");

            // Start Config advertising
            URLCfgMode = TRUE;
            SimpleEddystoneBeacon_startConfigAdv();
            Util_startClock(&periodicClock);
        }
        break;


        case GAPROLE_ADVERTISING:
            Display_print0(dispHandle, 2, 0, "Config Mode");
        break;

        case GAPROLE_ADVERTISING_NONCONN:
            attRsp_freeAttRsp(bleNotConnected);

            if (currentFrameType == EDDYSTONE_FRAME_TYPE_UID)
            {
                Display_print0(dispHandle, 2, 0, "Advertising UID");
            }
            else
            {
                Display_print0(dispHandle, 2, 0, "Advertising URL");
            }

        break;

        case GAPROLE_CONNECTED:
        {
            linkDBInfo_t linkInfo;
            uint8_t numActive = 0;

            Util_startClock(&periodicClock);

            numActive = linkDB_NumActive();

            // Use numActive to determine the connection handle of the last
            // connection
            if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
            {
                Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
                Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
            }
            else
            {
                uint8_t peerAddress[B_ADDR_LEN];

                GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

                Display_print0(dispHandle, 2, 0, "Connected");
                Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
            }

            ConnectedInCfgMode = TRUE;
            Util_stopClock(&periodicClock);
        }
        break;

        case GAPROLE_CONNECTED_ADV:
            Display_print0(dispHandle, 2, 0, "Connected Advertising");
        break;

        case GAPROLE_WAITING:
            Util_stopClock(&periodicClock);
            attRsp_freeAttRsp(bleNotConnected);

            if (ConnectedInCfgMode == TRUE)
            {
                ConnectedInCfgMode = FALSE;
                URLCfgMode = FALSE;
                SimpleEddystoneBeacon_startRegularAdv();
            }

            Display_print0(dispHandle, 2, 0, "Disconnected");

            // Clear remaining lines
            Display_clearLines(dispHandle, 3, 5);
        break;

        case GAPROLE_WAITING_AFTER_TIMEOUT:
            attRsp_freeAttRsp(bleNotConnected);

            if (ConnectedInCfgMode == TRUE)
            {
                ConnectedInCfgMode = FALSE;
                URLCfgMode = FALSE;
                SimpleEddystoneBeacon_startRegularAdv();
            }

            Display_print0(dispHandle, 2, 0, "Timed Out");

            // Clear remaining lines
            Display_clearLines(dispHandle, 3, 5);

        break;

        case GAPROLE_ERROR:
            Display_print0(dispHandle, 2, 0, "Error");
        break;

        default:
            Display_clearLine(dispHandle, 2);
        break;
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_charValueChangeCB(uint8_t paramID)
{
    SimpleEddystoneBeacon_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, 0);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_processCharValueChangeEvt(uint8_t paramID)
{
    switch(paramID)
    {
        case URLCFGSVC_RESET:
            SimpleEddystoneBeacon_initConfiguration();
        break;

        default:
        // should not reach here!
        break;
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_performPeriodicTask(void)
{
    if (URLCfgMode)
    {
        if (ConnectedInCfgMode)
        {
            GAPRole_TerminateConnection();
            ConnectedInCfgMode = FALSE;
        }

        SimpleEddystoneBeacon_startRegularAdv();
        URLCfgMode = FALSE;
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_pairStateCB(uint16_t connHandle, uint8_t state,
                                            uint8_t status)
{
    uint8_t *pData;

    // Allocate space for the event data.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = status;

        // Queue the event.
        SimpleEddystoneBeacon_enqueueMsg(SBP_PAIRING_STATE_EVT, state, pData);
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_processPairState(uint8_t state, uint8_t status)
{
    if (state == GAPBOND_PAIRING_STATE_STARTED)
    {
        Display_print0(dispHandle, 2, 0, "Pairing started");
    }
    else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if (status == SUCCESS)
        {
            Display_print0(dispHandle, 2, 0, "Pairing success");
        }
        else
        {
            Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_BONDED)
    {
        if (status == SUCCESS)
        {
            Display_print0(dispHandle, 2, 0, "Bonding success");
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
    {
        if (status == SUCCESS)
        {
            Display_print0(dispHandle, 2, 0, "Bond save success");
        }
        else
        {
            Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
        }
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                           uint8_t uiInputs, uint8_t uiOutputs)
{
    uint8_t *pData;

    // Allocate space for the passcode event.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = uiOutputs;

        // Enqueue the event.
        SimpleEddystoneBeacon_enqueueMsg(SBP_PASSCODE_NEEDED_EVT, 0, pData);
    }
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleEddystoneBeacon_processPasscode(uint8_t uiOutputs)
{
    // This app uses a default passcode. A real-life scenario would handle all
    // pairing scenarios and likely generate this randomly.
    uint32_t passcode = B_APP_DEFAULT_PASSCODE;

    // Display passcode to user
    if (uiOutputs != 0)
    {
        Display_print1(dispHandle, 4, 0, "Passcode: %d", passcode);
    }

    uint16_t connectionHandle;
    GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

    // Send passcode response
    GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleEddystoneBeacon_clockHandler(UArg arg)
{
    // Wake up the application.
    Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimpleEddystoneBeacon_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimpleEddystoneBeacon_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
    // Enqueue the event for processing in the app context.
    if( SimpleEddystoneBeacon_enqueueMsg(SBP_CONN_EVT, 0 ,(uint8_t *) pReport) == FALSE)
    {
        ICall_free(pReport);
    }
}

/*********************************************************************
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleEddystoneBeacon_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
    sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

    // Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    }

    return FALSE;
}
/*********************************************************************
*********************************************************************/
