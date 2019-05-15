/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef __LORAWAN_API_H__
#define __LORAWAN_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * LoRaWAN socket type definition
 */
typedef uint32_t lorawan_sock_t;

/*
 * Status type definition
 */
typedef enum {
    LORAWAN_STATUS_OK = 0,
    LORAWAN_STATUS_INVALID_SOCK,
    LORAWAN_STATUS_INVALID_PARAM,
    LORAWAN_STATUS_PORT_ALREADY_USED,
    LORAWAN_STATUS_ERROR,
    LORAWAN_STATUS_PORT_BUSY,
} lorawan_status_t ;

/*
 * Socket type definition
 */
typedef enum {
    LORAWAN_EVENT_NONE          = 0,
    LORAWAN_EVENT_ERROR         = 1<<0,
    LORAWAN_EVENT_ACK           = 1<<1,
    LORAWAN_EVENT_SENT          = 1<<2,
    LORAWAN_EVENT_PENDING_RX    = 1<<3,
} lorawan_event_t ;

/*
 * Socket information
 */
struct lorawan_sockinfo_t {
    uint32_t last_rx_update;
    uint32_t devAddr;
    bool ack_received;
    bool empty_msg;
    uint32_t DLcounter;
    int16_t rssi;
    uint8_t snr;
};

/*
 * Socket type
 */
typedef enum {
    SOCKET_TYPE_TX = 0,
    SOCKET_TYPE_RX = 1,
} lorawan_socket_type_t ;

/*
 * Frame type definition
 */
typedef enum {
    LORAWAN_FRAME_TYPE_MCPS = 0,
    LORAWAN_FRAME_TYPE_MLME,
} lorawan_frame_type_t ;

/*
 * TX continuous structure
 */
typedef struct {
    uint32_t frequency;
    uint16_t timeout;
    uint8_t  tx_power;
}lorawan_tx_continuous_t;

/*
 * Join structure (copy of MlmeReqJoin_t)
 */
typedef struct {
    uint8_t *DevEui;
    uint8_t *AppEui;
    uint8_t *AppKey;
    uint8_t Datarate;
}lorawan_join_t;

typedef enum eLoRaWanRegion lorawan_region_t;

typedef enum {
    LORAWAN_ACTIVATION_MODE_UNINIT = 0,
    LORAWAN_ACTIVATION_MODE_ABP,
    LORAWAN_ACTIVATION_MODE_OTAA,
}lorawan_activation_mode_t;

/*******************************************************/
/*                                                     */
/*             COMMON FUNCTIONS                        */
/*                                                     */
/*******************************************************/

/*
 * Configure the LoRaWAN in ABP mode.
 * return: the status of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_configure_ABP(uint32_t devAddr, uint8_t* nwkSkey,
                                       uint8_t* appSkey, uint8_t nb_rep,
                                       int8_t tx_pow, lorawan_region_t region);

/*
 * Configure the LoRaWAN in OTAA mode.
 * return: the status of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_configure_OTAA(uint8_t* devEUI, uint8_t* appEUI,
                                        uint8_t* appkey, uint8_t nb_rep,
                                        int8_t tx_pow, lorawan_region_t region);

/*
 * Create a LoRaWAN socket: mandatory before any Tx or Rx.
 * return: the identifier of the socket allocated / 0 if an error occurs.
 */
lorawan_sock_t lorawan_socket(lorawan_socket_type_t type);

/*
 * Close a LoRaWAN socket (a socket is rarely removed in a classic app).
 * Warning: be sure that no actions are in progress on the socket
 * return: status of the operation (0=success / other=errors).
 */
int lorawan_close(lorawan_sock_t socket_id);


/*******************************************************/
/*                                                     */
/*                TX FUNCTIONS                         */
/*                                                     */
/*******************************************************/

/*
 * Put a LoRaWAN MCPS message in the queue.
 * return: the result of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_send(lorawan_sock_t sock, uint8_t port, uint8_t* payload, uint8_t payload_size);

/*
 * Put a LoRaWAN message of desired type in the queue.
 * return: the result of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_send_extended(lorawan_sock_t sock, lorawan_frame_type_t type, uint8_t port,
                                       uint8_t* payload, uint8_t payload_size);

/*
 * Allow the current thread to wait an event from the previous Tx.
 * return: the unlock cause
 * ( Blocking function )
 */
lorawan_event_t lorawan_wait_ev(lorawan_sock_t sock, lorawan_event_t ev, uint32_t timeout_ms);

/*
 * Allow the current thread to query the last known event of a socket
 * (same as the lorawan_wait_ev(), but non-blocking)
 * return: the last known event
 * ( Non-blocking function )
 */
lorawan_event_t lorawan_get_ev(lorawan_sock_t sock);


/*******************************************************/
/*                                                     */
/*                RX FUNCTIONS                         */
/*                                                     */
/*******************************************************/

/*
 * Bind a specific port to the socket (It is allowed to bind several ports on the same socket).
 *   - devAddr is requested to allow specific binds on multicast addresses.
 *   - Several binds can be done on the same socket.
 *   - A devAddr/port couple can be binded to only one socket.
 * return: status of the operation
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_bind(lorawan_sock_t sock, uint32_t devAddr, uint8_t port);

/*
 * Wait for a packet on a specific socket.
 *   - devAddr / port / payload fields will be filled with the received data if it is valid.
 * return: size of data received. 0 if timeout occurs.
 * ( Blocking function )
 */
uint8_t lorawan_recv(lorawan_sock_t sock, uint32_t* devAddr, uint8_t* port, uint8_t* payload, uint8_t payload_max_len, uint32_t timeout_ms);


/*******************************************************/
/*                                                     */
/*              ACCESSOR FUNCTIONS                     */
/*                                                     */
/*******************************************************/

/*
 * Get the current unicast devAddr used by the LoRaWAN stack.
 * return: the current devAddr used. 0 if error occurs.
 */
uint32_t lorawan_get_devAddr_unicast(void);

/*
 * Find the socket matching with the couple devAddr/port.
 * Useful to know if a couple devAddr/port is already binded.
 * return: the id of the matching socket (0 if no match).
 */
lorawan_sock_t lorawan_find_sock_by_params(uint32_t devAddr, uint8_t port);

/*
 * Add a multicast address to the devAddr pool.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_multicast_add(uint32_t devAddr, uint8_t* nwkSkey, uint8_t* appSkey, uint32_t downlink_counter);

/*
 * Remove a multicast address from the devAddr pool.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_multicast_remove(uint32_t devAddr);



/************ ATTRIBUTES ACCESSORS ********************/
//TODO: all of this section should be improved by adding a check on types

typedef struct sLorawanRx2ChannelParams
{
    uint32_t Frequency;
    uint8_t  Datarate;
} lorawan_Rx2ChannelParams_t;

typedef enum eLorawanDeviceClass
{
    LORAWAN_CLASS_A,
    LORAWAN_CLASS_B,
    LORAWAN_CLASS_C,
} lorawan_DeviceClass_t;

typedef enum eLorawanMcpsType
{
    LORAWAN_MCPS_UNCONFIRMED,
    LORAWAN_MCPS_CONFIRMED,
    LORAWAN_MCPS_MULTICAST,
    LORAWAN_MCPS_PROPRIETARY,
}lorawan_McpsType_t;

typedef enum eLorawanMlmeType
{
    LORAWAN_MLME_JOIN,
    LORAWAN_MLME_LINK_CHECK,
    LORAWAN_MLME_TXCW,
    LORAWAN_MLME_TXCW_1,
    LORAWAN_MLME_SCHEDULE_UPLINK
}lorawan_MlmeType_t;

typedef union uLorawanParam
{
    lorawan_DeviceClass_t Class;
    bool IsNetworkJoined;
    bool AdrEnable;
    uint32_t NetID;
    uint32_t DevAddr;
    uint8_t *NwkSKey;
    uint8_t *AppSKey;
    bool EnablePublicNetwork;
    bool EnableRepeaterSupport;
    void* ChannelList;//change to void*
    lorawan_Rx2ChannelParams_t Rx2Channel;
    lorawan_Rx2ChannelParams_t Rx2DefaultChannel;
    uint16_t* ChannelsMask;
    uint16_t* ChannelsDefaultMask;
    uint8_t ChannelNbRep;
    uint32_t MaxRxWindow;
    uint32_t ReceiveDelay1;
    uint32_t ReceiveDelay2;
    uint32_t JoinAcceptDelay1;
    uint32_t JoinAcceptDelay2;
    int8_t ChannelsDefaultDatarate;
    int8_t ChannelsDatarate;
    int8_t ChannelsDefaultTxPower;
    int8_t ChannelsTxPower;
    uint32_t UpLinkCounter;
    uint32_t DownLinkCounter;
    void* MulticastList;//change to void*
    uint32_t SystemMaxRxError;
    uint8_t MinRxSymbols;
    float AntennaGain;
    float DefaultAntennaGain;

    /* API specific parameters */
    lorawan_McpsType_t McpsType;
    lorawan_MlmeType_t MlmeType;
} lorawan_attr_Param_t;

typedef enum eLorawan
{
    LORAWAN_ATTR_DEVICE_CLASS,
    LORAWAN_ATTR_NETWORK_JOINED,
    LORAWAN_ATTR_ADR,
    LORAWAN_ATTR_NET_ID,
    LORAWAN_ATTR_DEV_ADDR,
    LORAWAN_ATTR_NWK_SKEY,
    LORAWAN_ATTR_APP_SKEY,
    LORAWAN_ATTR_PUBLIC_NETWORK,
    LORAWAN_ATTR_REPEATER_SUPPORT,
    LORAWAN_ATTR_RFU_1,//MIB_CHANNELS,
    LORAWAN_ATTR_RX2_CHANNEL,
    LORAWAN_ATTR_RX2_DEFAULT_CHANNEL,
    LORAWAN_ATTR_CHANNELS_MASK,
    LORAWAN_ATTR_CHANNELS_DEFAULT_MASK,
    LORAWAN_ATTR_CHANNELS_NB_REP,
    LORAWAN_ATTR_MAX_RX_WINDOW_DURATION,
    LORAWAN_ATTR_RECEIVE_DELAY_1,
    LORAWAN_ATTR_RECEIVE_DELAY_2,
    LORAWAN_ATTR_JOIN_ACCEPT_DELAY_1,
    LORAWAN_ATTR_JOIN_ACCEPT_DELAY_2,
    LORAWAN_ATTR_CHANNELS_DEFAULT_DATARATE,
    LORAWAN_ATTR_CHANNELS_DATARATE,
    LORAWAN_ATTR_CHANNELS_TX_POWER,
    LORAWAN_ATTR_CHANNELS_DEFAULT_TX_POWER,
    LORAWAN_ATTR_UPLINK_COUNTER,
    LORAWAN_ATTR_DOWNLINK_COUNTER,
    LORAWAN_ATTR_RFU_2,//MIB_MULTICAST_CHANNEL,
    LORAWAN_ATTR_SYSTEM_MAX_RX_ERROR,
    LORAWAN_ATTR_MIN_RX_SYMBOLS,
    LORAWAN_ATTR_ANTENNA_GAIN,
    LORAWAN_ATTR_DEFAULT_ANTENNA_GAIN,

    /* API specific attributes (must always be the last attributes) */
    LORAWAN_ATTR_MCPS_TYPE,
    LORAWAN_ATTR_MLME_TYPE
} lorawan_attr_t;

typedef struct eLorawanRequestConfirm
{
    lorawan_attr_t Type;
    lorawan_attr_Param_t Param;
} lorawan_MibRequestConfirm_t;

#define lorawan_attribute_t lorawan_MibRequestConfirm_t

enum eLoRaWanRegion
{
    LORAWAN_REGION_AS923,
    LORAWAN_REGION_AU915,
    LORAWAN_REGION_CN470,
    LORAWAN_REGION_CN779,
    LORAWAN_REGION_EU433,
    LORAWAN_REGION_EU868,
    LORAWAN_REGION_KR920,
    LORAWAN_REGION_IN865,
    LORAWAN_REGION_US915,
    LORAWAN_REGION_US915_HYBRID,
};


/*
 * Set an attribute of a LoRaWAN socket.
 * Note: always set the type before setting the datarate.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_setsockopt(lorawan_sock_t sock, lorawan_attribute_t *mibSet);

/*
 * Get an attribute of a LoRaWAN socket.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_getsockopt(lorawan_sock_t sock, lorawan_attribute_t *mibGet);

/*
 * Get information of a LoRaWAN socket.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_getsockinfo (lorawan_sock_t sock, struct lorawan_sockinfo_t *p_sockinfo);

/*
 * Enable/disable the dutycycle
 * Note: dutycycle of JoinRequest can not be disabled
 */
void lorawan_set_dutycycle(bool enabled);

/*
 * Retrieve the region currently configured
 */
lorawan_region_t lorawan_get_current_region(void);

/*
 * Retrieve the activation mode currently configured
 */
lorawan_activation_mode_t lorawan_get_current_activation_mode(void);

#ifdef __cplusplus
}
#endif

#endif /*  __LORAWAN_API_H__ */
