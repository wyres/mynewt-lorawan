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

#include <bsp/bsp.h>
#include <assert.h>
#include <string.h>

#include "LoRaMac.h"
#include "lorawan_api/lorawan_api.h"
#include "lorawan_api/lorawan_api_private.h"

#include "hal/hal_bsp.h"

#include "queue-board.h"

#if (MYNEWT_VAL(LORAWAN_API_TRACE_ACTIVATION) == 1)
extern void log_debug_fn(const char*, ...);
#define lorawan_printf log_debug_fn
//#include <stdio.h>
//#define lorawan_printf(...) printf(__VA_ARGS__)
#else
#define lorawan_printf(...)  {do{}while(0);}
#endif

//TODO: never defined in another place ?
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

extern struct sock_el* _lorawan_find_el(lorawan_sock_t sock);
extern SLIST_HEAD(s_socket, sock_el) l_sock_list;

extern LoRaMacStatus_t lorawan_api_private_reset(LoRaMacRegion_t region);

extern void lorawan_api_flush_queue(struct os_eventq *sock_eventq);

extern lorawan_region_t current_region;

lorawan_join_t mlme_req_join;

static lorawan_activation_mode_t current_mode = LORAWAN_ACTIVATION_MODE_UNINIT;

/*
 * Initialize the MCPS struct
 */
static void _lorawan_init_mcps(McpsReq_t* mcps_req){
#if MYNEWT_VAL(LORAWAN_API_DEFAULT_MCPS_CONFIRMED)
    mcps_req->Type = MCPS_CONFIRMED;
    mcps_req->Req.Confirmed.Datarate = MYNEWT_VAL(LORAWAN_API_DEFAULT_DR);
    mcps_req->Req.Confirmed.NbTrials = MYNEWT_VAL(LORAWAN_API_DEFAULT_NB_TRIALS);
#else
    mcps_req->Type = MCPS_UNCONFIRMED;
    mcps_req->Req.Unconfirmed.Datarate = MYNEWT_VAL(LORAWAN_API_DEFAULT_DR);
#endif
}

/*
 * Create a LoRaWAN socket: mandatory before any Tx or Rx.
 * return: the identifier of the socket allocated. NULL if an error occurs.
 */
lorawan_sock_t lorawan_socket(lorawan_socket_type_t type)
{
    static uint32_t sock_cpt = 1;
    struct sock_el* sock_el;

    /* Allocate one element on the list */
    sock_el = malloc( sizeof(struct sock_el) );

    if(sock_el == NULL)
        return 0;

    /* Clean the socket */
    memset(sock_el, 0, sizeof(struct sock_el));

    /* Create a unique sock id */
    sock_el->sock = sock_cpt;
    sock_cpt++;

    /* Initialize MCPS_req to the default values */
    _lorawan_init_mcps(&(sock_el->mcps_req));

    /* Init the event queue of this socket */
    os_eventq_init(&(sock_el->sock_eventq));

    /* Init the semaphore of this socket */
    os_sem_init(&(sock_el->sock_sem), 1);

    /* Init the socket type */
    sock_el->socket_type = type;

    SLIST_INSERT_HEAD(&l_sock_list, sock_el, sc_next);

    return sock_el->sock;
}

/*
 * Close a LoRaWAN socket (a socket is rarely removed in a classic app).
 * Warning: be sure that no actions are in progress on the socket
 * return: status of the operation (0=success / other=errors).
 */
int lorawan_close(lorawan_sock_t socket_id)
{
    struct sock_el* p_sock_el;
    p_sock_el = _lorawan_find_el(socket_id);

    if( p_sock_el == NULL )
        return -1;

    /* Flush the socket queue */
    lorawan_api_flush_queue(&(p_sock_el->sock_eventq));

    /* Release the socket semaphore */
    if(p_sock_el->sock_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
        os_sem_release( &(p_sock_el->sock_sem) );

    /* Remove the event queue from the socket list */
    SLIST_REMOVE(&l_sock_list, p_sock_el, sock_el, sc_next);

    /* Finally free the socket */
    free(p_sock_el);

    return 0;
}

/*
 * Put a LoRaWAN MCPS message in the queue.
 * return: the result of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_send(lorawan_sock_t sock,
                              uint8_t port,
                              uint8_t* payload,
                              uint8_t payload_size)
{
    return lorawan_send_extended(sock, LORAWAN_FRAME_TYPE_MCPS,
                                 port, payload, payload_size);
}

/*
 * Put a LoRaWAN message of desired type in the queue.
 * return: the result of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_send_extended(lorawan_sock_t sock,
                                       lorawan_frame_type_t type,
                                       uint8_t port,
                                       uint8_t* payload,
                                       uint8_t payload_size)
{
    lorawan_tx_continuous_t tx_cont;
    struct sock_el* sock_el = _lorawan_find_el(sock);
    struct os_event* ev;
    if( sock_el == NULL )
        return LORAWAN_STATUS_INVALID_SOCK;

    /* Set the type of the socket   */
    if (sock_el->socket_type == SOCKET_TYPE_RX){
        return LORAWAN_STATUS_INVALID_SOCK;
    }

    /* Update the socket data */
    if (type == LORAWAN_FRAME_TYPE_MCPS) {
        switch (sock_el->mcps_req.Type) {
            case MCPS_UNCONFIRMED:
                sock_el->mcps_req.Req.Unconfirmed.fBuffer = payload;
                sock_el->mcps_req.Req.Unconfirmed.fBufferSize = payload_size;
                sock_el->mcps_req.Req.Unconfirmed.fPort = port;
                break;
            case MCPS_CONFIRMED:
                sock_el->mcps_req.Req.Confirmed.fBuffer = payload;
                sock_el->mcps_req.Req.Confirmed.fBufferSize = payload_size;
                sock_el->mcps_req.Req.Confirmed.fPort = port;
                break;
            case LORAWAN_MCPS_MULTICAST:
                /*no break*/
            case LORAWAN_MCPS_PROPRIETARY:
                /*no break*/
            default:
                /* Frame type not managed yet */
                return LORAWAN_STATUS_INVALID_PARAM;
        }
    } else {
        switch (sock_el->mlme_req.Type) {
            case LORAWAN_MLME_JOIN:
                sock_el->mlme_req.Req.Join = *(MlmeReqJoin_t*)payload;
                break;
            case LORAWAN_MLME_LINK_CHECK:
                /* No payload */
                break;
            case LORAWAN_MLME_TXCW:
                memcpy(&sock_el->mlme_req.Req.TxCw.Timeout, payload,
                       sizeof(sock_el->mlme_req.Req.TxCw.Timeout));
                break;
            case LORAWAN_MLME_TXCW_1:
                tx_cont = *(lorawan_tx_continuous_t *)payload;
                sock_el->mlme_req.Req.TxCw.Timeout = tx_cont.timeout;
                sock_el->mlme_req.Req.TxCw.Frequency = tx_cont.frequency;
                sock_el->mlme_req.Req.TxCw.Power = tx_cont.tx_power;
                break;
            default:
                /* Frame type not managed yet */
                return LORAWAN_STATUS_INVALID_PARAM;
        }
    }

    sock_el->type = type;

    /* Clear the last known event */
    sock_el->last_event = LORAWAN_EVENT_NONE;

    /* Force the token release, case with previous TX without wait event */
    if(sock_el->sock_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
        os_sem_release( &(sock_el->sock_sem) );

    /* Take the token, it will be released by the wait event
     * or before the next TX */
    os_sem_pend( &(sock_el->sock_sem), 0 );

    /* Send an event in the queue to generate the TX */
    ev = malloc( sizeof(struct os_event) );
    if(ev != NULL){
        ev->ev_arg = (void *)sock;
        ev->ev_queued = 0;
        os_eventq_put(os_eventq_lorawan_api_get(), ev);
    } else {
        // BW return error!
        return LORAWAN_STATUS_ERROR;
    }

    return LORAWAN_STATUS_OK;
}

/*
 * Allow the current thread to wait an event from the previous Tx.
 * return: the unlock cause
 * ( Blocking function )
 */
lorawan_event_t lorawan_wait_ev(lorawan_sock_t sock, lorawan_event_t ev, uint32_t timeout_ms){
    struct sock_el* sock_el = _lorawan_find_el(sock);
    lorawan_event_t event;
    os_error_t res;

    if (sock_el == NULL)
        return LORAWAN_EVENT_ERROR;

    //TODO check the ev

    do {
        /* Wait for the token representing an RX event */
        res = os_sem_pend( &(sock_el->sock_sem), timeout_ms);
        event = sock_el->last_event & ev;
        /* Release the token */
        os_sem_release( &(sock_el->sock_sem) );

        if (res == OS_TIMEOUT) {
            return LORAWAN_EVENT_NONE;
        }
    } while (event == 0);

    return event;
}

lorawan_event_t lorawan_get_ev(lorawan_sock_t sock) {
    struct sock_el* sock_el = _lorawan_find_el(sock);

    if (sock_el == NULL)
        return LORAWAN_STATUS_INVALID_SOCK;

    return sock_el->last_event;
}

/*
 * Configure the LoRaWAN in ABP mode.
 * return: the status of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_configure_ABP(uint32_t devAddr, uint8_t* nwkSkey,
                                       uint8_t* appSkey, uint8_t nb_rep,
                                       int8_t tx_pow, lorawan_region_t region){
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;
    MibRequestConfirm_t mibReq;

    /* Reset the stack */
    status |= lorawan_api_private_reset((LoRaMacRegion_t)region);

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = devAddr;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = nwkSkey;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = appSkey;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = tx_pow;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = nb_rep;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

#if CLASS_C_ENABLE
    LoRaMacStatus_t state;
    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class = CLASS_C;
    state = LoRaMacMibSetRequestConfirm( &mibReq );
    lorawan_printf("classC:%d\r\n", state);
#endif

#if MYNEWT_VAL(SX1272) || MYNEWT_VAL(SX1276)
    mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mibReq.Param.SystemMaxRxError = (MYNEWT_VAL(SX127X_RADIO_MIN_RX_DURATION))/2;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );
#endif

    if(status == LORAMAC_STATUS_OK) {
        current_region = region;
        current_mode = LORAWAN_ACTIVATION_MODE_ABP;
        return LORAWAN_STATUS_OK;
    } else {
        return LORAWAN_STATUS_ERROR;
    }
}

/*
 * Configure the LoRaWAN in OTAA mode.
 * return: the status of the action.
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_configure_OTAA(uint8_t* devEUI, uint8_t* appEUI,
                                        uint8_t* appkey, uint8_t nb_rep,
                                        int8_t tx_pow, lorawan_region_t region){
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;
    MibRequestConfirm_t mibReq;

    /* Reset the stack */
    status |= lorawan_api_private_reset((LoRaMacRegion_t)region);

    mlme_req_join.DevEui = devEUI;
    mlme_req_join.AppEui = appEUI;
    mlme_req_join.AppKey = appkey;
    mlme_req_join.Datarate = MYNEWT_VAL(LORAWAN_API_DEFAULT_DR);

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = false;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = tx_pow;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_CHANNELS_NB_REP;
    mibReq.Param.ChannelNbRep = nb_rep;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );

#if MYNEWT_VAL(SX1272) || MYNEWT_VAL(SX1276)
    mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mibReq.Param.SystemMaxRxError = (MYNEWT_VAL(SX127X_RADIO_MIN_RX_DURATION))/2;
    status |= LoRaMacMibSetRequestConfirm( &mibReq );
#endif

    if(status == LORAMAC_STATUS_OK) {
        current_region = region;
        current_mode = LORAWAN_ACTIVATION_MODE_OTAA;
        return LORAWAN_STATUS_OK;
    } else {
        return LORAWAN_STATUS_ERROR;
    }
}

/*
 * Bind a specific port to the socket (It is allowed to bind several ports on the same socket).
 *   - devAddr is requested to allow specific binds on multicast addresses.
 *   - Several binds can be done on the same socket.
 *   - A devAddr/port couple can be binded to only one socket.
 * return: status of the operation
 * ( Non-blocking function )
 */
lorawan_status_t lorawan_bind(lorawan_sock_t sock, uint32_t devAddr, uint8_t port){
    uint8_t slot, position;

    struct sock_el* sock_el = _lorawan_find_el(sock);
    if( sock_el == NULL )
        return LORAWAN_STATUS_INVALID_SOCK;

    if( sock_el->devAddr == 0)
        sock_el->devAddr = devAddr;

    if( sock_el->devAddr != devAddr)
        return LORAWAN_STATUS_ERROR;

    if( port == 0 )
        return LORAWAN_STATUS_PORT_ALREADY_USED;

    // Check if the devAddr/port is not binded by someone else !
    if( lorawan_find_sock_by_params(devAddr, port) != 0 ){
        return LORAWAN_STATUS_PORT_ALREADY_USED;
    }

    /* Find the correct port slot */
    slot = port/32;
    position = port%32;
    sock_el->ports[slot] |= (1<<position);

    return LORAWAN_STATUS_OK;
}

/*
 * Wait for a packet on a specific socket.
 *   - devAddr / port / payload fields will be filled with the received data if it is valid.
 * return: size of data received. 0 if timeout occurs.
 * ( Blocking function )
 */
uint8_t lorawan_recv(lorawan_sock_t sock, uint32_t* devAddr, uint8_t* port, uint8_t* payload, uint8_t payload_max_len, uint32_t timeout_ms){
    uint8_t size = 0;
    struct os_event* ev;
    struct os_eventq *evq;
    McpsIndication_t* rx_data;
    os_time_t timo;
    int i;
    //TODO: block several lorawan_recv on the same socket

    struct sock_el* sock_el = _lorawan_find_el(sock);
    if( sock_el == NULL )
        return 0;

    /* Set the type of the socket   */
    if (sock_el->socket_type == SOCKET_TYPE_TX){
        return 0;
    }

    /* Check that one port (at least) is present */
    for(i=0; i<sizeof(sock_el->ports)/sizeof(sock_el->ports[0]); i++){
        if( sock_el->ports[i] != 0)
            break;
        else
            continue;
    }
    if( i == sizeof(sock_el->ports)/sizeof(sock_el->ports[0]) )
        return 0;

    evq = &(sock_el->sock_eventq);

    if((timeout_ms == 0) || ((int32_t)timeout_ms == OS_WAIT_FOREVER)) {
        timo = OS_WAIT_FOREVER;
    } else {
        timo = (timeout_ms*OS_TICKS_PER_SEC)/1000;
    }

    /* Wait either an event or the timeout */
    ev = os_eventq_poll(&evq, 1, timo);

    if(ev == NULL) //means timeout
        return 0;

    assert(ev->ev_arg != NULL);
    rx_data = (McpsIndication_t*)(ev->ev_arg);

    /* Feed all data */
    *devAddr = rx_data->DevAddr;
    *port = rx_data->Port;
    memcpy(payload, rx_data->Buffer, MIN(payload_max_len, rx_data->BufferSize) );
    memcpy(&(sock_el->mcps_ind), rx_data, sizeof(McpsIndication_t) );

    size = MIN(payload_max_len, rx_data->BufferSize);

    /* Finally, free the event */
    free(ev);

    return size;
}

/*
 * Get the current unicast devAddr used by the LoRaWAN stack.
 * return: the current devAddr used. 0 if error occurs.
 */
uint32_t lorawan_get_devAddr_unicast(void){
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEV_ADDR;
    if( LoRaMacMibGetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK )
        return mibReq.Param.DevAddr;
    else
        return 0;
}

/*
 * Add a multicast address to the devAddr pool.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_multicast_add(uint32_t devAddr, uint8_t* nwkSkey, uint8_t* appSkey, uint32_t downlink_counter){
    MulticastParams_t* mcast_param;

    mcast_param = malloc( sizeof(MulticastParams_t) );

    if(mcast_param == NULL)
        return LORAWAN_STATUS_ERROR;

    memcpy(&mcast_param->Address, &devAddr, sizeof(uint32_t));
    memcpy(&mcast_param->NwkSKey, nwkSkey, 16);
    memcpy(&mcast_param->AppSKey, appSkey, 16);
    mcast_param->DownLinkCounter = downlink_counter;
    mcast_param->Next = NULL;

    /* Remove the devAddr from the list, if it is present */
    lorawan_multicast_remove(devAddr);

    if( LoRaMacMulticastChannelLink(mcast_param) != LORAMAC_STATUS_OK ){
        free(mcast_param);
        return LORAWAN_STATUS_ERROR;
    }

    return LORAWAN_STATUS_OK;
}

/*
 * Remove a multicast address from the devAddr pool.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_multicast_remove(uint32_t devAddr){
    MibRequestConfirm_t mibReq;
    MulticastParams_t* mcast_el_cur;

    mibReq.Type = MIB_MULTICAST_CHANNEL;
    if( LoRaMacMibGetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK )
        return LORAWAN_STATUS_ERROR;

    mcast_el_cur = mibReq.Param.MulticastList;

    /* Search the devAddr in the list */
    while( mcast_el_cur != NULL ){
        if(mcast_el_cur->Address == devAddr)
            break;
        else
            mcast_el_cur = mcast_el_cur->Next;
    }

    /* devAddr not found or empty list */
    if( mcast_el_cur == NULL )
        return LORAWAN_STATUS_ERROR;

    /* Remove the address from the list */
    if( LoRaMacMulticastChannelUnlink(mcast_el_cur) != LORAMAC_STATUS_OK )
        return LORAWAN_STATUS_ERROR;

    /* Finally free the element */
    free(mcast_el_cur);

    return LORAWAN_STATUS_OK;
}

/*
 * Find the socket matching with the couple devAddr/port
 */
lorawan_sock_t lorawan_find_sock_by_params(uint32_t devAddr, uint8_t port)
{
    struct sock_el* i_list;
    uint8_t slot, position;
    slot = port/32;
    position = port%32;

    if( SLIST_EMPTY(&l_sock_list) )
        return 0;

    /* Search devAddr/Port on all sockets */
    for (i_list = SLIST_FIRST(&l_sock_list); i_list != NULL; i_list = SLIST_NEXT(i_list, sc_next)) {
        if( i_list->devAddr == devAddr){
            if( ( i_list->ports[slot] & (1<<position) ) != 0 ) { // "devAddr+Port" match !
                break;
//            } else {
//                lorawan_printf("findsock : matched devAddr but not port (%d) for skt (%d, %d, %02x)", port, slot, position, i_list->ports[slot]);
            }
//        } else {
//            lorawan_printf("findsock : no match for devAddr(%04x) for skt (%04x)", port, i_list->devAddr);
        }
    }

    // i_list is NULL if no match
    if( i_list == NULL )
        return 0;
    else
        return i_list->sock;
}

/*
 * Set an attribute of a LoRaWAN socket.
 * Note: always set the type before setting the datarate.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_setsockopt(lorawan_sock_t sock, lorawan_attribute_t *mibSet)
{
    LoRaMacStatus_t ret_code = 0;

    struct sock_el* sock_el = _lorawan_find_el(sock);
    if (sock_el == NULL)
        return LORAWAN_STATUS_INVALID_SOCK;

    if (mibSet==NULL)
        return LORAWAN_STATUS_INVALID_PARAM;

    switch (mibSet->Type) {
        case LORAWAN_ATTR_CHANNELS_DATARATE: {
            if (sock_el->mcps_req.Type == MCPS_UNCONFIRMED) {
                sock_el->mcps_req.Req.Unconfirmed.Datarate =
                        mibSet->Param.ChannelsDatarate;
            } else if (sock_el->mcps_req.Type == MCPS_CONFIRMED) {
                sock_el->mcps_req.Req.Confirmed.Datarate =
                        mibSet->Param.ChannelsDatarate;
            }
            mlme_req_join.Datarate = mibSet->Param.ChannelsDatarate;
            break;
        }
        case LORAWAN_ATTR_MCPS_TYPE: {
            sock_el->mcps_req.Type = (Mcps_t)mibSet->Param.McpsType;
            break;
        }
        case LORAWAN_ATTR_MLME_TYPE: {
            sock_el->mlme_req.Type = (Mlme_t)mibSet->Param.MlmeType;
            break;
        }
        case LORAWAN_ATTR_CHANNELS_NB_REP:
            if (sock_el->mcps_req.Type == MCPS_UNCONFIRMED) {
                ret_code = LoRaMacMibSetRequestConfirm((MibRequestConfirm_t *) mibSet);
            } else if (sock_el->mcps_req.Type == MCPS_CONFIRMED) {
                sock_el->mcps_req.Req.Confirmed.NbTrials =
                        mibSet->Param.ChannelNbRep;
            }
            break;
        default:
            ret_code = LoRaMacMibSetRequestConfirm((MibRequestConfirm_t *) mibSet);
            break;
    }

    if (ret_code == LORAMAC_STATUS_OK)
        return LORAWAN_STATUS_OK;
    else
        return LORAWAN_STATUS_ERROR;
}

/*
 * Get an attribute of a LoRaWAN socket.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_getsockopt (lorawan_sock_t sock,
        lorawan_attribute_t *mibGet)
{
    LoRaMacStatus_t ret_code = 0;

    MibRequestConfirm_t mibReq;

    struct sock_el* sock_el = _lorawan_find_el(sock);
    if (sock_el == NULL)
        return LORAWAN_STATUS_INVALID_SOCK;

    if (mibGet==NULL)
        return LORAWAN_STATUS_INVALID_PARAM;

    switch (mibGet->Type) {
        case LORAWAN_ATTR_CHANNELS_DATARATE: {
            /* Request to know if ADR is activated */
            mibReq.Type = MIB_ADR;
            ret_code = LoRaMacMibGetRequestConfirm(&mibReq);
            if (ret_code != LORAMAC_STATUS_OK)
                return LORAWAN_STATUS_ERROR;

            if (mibReq.Param.AdrEnable == 0) {
                /* ADR is not activated. Get Datarate from socket   */
                if (sock_el->mcps_req.Type == MCPS_UNCONFIRMED) {
                    mibGet->Param.ChannelsDatarate =
                            sock_el->mcps_req.Req.Unconfirmed.Datarate;
                } else if (sock_el->mcps_req.Type == MCPS_CONFIRMED) {
                    mibGet->Param.ChannelsDatarate =
                            sock_el->mcps_req.Req.Confirmed.Datarate;
                }
            } else {
                /* ADR is activated. Get Datarate from the LoRaMAC   */
                mibGet->Type = MIB_CHANNELS_DATARATE;
                ret_code = LoRaMacMibGetRequestConfirm(
                        (MibRequestConfirm_t *) mibGet);
                if (ret_code != LORAMAC_STATUS_OK)
                    return LORAWAN_STATUS_ERROR;
            }
            break;
        }
        case LORAWAN_ATTR_MCPS_TYPE: {
            mibGet->Param.McpsType = (lorawan_McpsType_t)sock_el->mcps_req.Type;
            break;
        }
        case LORAWAN_ATTR_MLME_TYPE: {
            mibGet->Param.MlmeType = (lorawan_MlmeType_t)sock_el->mlme_req.Type;
            break;
        }
        case LORAWAN_ATTR_CHANNELS_NB_REP:
            if (sock_el->mcps_req.Type == MCPS_UNCONFIRMED) {
                ret_code = LoRaMacMibGetRequestConfirm((MibRequestConfirm_t *) mibGet);
            } else if (sock_el->mcps_req.Type == MCPS_CONFIRMED) {
                mibGet->Param.ChannelNbRep =
                        sock_el->mcps_req.Req.Confirmed.NbTrials;
            }
            break;
        default:
            ret_code = LoRaMacMibGetRequestConfirm((MibRequestConfirm_t *) mibGet);
            break;
    }

    if (ret_code == LORAMAC_STATUS_OK)
        return LORAWAN_STATUS_OK;
    else
        return LORAWAN_STATUS_ERROR;
}

/*
 * Get information of a LoRaWAN socket.
 * return: status code of the operation.
 */
lorawan_status_t lorawan_getsockinfo (lorawan_sock_t sock, struct lorawan_sockinfo_t *p_sockinfo)
{
    struct sock_el* sock_el = _lorawan_find_el(sock);
    if ( (sock_el == NULL) || (p_sockinfo == NULL) )
        return LORAWAN_STATUS_INVALID_SOCK;

    p_sockinfo->devAddr = sock_el->devAddr;
    p_sockinfo->last_rx_update = sock_el->last_rx_update;
    p_sockinfo->ack_received = sock_el->mcps_ind.AckReceived;
    p_sockinfo->DLcounter = sock_el->mcps_ind.DownLinkCounter;
    p_sockinfo->rssi = sock_el->mcps_ind.Rssi;
    p_sockinfo->snr = sock_el->mcps_ind.Snr;

    if(sock_el->mcps_ind.Port == 0)
        p_sockinfo->empty_msg = true;
    else
        p_sockinfo->empty_msg = false;

    return LORAWAN_STATUS_OK;
}

//TODO: check this
void lorawan_set_dutycycle(bool enabled)
{
    LoRaMacTestSetDutyCycleOn(enabled);
}

/*
 * Retrieve the region currently configured
 */
lorawan_region_t lorawan_get_current_region(void)
{
    return current_region;
}

/*
 * Retrieve the activation mode currently configured
 */
lorawan_activation_mode_t lorawan_get_current_activation_mode(void)
{
    return current_mode;
}
