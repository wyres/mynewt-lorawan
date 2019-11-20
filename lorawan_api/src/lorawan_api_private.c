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
#define lorawan_printf(...) {do{}while(0);}
#endif

static struct os_task lorawan_eventq_task;
static os_stack_t lorawan_eventq_stack[OS_STACK_ALIGN(LORAWAN_STACK_SIZE)];
static void lorawan_eventq_thread (void* data);

static struct os_task lorawan_api_eventq_task;
static os_stack_t lorawan_api_eventq_stack[OS_STACK_ALIGN(LORAWAN_API_STACK_SIZE)];
static void lorawan_api_eventq_thread (void* data);

void lorawan_api_flush_queue(struct os_eventq *sock_eventq);

static struct os_sem mlme_done_sem;
static struct os_sem mcps_done_sem;

/* Event queue for the LoRaWAN API */
static struct os_eventq os_eventq_lorawan_api;

static uint32_t current_devAddr = 0;

lorawan_region_t current_region;

extern lorawan_join_t mlme_req_join;

/**
 * Retrieves the event queue used by the LoRaWAN API.
 *
 * @return                      The event queue.
 */
struct os_eventq *
os_eventq_lorawan_api_get(void)
{
    return &os_eventq_lorawan_api;
}


/*
 * Socket list head pointer
 */
SLIST_HEAD(s_socket, sock_el) l_sock_list =
    SLIST_HEAD_INITIALIZER();

struct sock_el* _lorawan_find_el(lorawan_sock_t sock){
    struct sock_el* i_list;

    /* Find the matching element */
    for (i_list = SLIST_FIRST(&l_sock_list); i_list != NULL; i_list = SLIST_NEXT(i_list, sc_next)) {
        if (i_list->sock == sock)
            break;
    }
    /* If a valid sock has been found, the struct ptr is returned */
    /* If no valid sock has been found, the last value is NULL, and is returned */
    return i_list;
}

/* Primitive definitions used by the LoRaWAN */
static void _mcps_confirm ( McpsConfirm_t *McpsConfirm ){
    lorawan_printf("MCPSconfirm: %d\r\n", McpsConfirm->AckReceived);
    // TODO should find socket that sent tx and set event ACK in its last_event mask!
    os_sem_release( &(mcps_done_sem) );
}

static void _mcps_indication ( McpsIndication_t *McpsIndication ){
    struct sock_el* i_list;
    struct os_event* ev;
    lorawan_printf("MCPSind (%d/%d)\r\n", McpsIndication->Status, McpsIndication->Port);

    if( McpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK){
        lorawan_printf("status not OK\r\n");
        return;
    }

#if 0
    int i;
    lorawan_printf("$ LoRaWAN Rx Data: [devAddr:%08lx] [ack:%d] [Fcnt:%lu] [Fpend:%u] [mCast:%u] [port:%u] [rssi:%d] [snr:%u] [slot:%d]\r\n",
            McpsIndication->DevAddr,
            McpsIndication->AckReceived,
            McpsIndication->DownLinkCounter,
            McpsIndication->FramePending,
            McpsIndication->Multicast,
            McpsIndication->Port,
            McpsIndication->Rssi,
            McpsIndication->Snr,
            McpsIndication->RxSlot
    );

    //print data if there are
    if(McpsIndication->BufferSize != 0){
        lorawan_printf("$ Payload Data [size:%u]:\r\n", McpsIndication->BufferSize);
        for(i=0;i<McpsIndication->BufferSize;i++)
            lorawan_printf("%02x", McpsIndication->Buffer[i]);
        lorawan_printf("\r\n");
    }
#endif

    if( McpsIndication->Port != 0 ){
        /* Search for a valid socket on the devAddr/port to forward payload */
        i_list = _lorawan_find_el( lorawan_find_sock_by_params(McpsIndication->DevAddr, McpsIndication->Port) );
        if(i_list == NULL){ //drop the packet, no match...
            return;
        }

        /* Feed information for sockinfo */
        memcpy( &(i_list->mcps_ind), McpsIndication, sizeof(i_list->mcps_ind) );
        i_list->last_rx_update = os_time_get();

        /* Update events */
        if (McpsIndication->AckReceived) {
            i_list->last_event |= LORAWAN_EVENT_ACK;
        }
        i_list->last_event |= LORAWAN_EVENT_PENDING_RX;

        ev = malloc( sizeof(struct os_event) );
        if(ev != NULL){
            ev->ev_arg = McpsIndication;//TODO: memcpy data to allow multiple treatments.
            ev->ev_queued = 0;
            os_eventq_put(&(i_list->sock_eventq), ev);
        } else {
            lorawan_printf("failed to malloc event to signal rx pkt\r\n");
        }
    }
    else{ /* Means that neither Fport, nor Payload, are in the frame -> notify all socket with the correct devAddr */
        /* Search for a valid socket on the devAddr, to update all metadatas (rssi, snr, ack...) */
        for (i_list = SLIST_FIRST(&l_sock_list); i_list != NULL; i_list = SLIST_NEXT(i_list, sc_next)) {
            if (i_list->socket_type != SOCKET_TYPE_RX){
                continue;
            }
            if (i_list->devAddr == McpsIndication->DevAddr){
                /* Update all sockinfo, before notify sockets */
                memcpy( &(i_list->mcps_ind), McpsIndication, sizeof(i_list->mcps_ind) );
                i_list->last_rx_update = os_time_get();

                /* Notify all valid sockets */
                ev = malloc( sizeof(struct os_event) );
                if(ev != NULL){
                    ev->ev_arg = McpsIndication;//TODO: memcpy data to allow multiple treatments.
                    ev->ev_queued = 0;
                    os_eventq_put(&(i_list->sock_eventq), ev);
                }
            }
        }
    }
}

static void _mlme_confirm( MlmeConfirm_t *MlmeConfirm ){
    struct sock_el* i_list;
    MibRequestConfirm_t mibReq;

    lorawan_printf("MLMEconf %d/%d\r\n", MlmeConfirm->MlmeRequest,
                                 MlmeConfirm->Status);

    if ((MlmeConfirm->MlmeRequest == MLME_JOIN) &&
        (MlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)) {
        lorawan_printf("Join Accepted\r\n");

        /* Retrieve the new devAddr given to the end-device */
        mibReq.Type = MIB_DEV_ADDR;
        LoRaMacMibGetRequestConfirm( &mibReq );

        /* Search for all sockets with the current devAddr and update it */
        for (i_list = SLIST_FIRST(&l_sock_list); i_list != NULL; i_list = SLIST_NEXT(i_list, sc_next)) {
            if (i_list->devAddr == current_devAddr) {
                i_list->devAddr = mibReq.Param.DevAddr;
            }
        }

        /* Update current DevAddr */
        current_devAddr = mibReq.Param.DevAddr;
    }

    /* Release the MLME action token to resume the TX thread execution */
    os_sem_release( &mlme_done_sem );
}

static void _mlme_indication( MlmeIndication_t *MlmeIndication ){
    lorawan_printf("MLMEind\r\n");
}

static LoRaMacPrimitives_t _lorawan_primitives = {
        _mcps_confirm,
        _mcps_indication,
        _mlme_confirm,
        _mlme_indication
};

static uint8_t _get_battery_level( void ){
    // TODO: link this with the HAL
    return 255;
}
static LoRaMacCallback_t _lorawan_callbacks = { _get_battery_level };

static LoRaMacRegion_t lorawan_get_first_active_region( void ){
    LoRaMacRegion_t region;
    for(region=0; region != LORAMAC_REGION_US915_HYBRID; region++){
        if( RegionIsActive( region ) == true )
            return region;
    }
    return 0;
}

LoRaMacStatus_t lorawan_api_private_reset(LoRaMacRegion_t region){
    LoRaMacStatus_t status;

    /* Flush queues */
    lorawan_api_flush_queue(os_eventq_lorawan_get());
    lorawan_api_flush_queue(os_eventq_lorawan_api_get());

    /* Release semaphores */
    if(mlme_done_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
        os_sem_release( &mlme_done_sem );
    if(mcps_done_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
        os_sem_release( &mcps_done_sem );

    status = LoRaMacInitialization(&_lorawan_primitives, &_lorawan_callbacks,
                                   region);
    return status;
}

void lorawan_api_private_init(void){
    LoRaMacStatus_t status;
    LoRaMacRegion_t region;

    /* Initialize the LoRaWAN event queues and semaphores */
    os_eventq_init( os_eventq_lorawan_get() );
    os_eventq_init( os_eventq_lorawan_api_get() );

    os_sem_init( &mlme_done_sem, 1 );
    os_sem_init( &mcps_done_sem, 1 );


    /* Create the LoRaWAN to treat the event queues */
    os_task_init(&lorawan_eventq_task, "lw_eventq",
                 lorawan_eventq_thread, NULL,
                 LORAWAN_TASK_PRIO, OS_WAIT_FOREVER,
                 lorawan_eventq_stack,
                 OS_STACK_ALIGN(LORAWAN_STACK_SIZE));

    os_task_init(&lorawan_api_eventq_task, "lw_api_eventq",
                 lorawan_api_eventq_thread, NULL,
                 LORAWAN_API_TASK_PRIO, OS_WAIT_FOREVER,
                 lorawan_api_eventq_stack,
                 OS_STACK_ALIGN(LORAWAN_API_STACK_SIZE));

    region = lorawan_get_first_active_region();
    status = lorawan_api_private_reset(region);
    assert( status == LORAMAC_STATUS_OK);

    /* Everything is OK, update the region */
    current_region = region;
    USEDBYASSERT(status);
}

static void lorawan_eventq_thread (void* data)
{
    while (1) {
        /* Blocking call, unblocked by the reception of events */
        os_eventq_run( os_eventq_lorawan_get() );
    }
    assert(0);
}

#include "console/console.h"
static void lorawan_api_eventq_thread (void* data)
{
    struct os_event* ev;
    struct sock_el* sock_el;
    LoRaMacStatus_t status;
    MibRequestConfirm_t mibReq;

    while (1) {
        /* Blocking call, unblocked by the reception of events */
        ev = os_eventq_get( os_eventq_lorawan_api_get() );

        /* Retrieve the element associated to the socket Id */
        sock_el = _lorawan_find_el((lorawan_sock_t)ev->ev_arg);

        //TODO: not sure that we should reconfigure all of these mibReq on each Tx.
        //LoRaMacMibSetRequestConfirm( &mibReq );

retry_tx:
        /* Send the message */
        if (sock_el->type == LORAWAN_FRAME_TYPE_MCPS) {
            /* Take the MCPS done token */
            os_sem_pend( &mcps_done_sem, 0);

            status = LoRaMacMcpsRequest( &(sock_el->mcps_req) );
        } else {
            /* Take the MLME done token */
            os_sem_pend( &mlme_done_sem, 0);

            status = LoRaMacMlmeRequest( &(sock_el->mlme_req) );
        }

        switch(status) {
            /* Non-critical errors, actions can be done */
            case LORAMAC_STATUS_BUSY:
                lorawan_printf("Stack busy\r\n");
                os_time_delay(MYNEWT_VAL(LORAWAN_API_RETRY_TX_DELAY)* OS_TICKS_PER_SEC / 1000);
                goto retry_tx;
            case LORAMAC_STATUS_DUTYCYCLE_RESTRICTED:
                lorawan_printf("Dutycycle restriction\r\n");
                os_time_delay(MYNEWT_VAL(LORAWAN_API_RETRY_TX_DELAY)* OS_TICKS_PER_SEC / 1000);
                goto retry_tx;
            case LORAMAC_STATUS_NO_NETWORK_JOINED:
                lorawan_printf("Not joined - Send JoinRequest\r\n");

                /* Update the current devAddr */
                mibReq.Type = MIB_DEV_ADDR;
                LoRaMacMibGetRequestConfirm( &mibReq );
                current_devAddr = mibReq.Param.DevAddr;

                /* Take the join token */
                os_sem_pend( &mlme_done_sem, 0);

                /* Send a Join Request */
                sock_el->mlme_req.Type = MLME_JOIN;
                sock_el->mlme_req.Req.Join = *(MlmeReqJoin_t*)&mlme_req_join;

                status = LoRaMacMlmeRequest( &(sock_el->mlme_req) );

                if (status == LORAMAC_STATUS_OK) {
                    mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
                    LoRaMacMibGetRequestConfirm( &mibReq );

                    /* Wait for a Join Accept or a timeout from the stack */
                    os_sem_pend( &mlme_done_sem, OS_TIMEOUT_NEVER);

                    mibReq.Type = MIB_NETWORK_JOINED;
                    LoRaMacMibGetRequestConfirm( &mibReq );

                    if (mibReq.Param.IsNetworkJoined == false) {
                        /* Drop the message */
                        free(ev);
                        sock_el->last_event = LORAWAN_EVENT_ERROR;
                        lorawan_printf("Failed to join\r\n");
                        os_sem_release( &(sock_el->sock_sem) );
                    } else {
                        lorawan_printf("Retry to send the Frame\r\n");
                        goto retry_tx;
                    }
                } else { //TODO manage other potential return
                    if(status == LORAMAC_STATUS_DUTYCYCLE_RESTRICTED){
                        lorawan_printf("Dutycycle restriction\r\n");
                    }
                    /* Drop the message */
                    free(ev);
                    sock_el->last_event = LORAWAN_EVENT_ERROR;
                    os_sem_release( &(sock_el->sock_sem) );
                    os_sem_release( &mlme_done_sem );
                }
                break;

            /* TX done or a critical error in the stack occurred.
             * Update last known event, free the queue event and
             * release the semaphore token */
            case LORAMAC_STATUS_OK:
                free(ev);

                /* Wait the message and repetitions sending before
                 * resuming the execution */
                if (sock_el->type == LORAWAN_FRAME_TYPE_MCPS) {
                    os_sem_pend( &mcps_done_sem, OS_TIMEOUT_NEVER);
                    os_sem_release( &mcps_done_sem );
                } else {
                    os_sem_pend( &mlme_done_sem, OS_TIMEOUT_NEVER);
                    os_sem_release( &mlme_done_sem );
                }
                sock_el->last_event = LORAWAN_EVENT_SENT;
                os_sem_release( &(sock_el->sock_sem) );
                break;
            default:
                /* Drop the message */
                free(ev);
                sock_el->last_event = LORAWAN_EVENT_ERROR;
                /* Release semaphores */
                os_sem_release( &(sock_el->sock_sem) );
                if(mlme_done_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
                    os_sem_release( &mlme_done_sem );
                if(mcps_done_sem.sem_tokens == 0) //TODO remove workaround when fixed in MyNewt OS
                    os_sem_release( &mcps_done_sem );
                break;
        }
    }
    assert(0);
}

void lorawan_api_flush_queue(struct os_eventq *sock_eventq){

    struct os_event *ev;

    while ( (ev = os_eventq_get_no_wait(sock_eventq)) != NULL) {
        free(ev);
    }
}
