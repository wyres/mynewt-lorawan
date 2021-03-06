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

#ifndef __LORAWAN_API_PRIVATE_H__
#define __LORAWAN_API_PRIVATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "LoRaMac.h"
#include "queue-board.h"

#define LORAWAN_TASK_PRIO           MYNEWT_VAL(LORAWAN_TASK_PRIO)
#define LORAWAN_STACK_SIZE          MYNEWT_VAL(LORAWAN_STACK_SIZE)
#define LORAWAN_API_TASK_PRIO       MYNEWT_VAL(LORAWAN_API_TASK_PRIO)
#define LORAWAN_API_STACK_SIZE      MYNEWT_VAL(LORAWAN_API_STACK_SIZE)

/*!
 * Socket list structure definition
 */
struct sock_el {
    lorawan_sock_t sock;
    lorawan_frame_type_t type;
    McpsReq_t mcps_req;
    MlmeReq_t mlme_req;
    lorawan_event_t last_event;
    McpsIndication_t mcps_ind;
    uint32_t last_rx_update;
    uint32_t devAddr;//TODO: on first implementation, allow only one devAddr by socket.
    uint32_t ports[8];
    struct os_eventq sock_eventq;
    struct os_sem sock_sem;
    SLIST_ENTRY(sock_el) sc_next;
    lorawan_socket_type_t socket_type;
};

struct os_eventq *os_eventq_lorawan_api_get(void);

void LoRaMacTestSetDutyCycleOn( bool enable );

#ifdef __cplusplus
}
#endif

#endif /*  __LORAWAN_API_PRIVATE_H__ */
