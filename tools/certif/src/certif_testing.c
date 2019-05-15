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

#include <assert.h>

#include "bsp/bsp.h"
#include "os/os.h"
#include "console/console.h"
#include "hal/hal_system.h"
#include "hal/hal_bsp.h"

#include "lorawan_api/lorawan_api.h"
#include "certif/certif_testing.h"

#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

#define CERTIF_DATA_MAX_SIZE                20
#define CERTIF_NB_FRAMES_STOP_CONDITION    192  // Frames sent without any response from the server

struct os_task certif_rx_thread_task;
static os_stack_t certif_rx_thread_stack[OS_STACK_ALIGN(CERTIF_RX_THREAD_STACK_SIZE)];

struct os_task certif_tx_thread_task;
static os_stack_t certif_tx_thread_stack[OS_STACK_ALIGN(CERTIF_TX_THREAD_STACK_SIZE)];

static lorawan_sock_t sock_rx;
static lorawan_sock_t sock_tx;

static uint8_t app_data[CERTIF_DATA_MAX_SIZE];
static uint8_t app_data_size;

static uint8_t deveui[8];
static uint8_t appeui[8];
static uint8_t appkey[16];

static uint8_t nb_tx_since_last_rx = 0;
static uint16_t downlink_counter = 0;
static bool certif_ongoing = false;

static certif_entering_callback certif_enter_cb = NULL;
static certif_exiting_callback certif_exit_cb = NULL;

void certif_tx_thread(void* data);
void certif_rx_thread(void* data);


extern lorawan_join_t mlme_req_join;

void
certif_prepare_downlink_counter_payload (uint8_t * payload, uint8_t * size)
{
    payload[0] = downlink_counter >> 8;
    payload[1] = downlink_counter;
    *size = 2;
}

int
certif_treat_rx_payload (uint8_t * payload, uint8_t size)
{
    uint32_t id[3];
    uint16_t i;
    os_error_t err;
    lorawan_attribute_t mib;
    lorawan_status_t status;
    lorawan_activation_mode_t mode;

    if ( certif_ongoing == false ) {
        // Check compliance test enable command (i)
        if( ( size == 4 ) &&
            ( payload[0] == 0x01 ) && ( payload[1] == 0x01 ) &&
            ( payload[2] == 0x01 ) && ( payload[3] == 0x01 ) ) {
            downlink_counter = 0;
            certif_ongoing = true;

            console_printf("Start certif\r\n");

            if (certif_enter_cb) {
                certif_enter_cb();
            }

            certif_prepare_downlink_counter_payload(app_data,
                                                    &app_data_size);

            /* Start the TX task */
            err = os_task_init(&certif_tx_thread_task, "certif_tx",
                               certif_tx_thread, NULL, CERTIF_TX_THREAD_PRIO,
                               OS_WAIT_FOREVER, certif_tx_thread_stack,
                               CERTIF_TX_THREAD_STACK_SIZE);
            assert(err == 0);
        }
    } else {
        switch ( payload[0] ) {
            case 0: // Check compliance test disable command (ii)
                downlink_counter = 0;
                certif_ongoing = false;

                console_printf("Stop certif\r\n");
                os_time_delay(100); //Give time to print the trace

                lorawan_close(sock_tx);
                os_task_remove(&certif_tx_thread_task);

                if (certif_exit_cb) {
                    certif_exit_cb();
                }
                break;
            case 1: // (iii, iv)
                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
            case 2: // Enable confirmed messages (v)
                console_printf("Enable confirmed frame\r\n");

                mib.Type = LORAWAN_ATTR_MCPS_TYPE;
                mib.Param.McpsType = LORAWAN_MCPS_CONFIRMED;
                lorawan_setsockopt(sock_tx, &mib);

                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
            case 3:  // Disable confirmed messages (vi)
                console_printf("Enable unconfirmed frame\r\n");

                mib.Type = LORAWAN_ATTR_MCPS_TYPE;
                mib.Param.McpsType = LORAWAN_MCPS_UNCONFIRMED;
                lorawan_setsockopt(sock_tx, &mib);

                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
            case 4: // (vii)
                console_printf("Test crypto\r\n");
                app_data_size = size;
                app_data[0] = 4;
                for (i=1; i<MIN(app_data_size, CERTIF_DATA_MAX_SIZE); i++) {
                    app_data[i] = payload[i] + 1;
                }
                break;
            case 5: // (viii)
#if MYNEWT_VAL(LORAWAN_REGION_EU868)
                console_printf("LinkCheckReq\r\n");

                /* Set the MLME Type (Linkcheck) */
                mib.Type = LORAWAN_ATTR_MLME_TYPE;
                mib.Param.MlmeType = LORAWAN_MLME_LINK_CHECK;
                lorawan_setsockopt(sock_tx, &mib);

                /* Prepare the MAC command for the next TX (not actually sent here) */
                lorawan_send_extended(sock_tx, LORAWAN_FRAME_TYPE_MLME,
                                      CERTIF_PORT, NULL, 0);

                /* Force sending a new message */
                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                lorawan_send(sock_tx, CERTIF_PORT, app_data, app_data_size);
                break;
#elif MYNEWT_VAL(LORAWAN_REGION_US915)
                console_printf("Unsupported command\r\n");
                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
#endif
            case 6: // (ix)
                mode = lorawan_get_current_activation_mode();

                if (mode == LORAWAN_ACTIVATION_MODE_ABP) {
                    /* Generate a DevEUI / AppEUI / AppKey */
                    //TODO make it more generic
                    hal_bsp_hw_id((uint8_t*)id, sizeof(id));
                    deveui[0] = 0x70;
                    deveui[1] = 0x76;
                    deveui[2] = 0xff;
                    deveui[3] = 0x99;
                    deveui[4] = (id[2] >> 24) & 0xFF;
                    deveui[5] = (id[2] >> 16) & 0xFF;
                    deveui[6] = (id[2] >>  8) & 0xFF;
                    deveui[7] = (id[2] >>  0) & 0xFF;

                    memset(appeui, 0, sizeof(appeui));
                    memset(appkey, 0, sizeof(appkey));
                }

                status = lorawan_configure_OTAA(deveui, appeui, appkey, 1, 2,
                        lorawan_get_current_region());
                assert(status == LORAWAN_STATUS_OK);

                /* Enable ADR */
                mib.Type = LORAWAN_ATTR_ADR;
                mib.Param.AdrEnable = true;
                status = lorawan_setsockopt(sock_tx, &mib);
                assert(status == LORAWAN_STATUS_OK);

                /* Disable Dutycycle */
                lorawan_set_dutycycle(false);

                console_printf("Starting LoRaWAN in OTAA mode [with devEUI:");
                for(i=0; i<sizeof(deveui); i++)
                    printf("%02x", deveui[i]);
                printf("]\r\n");

                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
            case 7: // (x)
#if MYNEWT_VAL(LORAWAN_REGION_EU868)
                if (size == 3) {
                    console_printf("TXCW\r\n");

                    uint16_t timeout = (uint16_t)((payload[1] << 8) |
                                                   payload[2]);

                    /* Set the MLME Type (Tx continuous) */
                    mib.Type = LORAWAN_ATTR_MLME_TYPE;
                    mib.Param.MlmeType = LORAWAN_MLME_TXCW;
                    lorawan_setsockopt(sock_tx, &mib);

                    lorawan_send_extended(sock_tx, LORAWAN_FRAME_TYPE_MLME,
                                          CERTIF_PORT, (uint8_t*)&timeout, 2);
                } else if (size == 7) {
                    console_printf("TXCW1\r\n");

                    lorawan_tx_continuous_t tx_cont;
                    tx_cont.timeout = (uint16_t)((payload[1] << 8) |
                                                  payload[2]);
                    tx_cont.frequency = (uint32_t)((payload[3] << 16) |
                                                   (payload[4] << 8) |
                                                    payload[5]) * 100;
                    tx_cont.tx_power = payload[6];

                    /* Set the MLME Type (Tx continuous 1) */
                    mib.Type = LORAWAN_ATTR_MLME_TYPE;
                    mib.Param.MlmeType = LORAWAN_MLME_TXCW_1;
                    lorawan_setsockopt(sock_tx, &mib);

                    lorawan_send_extended(sock_tx, LORAWAN_FRAME_TYPE_MLME,
                                          CERTIF_PORT, (uint8_t*)&tx_cont,
                                          sizeof(tx_cont));
                }
                break;
#elif MYNEWT_VAL(LORAWAN_REGION_US915)
                console_printf("Unsupported command\r\n");
                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
#endif
            default:
                console_printf("Unsupported command\r\n");
                certif_prepare_downlink_counter_payload(app_data,
                                                        &app_data_size);
                break;
        }
    }

    return 0;
}

void
certif_tx_thread(void* data)
{
    lorawan_status_t status;
    lorawan_event_t  lwan_event;
    lorawan_attribute_t mib;

    /* Get a socket */
    sock_tx = lorawan_socket(SOCKET_TYPE_TX);
    assert(sock_tx != 0);

    /* Enable ADR */
    mib.Type = LORAWAN_ATTR_ADR;
    mib.Param.AdrEnable = true;
    status = lorawan_setsockopt(sock_tx, &mib);
    assert(status == LORAWAN_STATUS_OK);

    /* Force the number of retries */
    mib.Type = LORAWAN_ATTR_CHANNELS_NB_REP;
    mib.Param.ChannelNbRep = 2;
    status = lorawan_setsockopt(sock_tx, &mib);
    assert(status == LORAWAN_STATUS_OK);


#if MYNEWT_VAL(LORAWAN_REGION_US915)
    uint16_t channels[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x00FF, 0x0000};

    /* Restore all channels   */
    mib.Type = LORAWAN_ATTR_CHANNELS_MASK;
    mib.Param.ChannelsMask = channels;
    status |= lorawan_setsockopt(sock_tx, &mib);

    mib.Type = LORAWAN_ATTR_CHANNELS_DEFAULT_MASK;
    mib.Param.ChannelsDefaultMask = channels;
    status |= lorawan_setsockopt(sock_tx, &mib);
#endif

    /* Disable Dutycycle */
    lorawan_set_dutycycle(false);

    console_printf("Certif TX thread started\r\n");

    while (1) {
        /* Send a frame every 5 seconds */
        status = lorawan_send(sock_tx, CERTIF_PORT, app_data, app_data_size);
        if (status == LORAWAN_STATUS_OK) {
            /* Wait the frame to be sent */
            lwan_event = lorawan_wait_ev(sock_tx, LORAWAN_EVENT_ERROR | LORAWAN_EVENT_SENT, OS_TIMEOUT_NEVER);

            if (lwan_event == LORAWAN_EVENT_SENT) {
                console_printf("TX certif sent (%d) [with devAddr:%08lx]\r\n",
                               status, lorawan_get_devAddr_unicast());
            } else {
                console_printf("TX certif send error\r\n");
            }
        } else {
            console_printf("TX certif error (%d)\r\n", status);
        }

        if (++nb_tx_since_last_rx >= CERTIF_NB_FRAMES_STOP_CONDITION) {
            hal_system_reset();
        }

        os_time_delay(5000);
    }
}

void
certif_rx_thread(void* data)
{
    lorawan_status_t lorawan_status;
    uint8_t i;
    uint32_t rx_devAddr = 0;
    uint8_t rx_port = 0;

    /* Get a socket */
    sock_rx = lorawan_socket(SOCKET_TYPE_RX);
    assert(sock_rx != 0);

    /* Bind the certification port to the devAddr */
    lorawan_status = lorawan_bind(sock_rx, lorawan_get_devAddr_unicast(), CERTIF_PORT);
    assert(lorawan_status == LORAWAN_STATUS_OK);

    while (1) {
        /* Blocking wait for a message reception */
        app_data_size = lorawan_recv(sock_rx, &rx_devAddr, &rx_port, app_data,
                                     (uint8_t)sizeof(app_data), 0);

        nb_tx_since_last_rx = 0;
        downlink_counter++;

        /* if data are present, print them */
        if(app_data_size != 0){
            os_time_delay(10); // To display correctly the trace
            console_printf("New Rx data (%d): [devAddr:%08lx] [port:%u]\r\n",
                           app_data_size, rx_devAddr, rx_port);
            for(i=0; i<app_data_size; i++)
                console_printf("%02x", app_data[i]);
            console_printf("\r\n");

            if(rx_port == CERTIF_PORT){
                certif_treat_rx_payload(app_data, app_data_size);
            }
        } else {
            certif_prepare_downlink_counter_payload(app_data, &app_data_size);
        }
    }
    assert(0);
}

void
certif_init (certif_entering_callback enter_cb, certif_exiting_callback exit_cb)
{
    os_error_t err;

    /* Register Entering/Exiting callbacks */
    if (enter_cb) {
        certif_enter_cb = enter_cb;
    }

    if (exit_cb) {
        certif_exit_cb = exit_cb;
    }

    /* Start RX Thread */
    err = os_task_init(&certif_rx_thread_task, "certif_rx", certif_rx_thread,
                       NULL, CERTIF_RX_THREAD_PRIO, OS_WAIT_FOREVER,
                       certif_rx_thread_stack, CERTIF_RX_THREAD_STACK_SIZE);
    assert(err == 0);
}

