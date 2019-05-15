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

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

//#include "bsp/bsp_defs.h"

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0


/* Radio pin definitions */
#if MYNEWT_VAL(SX1272)
#define BSP_RADIO_DIO_0_PIN         MYNEWT_VAL(SX1272_DIO_0)
#define BSP_RADIO_DIO_1_PIN         MYNEWT_VAL(SX1272_DIO_1)
#define BSP_RADIO_DIO_2_PIN         MYNEWT_VAL(SX1272_DIO_2)
#define BSP_RADIO_DIO_3_PIN         MYNEWT_VAL(SX1272_DIO_3)
#define BSP_RADIO_DIO_4_PIN         MYNEWT_VAL(SX1272_DIO_4)
#define BSP_RADIO_DIO_5_PIN         MYNEWT_VAL(SX1272_DIO_5)

#define BSP_RADIO_NSS_PIN           MYNEWT_VAL(SX1272_NSS)
#define BSP_RADIO_RESET_PIN         MYNEWT_VAL(SX1272_RESET)

#define BSP_RADIO_DEVICE_SEL_PIN    (-1)
#define BSP_RADIO_BUSY_PIN          (-1)

#elif MYNEWT_VAL(SX1276)
#error "SX1276 is not managed yet"

#elif MYNEWT_VAL(SX1261) || MYNEWT_VAL(SX1262)
#define BSP_RADIO_DEVICE_SEL_PIN    MYNEWT_VAL(SX126X_RADIO_DEVICE_SEL)//not usefull
#define BSP_RADIO_NSS_PIN           MYNEWT_VAL(SX126X_NSS)
#define BSP_RADIO_RESET_PIN         MYNEWT_VAL(SX126X_RESET)
#define BSP_RADIO_BUSY_PIN          MYNEWT_VAL(SX126X_RADIO_BUSY)

#define BSP_RADIO_DIO_0_PIN         (-1)
#define BSP_RADIO_DIO_1_PIN         MYNEWT_VAL(SX126X_DIO_1)
#define BSP_RADIO_DIO_2_PIN         (-1)
#define BSP_RADIO_DIO_3_PIN         (-1)
#define BSP_RADIO_DIO_4_PIN         (-1)
#define BSP_RADIO_DIO_5_PIN         (-1)

#else
#error "Please define one radio (SX1272,SX1276,SX1261,SX1262)"
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 BSP_RADIO_RESET_PIN

#define RADIO_NSS                                   BSP_RADIO_NSS_PIN

#define RADIO_DIO_0                                 BSP_RADIO_DIO_0_PIN
#define RADIO_DIO_1                                 BSP_RADIO_DIO_1_PIN
#define RADIO_DIO_2                                 BSP_RADIO_DIO_2_PIN
#define RADIO_DIO_3                                 BSP_RADIO_DIO_3_PIN
#define RADIO_DIO_4                                 BSP_RADIO_DIO_4_PIN
#define RADIO_DIO_5                                 BSP_RADIO_DIO_5_PIN

// independant of the radio type
#define RADIO_ANT_SWITCH_TX                         MYNEWT_VAL(RADIO_ANT_SWITCH_TX)
#define RADIO_ANT_SWITCH_RX                         MYNEWT_VAL(RADIO_ANT_SWITCH_RX)

/* Sx126x specific */
#define RADIO_BUSY                                  BSP_RADIO_BUSY_PIN

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#endif // __BOARD_CONFIG_H__
