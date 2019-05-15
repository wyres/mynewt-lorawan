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

#ifndef __CERTIF_TESTING_H__
#define __CERTIF_TESTING_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*certif_entering_callback)(void);
typedef void (*certif_exiting_callback)(void);

#define CERTIF_RX_THREAD_PRIO           MYNEWT_VAL(CERTIF_RX_THREAD_PRIO)
#define CERTIF_RX_THREAD_STACK_SIZE     MYNEWT_VAL(CERTIF_RX_THREAD_STACK_SIZE)
#define CERTIF_TX_THREAD_PRIO           MYNEWT_VAL(CERTIF_TX_THREAD_PRIO)
#define CERTIF_TX_THREAD_STACK_SIZE     MYNEWT_VAL(CERTIF_TX_THREAD_STACK_SIZE)

#define CERTIF_PORT                     224

void certif_init(certif_entering_callback enter_cb, certif_exiting_callback exit_cb);

#ifdef __cplusplus
}
#endif

#endif /* __CERTIF_TESTING_H__ */
