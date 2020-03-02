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

#include <stdlib.h>
#include "syscfg/syscfg.h"
#include "mcu/mcu.h"
#include "stm32l1xx_hal_gpio_ex.h"
#include "utilities.h"
#include "board-config.h"
#include "board.h"
#include "delay.h"
#include "radio.h"
#include "bsp/bsp.h"

#include "sx1272-board.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1272Init,
    SX1272GetStatus,
    SX1272SetModem,
    SX1272SetChannel,
    SX1272IsChannelFree,
    SX1272Random,
    SX1272SetRxConfig,
    SX1272SetTxConfig,
    SX1272CheckRfFrequency,
    SX1272GetTimeOnAir,
    SX1272Send,
    SX1272SetSleep,
    SX1272SetStby,
    SX1272SetRx,
    SX1272StartCad,
    SX1272SetTxContinuousWave,
    SX1272ReadRssi,
    SX1272Write,
    SX1272Read,
    SX1272WriteBuffer,
    SX1272ReadBuffer,
    SX1272SetMaxPayloadLength,
    SX1272SetPublicNetwork,
    SX1272GetWakeupTime
};

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchTx;
Gpio_t AntSwitchRx;

void SX1272IoInit( void )
{
    // NSS (SS) must be handled here or doesn't work??
    GpioInit( &SX1272.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    // Not neccessary to tell gpio wrapper module about these pins (and in fact will generate an assert!) as the SPI driver does them
//    GpioInit( &SX1272.Spi.Mosi, SPI_0_MASTER_PIN_MOSI, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
//    GpioInit( &SX1272.Spi.Miso, SPI_0_MASTER_PIN_MISO, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
//    GpioInit( &SX1272.Spi.Sclk, SPI_0_MASTER_PIN_SCK, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );

    GpioInit( &SX1272.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1272.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1272.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1272.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1272.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioInit( &SX1272.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
}

void SX1272IoIrqInit( DioIrqHandler **irqHandlers )
{
    GpioSetInterrupt( &SX1272.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0] );
    GpioSetInterrupt( &SX1272.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1] );
    GpioSetInterrupt( &SX1272.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2] );
    GpioSetInterrupt( &SX1272.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3] );
    GpioSetInterrupt( &SX1272.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4] );
    //GpioSetInterrupt( &SX1272.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5] );
}

void SX1272IoDeInit( void )
{
    GpioInit( &SX1272.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    GpioInit( &SX1272.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1272.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1272.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1272.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1272.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1272.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
static void SX1272SetBoardTcxo( uint8_t state )
{
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        DelayMs( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

uint32_t SX1272GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME + MYNEWT_VAL(SX127X_RADIO_WAKEUP_TIME);
}

void SX1272Reset( void )
{
    // Enables the TCXO if available on the board design
    SX1272SetBoardTcxo( true );

    // Set RESET pin to 1
    GpioInit( &SX1272.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    // Wait 1 ms
    DelayMs( 1 );

    // Configure RESET as input
    GpioInit( &SX1272.Reset, RADIO_RESET, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    // Wait 6 ms
    DelayMs( 6 );
}

void SX1272SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1272Read( REG_PACONFIG );
    paDac = SX1272Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1272GetPaSelect( SX1272.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_PACONFIG, paConfig );
    SX1272Write( REG_PADAC, paDac );
}

uint8_t SX1272GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_RFO;
}

void SX1272SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1272SetBoardTcxo( true );
            SX1272AntSwInit( );
        }
        else
        {
            SX1272SetBoardTcxo( false );
            SX1272AntSwDeInit( );
        }
    }
}

// BW : I feel these are a) radio chip independant and b) should be calling the BSP as the antenna switch depends on board layout!
void SX1272AntSwInit( void )
{
    BSP_antSwInit(RADIO_ANT_SWITCH_TX, RADIO_ANT_SWITCH_RX);
    /*
    if (RADIO_ANT_SWITCH_TX!=NC) {
        GpioInit( &AntSwitchTx, RADIO_ANT_SWITCH_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    }
    if (RADIO_ANT_SWITCH_RX!=NC) {
        GpioInit( &AntSwitchRx, RADIO_ANT_SWITCH_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    }
    */
    // else its managed by radio chip
}

void SX1272AntSwDeInit( void )
{
    BSP_antSwDeInit(RADIO_ANT_SWITCH_TX, RADIO_ANT_SWITCH_RX);
/*
    if (RADIO_ANT_SWITCH_TX!=NC) {
        GpioInit( &AntSwitchTx, RADIO_ANT_SWITCH_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
    if (RADIO_ANT_SWITCH_RX!=NC) {
        GpioInit( &AntSwitchRx, RADIO_ANT_SWITCH_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
    */
}

void SX1272SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        BSP_antSwTx(RADIO_ANT_SWITCH_TX, RADIO_ANT_SWITCH_RX);
        /*
        if (RADIO_ANT_SWITCH_TX!=NC) {
            GpioWrite( &AntSwitchTx, 0 );
        }
        if (RADIO_ANT_SWITCH_RX!=NC) {
            GpioWrite( &AntSwitchRx, 1 );
        }
        */
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        BSP_antSwRx(RADIO_ANT_SWITCH_TX, RADIO_ANT_SWITCH_RX);
/*
        if (RADIO_ANT_SWITCH_TX!=NC) {
            GpioWrite( &AntSwitchTx, 1 );       // FOR REVC
        }
        if (RADIO_ANT_SWITCH_RX!=NC) {
            GpioWrite( &AntSwitchRx, 1 );
        }
        */
        break;
    }
}

bool SX1272CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
