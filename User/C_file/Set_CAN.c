/*
 * Set_CAN.c
 *
 *  Created on: 2021. 5. 9.
 *      Author: Plasma Science
 */
#include "UserDefine.h"


void Set_Gpio_CAN_A(void)
{
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
}

void Init_CANA(void)
{
    Can_id = (31 - ((GpioDataRegs.GPBDAT.bit.GPIO33) + (GpioDataRegs.GPBDAT.bit.GPIO34 << 1)
            + (GpioDataRegs.GPBDAT.bit.GPIO39 << 2) + (GpioDataRegs.GPBDAT.bit.GPIO40 << 3)
            + (GpioDataRegs.GPBDAT.bit.GPIO56 << 4)))*0x20;

    CAN_initModule(CANA_BASE);
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 18);  //(uint32_t base, uint32_t clock, uint32_t bitRate, uint16_t bitTime)
                                                                //uint32_t clock is the system clock for the device in Hz.
                                                                //uint32_t bitRate is the desired bit rate.
                                                                //

    // Enable CAN test mode
//    CAN_enableTestMode(CANA_BASE, 100);

    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x101
    //      Message Frame: CAN_MSG_FRAME_STD
    //      Message Type: CAN_MSG_OBJ_TYPE_TX
    //      Message ID Mask: 0
    //      Message Object Flags:
    //      Message Data Length: 8 Bytes
    //

    if(TX_MSG_DATA_LENGTH % 8)      byte8_tx_cnt = 1 + TX_MSG_DATA_LENGTH/8;
    else                            byte8_tx_cnt = TX_MSG_DATA_LENGTH/8;

    for (Can_tx_cnt = 1; Can_tx_cnt <= byte8_tx_cnt; Can_tx_cnt++)
    {
        CAN_setupMessageObject(CANA_BASE, Can_tx_cnt, Can_id + Can_tx_cnt, CAN_MSG_FRAME_STD,CAN_MSG_OBJ_TYPE_TX, 0, 0,8);
    }

   // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x106
    //      Message Frame: CAN_MSG_FRAME_STD
    //      Message Type: CAN_MSG_OBJ_TYPE_RX
    //      Message ID Mask: 0
    //      Message Object Flags: CAN_MSG_OBJ_NO_FLAGS
    //      Message Data Length: 8 Bytes
    //


    if(RX_MSG_DATA_LENGTH % 8)      byte8_rx_cnt = 1 + RX_MSG_DATA_LENGTH/8;
    else                            byte8_rx_cnt = RX_MSG_DATA_LENGTH/8;

    for (Can_rx_cnt = byte8_tx_cnt + 1 ; Can_rx_cnt <= byte8_tx_cnt + byte8_rx_cnt; Can_rx_cnt++)
    {
        CAN_setupMessageObject(CANA_BASE, Can_rx_cnt, Can_id + Can_rx_cnt, CAN_MSG_FRAME_STD,CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_NO_FLAGS,8);
    }

    CAN_startModule(CANA_BASE);
}


