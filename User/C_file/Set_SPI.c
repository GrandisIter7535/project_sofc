/*
 * Set_SPI.c
 *
 *  Created on: 2021. 4. 18.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

void Set_Gpio_SPI_A(void)
{
    EALLOW;

//    //
//    // Enable internal pull-up for the selected pins
//    //
//    // Pull-ups can be enabled or disabled by the user.
//    // This will enable the pullups for the specified pins.
//    // Comment out other unwanted lines.
//    //8 9 11
//    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0; // Enable pull-up on GPIO16 (SPISIMOA)
//    // GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;  // Enable pull-up on GPIO5 (SPISIMOA)
////    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0; // Enable pull-up on GPIO17 (SPISOMIA)
//    // GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;  // Enable pull-up on GPIO3 (SPISOMIA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0; // Enable pull-up on GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0; // Enable pull-up on GPIO19 (SPISTEA)
//
//
//
//
//    //
//    // Set qualification for selected pins to asynch only
//    //
//    // This will select asynch (no qualification) for the selected pins.
//    // Comment out other unwanted lines.
//    //
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO8 = 3; // Asynch input GPIO16 (SPISIMOA)
//    // GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 3;  // Asynch input GPIO5 (SPISIMOA)
////    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
//    // GpioCtrlRegs.GPAQSEL1.bit.GPIO3 = 3;  // Asynch input GPIO3 (SPISOMIA)
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 3; // Asynch input GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3; // Asynch input GPIO19 (SPISTEA)
//
//    //
//    //Configure SPI-A pins using GPIO regs
//    //
//    // This specifies which of the possible GPIO pins will be SPI functional
//    // pins.
//    // Comment out other unwanted lines.
//    //
//    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1; // Configure GPIO16 as SPISIMOA
//    // GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 2;  // Configure GPIO5 as SPISIMOA
////    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
//    // GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2;  // Configure GPIO3 as SPISOMIA
//    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1; // Configure GPIO18 as SPICLKA
//    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // Configure GPIO19 as SPISTEA

    GPIO_setPadConfig(DEVICE_GPIO_PIN_SPISIMOA,GPIO_PIN_TYPE_PULLUP);       //Pull-up
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SPISIMOA);                            //Mux
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SPISIMOA,GPIO_QUAL_ASYNC);    //No synchronization

    GPIO_setPadConfig(DEVICE_GPIO_PIN_SPICLKA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SPICLKA);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SPICLKA,GPIO_QUAL_ASYNC);

//    GPIO_setPadConfig(DEVICE_GPIO_PIN_SPISOMIA,GPIO_PIN_TYPE_STD);
//    GPIO_setPinConfig(DEVICE_GPIO_CFG_SPISOMIA);
//    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SPISOMIA,GPIO_QUAL_ASYNC);

    GPIO_setPadConfig(DEVICE_GPIO_PIN_SPISTEA,GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SPISTEA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SPISTEA,GPIO_DIR_MODE_OUT);       //GPIO Output

    EDIS;
}

void Init_SPIA(void)
{
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    // BISS- RLS
    SpiaRegs.SPICCR.all = 0x001F;

    SpiaRegs.SPICTL.all = 0x0006;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = ((100E6 / 4) / 2500E3) - 1;
    SpiaRegs.SPIFFTX.all = 0xC000;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFCT.all = 2;

    SpiaRegs.SPICCR.bit.SPISWRESET = 0x1;
    SpiaRegs.SPIFFTX.bit.TXFIFO     = 0x1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;


//    SPI_disableModule(SPIA_BASE);
//    //
//    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
//    //
//    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
//                  SPI_MODE_MASTER, 500000, 16);
////    SPI_enableLoopback(SPIA_BASE);
////    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);
//
//    //
//    // FIFO and interrupt configuration
//    //
//    SPI_resetTxFIFO(SPIA_BASE);
//    SPI_resetRxFIFO(SPIA_BASE);
//    SPI_setTxFifoTransmitDelay(SPIA_BASE, 2);
//    SPI_enableFIFO(SPIA_BASE);
//
////    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF | SPI_INT_TXFF);
////    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TX2, SPI_FIFO_RX2);
////    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF | SPI_INT_TXFF);
//
//    //
//    // Configuration complete. Enable the module.
//    //
//    SPI_enableModule(SPIA_BASE);

}


