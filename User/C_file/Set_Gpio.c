/*
 * SetGpio.c
 *
 *  Created on: 2021. 4. 12.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

void Set_Gpio(void)
{
    //GPIO_SetupPinMux(핀번호, CPU 1 or 2, 기능 : DataSheet의 MUX POSITION 참고);
    //GPIO_SetupPinOptions(핀번호, 입력 or 출력, 핀 추가 기능 : 아래 );
    //#define GPIO_PUSHPULL   0
    //#define GPIO_PULLUP       (1 << 0)
    //#define GPIO_INVERT       (1 << 1)
    //#define GPIO_OPENDRAIN    (1 << 2)

    EALLOW;

    GpioCtrlRegs.GPAAMSEL.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAAMSEL.bit.GPIO23 = 0;


    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);//EPWM 1A
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 1);//EPWM 1B
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);//EPWM 2A
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);//EPWM 2B
    GPIO_SetupPinOptions(3, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 1);//EPWM 3A
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);//DO
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);//DC Relay3
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);//DC Relay2
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 7);//SPIA_SIMO
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 7);//SPIA_CLK
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 0);//DC Relay1
    GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 7);//SPIA_STE
    GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);//LED
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);//SCIB_EN
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 2);//SCIB_TX
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 2);//SCIB_RX
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);//LED
    GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 0);//PWM_EN
    GPIO_SetupPinOptions(17, GPIO_OUTPUT, GPIO_PUSHPULL);



    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);//FAN_fault signal_1
    GPIO_SetupPinOptions(22, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(23, GPIO_MUX_CPU1, 0);//DI
    GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_PUSHPULL);



    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);//LED
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 11);//I2CA_SDA
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 11);//I2CA_SCL
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);//SCIA_RX
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);//SCIA_TX
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(30, GPIO_MUX_CPU1, 1);//CANA_RX
    GPIO_SetupPinOptions(30, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 1);//CANA_TX
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);



    GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 0);//ID_Check1
    GPIO_SetupPinOptions(33, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);//ID_Check2
    GPIO_SetupPinOptions(34, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(35, GPIO_MUX_CPU1, 15);//TDI
    GPIO_SetupPinOptions(35, GPIO_INPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(37, GPIO_MUX_CPU1, 15);//TDO
    GPIO_SetupPinOptions(37, GPIO_OUTPUT, GPIO_PULLUP);

    GPIO_SetupPinMux(39, GPIO_MUX_CPU1, 0);//ID_Check4
    GPIO_SetupPinOptions(39, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 0);//ID_Check8
    GPIO_SetupPinOptions(40, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 0);//ID_Check16
    GPIO_SetupPinOptions(56, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 0);//FC_OC_H/W_fault
    GPIO_SetupPinOptions(57, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 0);//Test_pin
    GPIO_SetupPinOptions(58, GPIO_OUTPUT, GPIO_PUSHPULL);

    EDIS;

    GpioDataRegs.GPACLEAR.all = 0x0FFFFFFF;
    // Input Qualification 사용하는 GPIO 샘플링 주기 설정

    GpioCtrlRegs.GPACTRL.bit.QUALPRD1 = 0xFF;   // 510*Tsysclk = 5.1us
    GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 0xFF;   // 510*Tsysclk = 5.1us

}



