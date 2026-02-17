/*###########################################################################

 FILE:   main.c

 This example demonstrates how to make use of the easyDSP with BitField
 Target MCU : F28002x, F28004x, F2807x, F2837xS, F2837xD, F2838xS, F2838xD

 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 1st. programmed by Hae-Chan Park, Apr 2021.

 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

###########################################################################*/
// Included Files
//#include "F28x_Project.h"
#include "easy28x_bitfield_v9.5.h"
#include "UserDefine.h"


//#define USE_DEBUGGER          // uncomment this if you use debugger for multicore cpu


//
// Main
//

void main(void)
{
    //
    //Step 1. Initialize System Control:
    //PLL, WatchDog,enable Peripheral Clocks
    //This example function is found in the f28004x_sysctrl.c file.
    //

    InitSysCtrl();

    //Variable value RESET
    memset_fast(&VARI, 0, sizeof(VARI));
    memset_fast(&PARA, 0, sizeof(PARA));
    memset_fast(&MB,   0, sizeof(MB));
    memset_fast(&CAN,  0, sizeof(CAN));
    //
    //Step 2. Initialize GPIO

    //
    InitGpio();

    //I/O setting
    Set_Gpio();
    Set_Gpio_SPI_A();
    Set_Gpio_SCI_B();
    Set_Gpio_I2C_A();
    Set_Gpio_CAN_A();


    PWM_Disable();

    //
    //Step 3. Clear all __interrupts and initialize PIE vector table:
    //Disable CPU __interrupts
    //
    DINT;

    //
    //Initialize the PIE control registers to their default state.
    //The default state is all PIE interrupts disabled and flags
    //are cleared.
    //


    //
    //Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    EALLOW;  // This is needed to write to EALLOW protected registers

    PieVectTable.ADCA1_INT      = &adca1_isr;
    PieVectTable.XINT1_INT      = &xint1_isr;
    PieVectTable.EPWM1_TZ_INT   = &Fault_isr;
    PieVectTable.I2CA_INT       = &i2c_isr;
    PieVectTable.I2CA_FIFO_INT  = &i2cFIFO_isr;
    PieVectTable.SCIB_RX_INT    = &scibRxFifoIsr;

    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    Init_EPwm();
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    Init_ADC(0,9);             // ADC A,B,C 초기화

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      // PIE 그룹 1 인터럽트(ADCA1) 활성화
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;      // PIE 그룹 1 인터럽트(XINT1) 활성화
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;      // PIE 그룹 2 인터럽트(TZ1) 활성화
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;      // PIE 그룹 8 인터럽트(I2CA) 활성화
    PieCtrlRegs.PIEIER8.bit.INTx2 = 1;      // PIE 그룹 8 인터럽트(I2CA_FIFO) 활성화
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // PIE 그룹 9 인터럽트(SCIB_RX) 활성화

    IER |= M_INT1;                          // PIE 그룹 1 인터럽트(INT1.x)  활성화 ADCA1, XINT1
    IER |= M_INT2;                          // PIE 그룹 2 인터럽트(INT2.x)  활성화 EPWM1_TZ
    IER |= M_INT8;                          // PIE 그룹 8 인터럽트(INT8.x)  활성화 I2CA, I2CA_FIFO
    IER |= M_INT9;                          // PIE 그룹 9 인터럽트(INT9.x)  활성화 SCIA_RX, SCIB_RX

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    //Step 4. User specific code:
    //
    Init_SPIA();            //SPI_A - DAC_B/D
    easyDSP_SCI_Init();     //SCI_A - easyDSP
    Init_SCIB();            //SCI_B - M_BOP
    Init_I2CA();            //I2C_A - EEPROM
    Init_CANA();            //CAN_A - Communication <-> ARM INV

    firm_Init();
    InitSerialDac();

    for(;;)
    {
        //10ms Operate
        if(VARI.CNT.uiADC_isrCnt >= (CNT_10ms)){
            OS_10ms();
        }

        //100ms Operate
        if(VARI.CNT.ui10msCnt >= 10){
            OS_100ms();
        }

        //500ms Operate
        if(VARI.CNT.ui100msCnt >= 5){
            OS_500ms();
        }

        //1s Operate
        if(VARI.CNT.ui500msCnt >= 2){
            OS_1s();

        }
    }
}



//
// End of file
//
