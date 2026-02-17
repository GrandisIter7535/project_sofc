/*
 * CONTROL_isr.c
 *
 *  Created on: 2021. 5. 26.
 *      Author: Plasma Science
 */

#include "UserDefine.h"

void CONTROL_isr(void)
{
    //18kHz
    VARI.CNT.uiADC_isrCnt++;

    //ADC

    ADC_cal();

    //FAN count check
    if(GpioDataRegs.GPADAT.bit.GPIO22 == VARI.CTRL.uiFAN_z){}
    else VARI.CNT.uiFANcnt++;
    VARI.CTRL.uiFAN_z = GpioDataRegs.GPADAT.bit.GPIO22;

    //fault check
    Fault_check_PWM();

    //////OPEN LOOP PWM Test////////////////////////////////////
    //    PWM_Enable();
    //
    //    EPwm1Regs.CMPA.bit.CMPA = (Uint32)(EPwm1Regs.TBPRD * HC7);
    //    EPwm2Regs.CMPA.bit.CMPA = (Uint32)(EPwm2Regs.TBPRD * HC7);
    //    EPwm1Regs.TBPRD = (Uint32)(SYSCLK / (HC8 * 2));
    //    EPwm2Regs.TBPRD = (Uint32)(SYSCLK / (HC8 * 2));
    ////////////////////////////////////////////////////////////
#ifndef NOT_COMM
#ifdef DC360V
    //Digital Input ON-OFF
    if(GpioDataRegs.GPADAT.bit.GPIO23 == 1 && VARI.PROC.ucPrechar_STATE == 1){
        if(VARI.PROC.ucBST_COM_RUN == 1) VARI.PROC.ucBST_RUN = 1;
        else VARI.PROC.ucBST_RUN = 0;
    }
    else VARI.PROC.ucBST_RUN = 0;


//    VARI.PROC.ucBST_RUN = VARI.PROC.ucBST_COM_RUN;
    //////////////
#endif
#ifdef DC750V
    VARI.PROC.ucBST_RUN = VARI.PROC.ucBST_COM_RUN;

#endif
#endif
    if(VARI.FLAG.uiFAULT){
        PWM_Disable();
        VARI.PROC.ucBST_STATE = STATE_FAULT;
        PieCtrlRegs.PIEIFR2.bit.INTx1 = 1;
    }
    else{
//            PWM_Enable();
    }

    if(!VARI.FLAG.uiFAULT && VARI.PROC.ucBST_RUN && (VARI.PROC.ucBST_STATE == STATE_BST_STOP)){
        if(VARI.PROC.ucBST_STATE == STATE_BST_STOP){
            VARI.PROC.ucBST_STATE = STATE_BST_INIT;
        }

    }
    else if(!VARI.FLAG.uiFAULT && !VARI.PROC.ucBST_RUN){
        VARI.PROC.ucBST_STATE = STATE_BST_STOP;
    }
//    PWM_Enable();//HAE

    switch(VARI.PROC.ucBST_STATE)
    {
        case STATE_BST_STOP: // Stop

            CTRL_BST_Stop();

            break;

        case STATE_BST_INIT: // 변수 초기와

            CTRL_BST_Inint();

            break;

        case STATE_BST_VC: // Voltage Control

            if((!GpioDataRegs.GPADAT.bit.GPIO10)&&(VARI.CTRL.fFC_IL_ref==0.5f)) DC_Relay1_ON;
            CTRL_BST_VC();

        case STATE_BST_CC: // Current Control

            CTRL_BST_CC();

            break;

        case STATE_FAULT:

            PWM_Disable();

            VARI.PROC.ucBST_RUN = 0;


            VARI.CTRL.fBST_duty = 0.;
            EPwm1Regs.CMPA.bit.CMPA = 0.;
            EPwm2Regs.CMPA.bit.CMPA = 0.;

            PieCtrlRegs.PIEIFR2.bit.INTx1 = 1;

#ifdef DC360V
            VARI.PROC.ucRelay_STATE = 0;
            VARI.PROC.ucPrechar_STATE = 0;
#endif
            break;

        default:
            break;

    }

//    VARI.CTRL.fBST_duty = 0.5;


    EPwm1Regs.CMPA.bit.CMPA = (Uint32)((float)EPwm1Regs.TBPRD * VARI.CTRL.fBST_duty);
    EPwm2Regs.CMPA.bit.CMPA = (Uint32)((float)EPwm2Regs.TBPRD * VARI.CTRL.fBST_duty);

//    SerialDacOut();

//        //source time calculated(up-down count)
//        if(EPwm1Regs.TBSTS.bit.CTRDIR == 1) { CCTime = EPwm1Regs.TBCTR + EPwm1Regs.TBPRD;}
//        else                                { CCTime = EPwm1Regs.TBPRD - EPwm1Regs.TBCTR;}
//        if(CCTime_max < CCTime) { CCTime_max = CCTime;}
//        else{}


}
