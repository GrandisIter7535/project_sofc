/*
 * Fault_chek.c
 *
 *  Created on: 2021. 4. 16.
 *      Author: Plasma Science
 */

#include "UserDefine.h"

void Fault_check_PWM(void)
{
    if(VARI.FLAG.uiRESET){
        VARI.FAULT.all = 0;
        VARI.FLAG.uiRESET = 0;
        VARI.FLAG.uiFAULT = 0;
    }

//    if(PARA.F_W_Level.fI2C < VARI.CNT.uiI2C_CNT){
//        VARI.FAULT.bit.bI2C_fault = 1;
//    }
//    if(){
//        VARI.FAULT.bit.bModbus_fault = 1;
//        VARI.FAULT.bit.bFAULT = 1;
//    }
//
//    if(PARA.F_W_Level.fDCLink_OC < VARI.AVR.fOUT_I_500ms){
//        VARI.FAULT.bit.bDCLink_OC = 1;
//    }
    if(PARA.F_W_Level.fDCLink_OV < VARI.AVR.fDCLink_V_100ms){
        VARI.FAULT.bit.bDCLink_OV = 1;
    }
    if(PARA.F_W_Level.fFC_OC < VARI.FILTER.fFC_I_LPF){
        VARI.FAULT.bit.bFC_OC = 1;
    }

//    if(!GpioDataRegs.GPBDAT.bit.GPIO57){
//        VARI.FAULT.bit.bHW_OC = 1;
//    }

    if(VARI.FAULT.all)
    {
        VARI.FLAG.uiFAULT = 1;
    }
    else
    {
        VARI.FLAG.uiFAULT = 0;
    }
    if((VARI.FAULT.all||VARI.WARN.bit.bFC_UV)||!CAN.Vari.RX.Inv_sequence)
    {
        MB.Word.DCDC_Ready = 0;
    }
    else
    {
        MB.Word.DCDC_Ready = 1;
    }

}

void Fault_check_10ms(void)
{
//    if(PARA.F_W_Level.fTemp_em < VARI.AVR.fTemp_em_10ms){
//        VARI.FAULT.bit.bTemp_em = 1;
//        VARI.FLAG.uiFAULT = 1;
//    }
//    if(PARA.F_W_Level.fTemp_sw < VARI.ADC.fTemp_HS1){
//        VARI.FAULT.bit.bTemp_sw = 1;
//    }
//    if(PARA.F_W_Level.fTemp_sw < VARI.ADC.fTemp_HS2){
//        VARI.FAULT.bit.bTemp_sw = 1;
//    }

    if(CanaRegs.CAN_ES.bit.LEC) VARI.CNT.uiCAN_F_Cnt++;


}

void Fault_check_100ms(void)
{
    if(PARA.F_W_Level.fFC_OV < VARI.AVR.fFC_V_100ms){
        VARI.FAULT.bit.bFC_OV = 1;

    }

    if(PARA.F_W_Level.fFC_UV > VARI.AVR.fFC_V_100ms){
        if(VARI.PROC.ucBST_RUN){
            VARI.FAULT.bit.bFC_UV = 1;
        }
    }
//    if(!VARI.PROC.ucBST_RUN && (VARI.AVR.fFC_V_100ms > 44.)){
//        VARI.FAULT.bit.bFC_UV = 0;
//    }

//    if(PARA.F_W_Level.fClamp_OV < VARI.AVR.fClamp_V_100ms){
//        VARI.FAULT.bit.bClamp_OV = 1;
//        VARI.FLAG.uiFAULT = 1;
//    }
#ifdef DC750V
    if(MB.Word.Heatbeats_return == Heatbeats_return_old)
    {
        Modbus_err_cnt++;
    }
    else
    {
        Modbus_err_cnt = 0;
    }
    if(Modbus_err_cnt > 30)
    {
//        VARI.FAULT.bit.bModbus_fault = 1;

        Modbus_err_cnt = 0;
    }

    Heatbeats_return_old = MB.Word.Heatbeats_return;
#endif
}


void Fault_check_1s(void)
{

    if(PARA.F_W_Level.fFC_UV > VARI.ADC.fFC_V ){
        VARI.WARN.bit.bFC_UV = 1;
    }
    else VARI.WARN.bit.bFC_UV = 0;

    if((PARA.F_W_Level.fDCLink_UV > VARI.ADC.fDCLink_V) && VARI.PROC.ucBST_RUN){
        VARI.WARN.bit.bDCLink_UV = 1;
    }
    else VARI.WARN.bit.bDCLink_UV = 0;

    if((VARI.CNT.uiCAN_F_Cnt > 50)&&((VARI.PROC.ucBST_STATE == 2)||(VARI.PROC.ucBST_STATE == 3))&&(VARI.AVR.fDCLink_V_100ms > 340.)) VARI.WARN.bit.bCAN_fault = 1;
    else VARI.WARN.bit.bCAN_fault = 0;

    VARI.CNT.uiCAN_F_Cnt = 0;

    if(VARI.CNT.uiFANcnt == 0){
        VARI.WARN.bit.bFAN = 1;
    }
    else {
        VARI.WARN.bit.bFAN = 0;
        VARI.CNT.uiFANcnt = 0;
    }

    if(VARI.WARN.all) VARI.FLAG.uiWARNNING = 1;
    else VARI.FLAG.uiWARNNING = 0;

}

interrupt void Fault_isr(void)
{
    PWM_Disable();

    VARI.CTRL.fVC_pterm = 0.;
    VARI.CTRL.fVC_iterm = 0.;
    VARI.CTRL.fCC_pterm = 0.;
    VARI.CTRL.fCC_iterm = 0.;
    VARI.CTRL.fBST_duty = 0.;
    VARI.CTRL.fCC_out = 0;
    VARI.CTRL.fFC_VL = 0;

    EPwm1Regs.CMPA.bit.CMPA = 0.;
    EPwm2Regs.CMPA.bit.CMPA = 0.;

    VARI.PROC.ucRelay_STATE = 0;
    VARI.PROC.ucPrechar_STATE = 0;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;     // Acknowledge this interrupt to receive more interrupts from group 2
}

interrupt void xint1_isr(void)
{


    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
