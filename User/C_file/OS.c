/*
 * OS.c
 *
 *  Created on: 2021. 4. 17.
 *      Author: Plasma Science
 */
#include "UserDefine.h"
int asa=0;
void OS_10ms(void)
{
    VARI.CNT.uiADC_isrCnt = 0;
    VARI.CNT.ui10msCnt++;
    //10ms average
    ADC_AVR_10ms();
    //fault check, every 10ms
    Fault_check_10ms();

    EEPROM_Phase();
    GpioDataRegs.GPBSET.bit.GPIO56 = 1;

    Modbus_loop();
    Modbus_Struct_Data();
    CAN_loop();
    Can_Struct_Data();

#ifdef DC750V
    if(!VARI.FAULT.all)
    {
        if(CAN.Vari.RX.Auto_manual == 0xf)
        {
            VARI.C_CMD.fFC_I_max_commu = (float32)CAN.Vari.RX.Set_current*0.1;

            if(CAN.Vari.RX.Start_stop == 170) VARI.PROC.ucBST_COM_RUN = 1;
            else if(CAN.Vari.RX.Start_stop == 85) VARI.PROC.ucBST_COM_RUN = 0;

            Modbus_err_cnt = 0;
        }
        else
        {
            VARI.C_CMD.fFC_I_max_commu = (float32)MB.Word.Set_current*0.1;

            if(MB.Word.ucBST_RUN == 170 && MB.Word.DCDC_Ready == 1) VARI.PROC.ucBST_COM_RUN = 1;
            else if(MB.Word.ucBST_RUN == 85) VARI.PROC.ucBST_COM_RUN = 0;
        }
    }
#ifndef NOT_COMM
    VARI.C_CMD.fFC_I_max = VARI.C_CMD.fFC_I_max_commu;
    if(VARI.C_CMD.fFC_I_max_commu < 0.5){
        VARI.C_CMD.fFC_I_max = 0.5;
    }
#endif

#endif


#ifdef DC360V
    if(!VARI.FAULT.all)
    {
        VARI.C_CMD.fFC_I_max_commu = (float32)MB.Word.Set_current*0.1;

        if(MB.Word.ucBST_RUN == 170) VARI.PROC.ucBST_COM_RUN = 1;
        else if(MB.Word.ucBST_RUN == 85) VARI.PROC.ucBST_COM_RUN = 0;
    }

#ifndef NOT_COMM
    VARI.C_CMD.fFC_I_max = VARI.C_CMD.fFC_I_max_commu;
    if(VARI.C_CMD.fFC_I_max_commu < 2.){
        VARI.C_CMD.fFC_I_max = 2.;
    }
#endif
#endif


    if((VARI.PROC.ucBST_RUN)&&(!Eeprom_wirte_flag))
    {
        Eeprom_wirte_flag = 1;
        VARI.PROC.ucI2C_EEPROM = 2;
    }
    else if((!VARI.PROC.ucBST_RUN)&&(Eeprom_wirte_flag))
    {
        Eeprom_wirte_flag = 0;
        VARI.PROC.ucI2C_EEPROM = 2;
    }


    if(VARI.PROC.ucBST_RUN) GpioDataRegs.GPASET.bit.GPIO5 = 1;
    else GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

}

void OS_100ms(void)
{
    if(MB_Heartbeats++ > 8) MB_Heartbeats = 0;

    VARI.CNT.ui10msCnt = 0;
    VARI.CNT.ui100msCnt++;
    //100ms average
    ADC_AVR_100ms();
    //fault check, every 100ms
    Fault_check_100ms();

}

void OS_500ms(void)
{
    VARI.CNT.ui100msCnt = 0;
    VARI.CNT.ui500msCnt++;
    //500ms average
    ADC_AVR_500ms();

    if(VARI.PROC.ucBST_RUN)     VARI.AVR.fFC_Power_100ms = VARI.AVR.fFC_V_100ms * VARI.AVR.fFC_I_100ms;
    else                        VARI.AVR.fFC_Power_100ms = 0;

#ifdef DC360V
    VARI.AVR.fEfficiency_500ms = __divf32(CAN.Vari.RX.Inv_Power, VARI.AVR.fFC_Power_100ms)*100;
#endif
}

void OS_1s(void)
{
    VARI.CNT.ui500msCnt = 0;

    if(VARI.FLAG.uiFAULT){
//        VARI.CNT.ui1sCnt++;
//        if(VARI.CNT.ui1sCnt > 10){
#ifdef DC750V
        if(MB.Word.Fault_reset||CAN.Vari.RX.Fault_reset)
#endif
#ifdef DC360V
            if(MB.Word.Fault_reset)
#endif
        {
            VARI.FLAG.uiRESET = 1;
//            VARI.CNT.ui1sCnt = 0;
        }
    }
    else VARI.CNT.ui1sCnt = 0;

    //1s average
    ADC_AVR_1s();
    //fault check, every 1s
    Fault_check_1s();
    //Module ID Check
    VARI.PROC.ucModule_ID = 31 - ((GpioDataRegs.GPBDAT.bit.GPIO33) + (GpioDataRegs.GPBDAT.bit.GPIO34 << 1)
                              + (GpioDataRegs.GPBDAT.bit.GPIO39 << 2) + (GpioDataRegs.GPBDAT.bit.GPIO40 << 3)
                              + (GpioDataRegs.GPBDAT.bit.GPIO56 << 4));
    //FAN PWM Duty
    if(VARI.PROC.ucBST_RUN) VARI.CTRL.fFAN_duty = 1.;
    else VARI.CTRL.fFAN_duty = 0.2;
    EPwm3Regs.CMPA.bit.CMPA = (Uint32)(EPwm3Regs.TBPRD * VARI.CTRL.fFAN_duty);

    //Filter Gain
//    IIR2CoeffInit(&VARI.FltDCLink_V2, VARI.FltDCLink_V2.w0, VARI.FltDCLink_V2.zeta);

    //LED ON/OFF
    if(!(VARI.PROC.ucBST_STATE == 99 || VARI.PROC.ucBST_STATE == 0)) GpioDataRegs.GPASET.bit.GPIO16 = 1;
    else GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    if(VARI.FLAG.uiWARNNING) GpioDataRegs.GPASET.bit.GPIO25 = 1;
    else GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
    if(VARI.FLAG.uiFAULT) GpioDataRegs.GPASET.bit.GPIO12 = 1;
    else GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;

#ifdef DC360V
    if(!(VARI.PROC.ucBST_STATE == 99)){
        if(GpioDataRegs.GPADAT.bit.GPIO23 == 1 && VARI.PROC.ucBST_COM_RUN == 1){
//            DC_Relay1_ON;
            if(VARI.PROC.ucRelay_STATE == 1){
                DC_Relay2_ON;
                DC_Relay3_OFF;
                VARI.PROC.ucPrechar_STATE = 1;
            }
            else{
                DC_Relay3_ON;
                DC_Relay2_OFF;

                VARI.PROC.ucRelay_STATE = 1;
            }
        }
        else{
//            DC_Relay1_OFF;
            DC_Relay2_OFF;
            DC_Relay3_OFF;

            VARI.PROC.ucRelay_STATE = 0;
            VARI.PROC.ucPrechar_STATE = 0;
        }
    }

#endif
#ifdef DC750V
    VARI.PROC.ucRelay_STATE = 1;
    VARI.PROC.ucPrechar_STATE = 1;
#endif


}



