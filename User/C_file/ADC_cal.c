/*
 * ADC_cal.c
 *
 *  Created on: 2021. 4. 14.
 *      Author: Plasma Science
 */

#include "UserDefine.h"

Uint16 MOV = 0;
float a=0.;
float x1 = 0., x2 = 0., y1 = 0., y2 = 0.;
float b0 = 0.9930701, b1 = -1.9843985, b2 = 0.9930701, a1 = -1.9843985, a2 = 0.98614021;
float DCLink_V_avr_sum = 0.;
void ADC_cal(void)
{
    //A0//FC_I  //A1//FC_V  //A2//Temp_HS1
    //B0//Clamp_V  //B1//OUT_I  //B2//Temp_HS2
    //C0//DCLink_V  //C1//Temp_em

//    if(VARI.PROC.ucBST_RUN){
//        PARA.ADCGAIN.fOffsetA1 = 23.33;
//    }
//    else{
//        PARA.ADCGAIN.fOffsetA1 = 3.89652;
//    }
    VARI.ADC.fFC_I      = PARA.ADCGAIN.fScaleA0 * ((float32)AdcaResultRegs.ADCRESULT0 - PARA.ADCGAIN.fOffsetA0);
    VARI.ADC.fFC_V      = PARA.ADCGAIN.fScaleA1 * ((float32)AdcaResultRegs.ADCRESULT1 - PARA.ADCGAIN.fOffsetA1);
//    VARI.ADC.fTemp_HS1  = PARA.ADCGAIN.fScaleA2 * ((float32)AdcaResultRegs.ADCRESULT2 - PARA.ADCGAIN.fOffsetA2);

    VARI.ADC.fClamp_V   = PARA.ADCGAIN.fScaleA1 * ((float32)AdcbResultRegs.ADCRESULT0 - PARA.ADCGAIN.fOffsetB0);
//    if(VARI.ADC.fClamp_V<60.)VARI.ADC.fClamp_V=60.;
    VARI.ADC.fOUT_I     = PARA.ADCGAIN.fScaleB1 * ((float32)AdcbResultRegs.ADCRESULT1 - PARA.ADCGAIN.fOffsetB1);
//    VARI.ADC.fTemp_HS2  = PARA.ADCGAIN.fScaleB2 * ((float32)AdcbResultRegs.ADCRESULT2 - PARA.ADCGAIN.fOffsetB2);

    VARI.ADC.fDCLink_V  = PARA.ADCGAIN.fScaleC0 * ((float32)AdccResultRegs.ADCRESULT0 - PARA.ADCGAIN.fOffsetC0);
//    VARI.ADC.fTemp_em   = PARA.ADCGAIN.fScaleC1 * ((float32)AdccResultRegs.ADCRESULT1 - PARA.ADCGAIN.fOffsetC1);
    VARI.ADC.fTemp_em = -1481.96 + __sqrt(((1.8639 - (((float32)AdccResultRegs.ADCRESULT1- PARA.ADCGAIN.fOffsetC1) * 8.058608058608059e-4)) * 257731.9587628866) + 2196200);

//    if(VARI.ADC.fFC_V < 9.) VARI.ADC.fFC_V = 9.;

    //////Temp calculate
    VARI.ADC.fTemp_HS1_ = ((float32)AdcaResultRegs.ADCRESULT2 - 1675);

    if(VARI.ADC.fTemp_HS1_ < 414){
        VARI.ADC.fTemp_HS1 = -0.12552  * (VARI.ADC.fTemp_HS1_ - 1249.20619);
    }
    else if(414 <= VARI.ADC.fTemp_HS1_ < 669){
        VARI.ADC.fTemp_HS1 = -0.07430 * (VARI.ADC.fTemp_HS1_ - 1811.62744);
    }
    else if(669 <= VARI.ADC.fTemp_HS1_ < 1100){
        VARI.ADC.fTemp_HS1 = -0.04421 * (VARI.ADC.fTemp_HS1_ - 2567.38514);
    }
    else if(1100 <= VARI.ADC.fTemp_HS1_ < 1803){
        VARI.ADC.fTemp_HS1 = -0.02709 * (VARI.ADC.fTemp_HS1_ - 3461.36838);
    }
    else if(1803 <= VARI.ADC.fTemp_HS1_ < 2820){
        VARI.ADC.fTemp_HS1 = -0.01902 * (VARI.ADC.fTemp_HS1_ - 4133.42657);
    }
    else if(2820 <= VARI.ADC.fTemp_HS1_){
        VARI.ADC.fTemp_HS1 = -0.01659 * (VARI.ADC.fTemp_HS1_ - 4319.38454);
    }

    VARI.FILTER.fTemp_HS1 = 0.705353943 * VARI.FILTER.fTemp_HS1_old + (1 - 0.705353943) * (VARI.ADC.fTemp_HS1);//1kHz
    VARI.FILTER.fTemp_HS1_old = (VARI.ADC.fTemp_HS1);

    if(VARI.FILTER.fTemp_HS1 > 150.) VARI.FILTER.fTemp_HS1 = 150.;
    else if(VARI.FILTER.fTemp_HS1 < -20.) VARI.FILTER.fTemp_HS1 = -20.;

    //////Temp calculate
    VARI.ADC.fTemp_HS2_ = ((float32)AdcbResultRegs.ADCRESULT2 - (-500));

    if(VARI.ADC.fTemp_HS2_ < 414){
        VARI.ADC.fTemp_HS2 = -0.12552  * (VARI.ADC.fTemp_HS2_ - 1249.20619);
    }
    else if(414 <= VARI.ADC.fTemp_HS2_ < 669){
        VARI.ADC.fTemp_HS2 = -0.07430 * (VARI.ADC.fTemp_HS2_ - 1811.62744);
    }
    else if(669 <= VARI.ADC.fTemp_HS2_ < 1100){
        VARI.ADC.fTemp_HS2 = -0.04421 * (VARI.ADC.fTemp_HS2_ - 2567.38514);
    }
    else if(1100 <= VARI.ADC.fTemp_HS2_ < 1803){
        VARI.ADC.fTemp_HS2 = -0.02709 * (VARI.ADC.fTemp_HS2_ - 3461.36838);
    }
    else if(1803 <= VARI.ADC.fTemp_HS2_ < 2820){
        VARI.ADC.fTemp_HS2 = -0.01902 * (VARI.ADC.fTemp_HS2_ - 4133.42657);
    }
    else if(2820 <= VARI.ADC.fTemp_HS2_){
        VARI.ADC.fTemp_HS2 = -0.01659 * (VARI.ADC.fTemp_HS2_ - 4319.38454);
    }

    VARI.FILTER.fTemp_HS2 = 0.705353943 * VARI.FILTER.fTemp_HS2_old + (1 - 0.705353943) * (VARI.ADC.fTemp_HS2);//1kHz
    VARI.FILTER.fTemp_HS2_old = (VARI.ADC.fTemp_HS2);

    if(VARI.FILTER.fTemp_HS2 > 150.) VARI.FILTER.fTemp_HS2 = 150.;
    else if(VARI.FILTER.fTemp_HS2 < -20.) VARI.FILTER.fTemp_HS2 = -20.;


////////////Average SUM
    VARI.AVR.fDCLink_V_10ms_sum += VARI.ADC.fDCLink_V;
    VARI.AVR.fFC_I_10ms_sum += VARI.ADC.fFC_I;
    VARI.AVR.fTemp_em_10ms_sum += VARI.ADC.fTemp_em;
    VARI.AVR.fTemp_HS2_10ms_sum += VARI.ADC.fTemp_HS2;
    VARI.AVR.fTemp_HS1_10ms_sum += VARI.ADC.fTemp_HS1;
    VARI.CNT.uiSUM10msCNT++;

//////////Moving Average
//    if(VARI.CNT.ui120HzCNT > 149) VARI.CNT.ui120HzCNT = 0;
//    MOV = VARI.CNT.ui120HzCNT + 1;
//    if(MOV > 149) MOV = 0;
//
//    VARI.AVR.DCLink_V_avr_buf[VARI.CNT.ui120HzCNT] = VARI.ADC.fDCLink_V;
//    VARI.AVR.fDCLink_V_mov += (VARI.ADC.fDCLink_V - VARI.AVR.DCLink_V_avr_buf[MOV])*0.00666666666666666666666666666667;

    VARI.AVR.Clamp_V_avr_buf[VARI.CNT.ui120HzCNT] = VARI.ADC.fClamp_V;
    VARI.AVR.fClamp_V_mov += (VARI.ADC.fClamp_V - VARI.AVR.Clamp_V_avr_buf[MOV])*0.00666666666666666666666666666667;



    DCLink_V_avr_sum -= VARI.AVR.DCLink_V_avr_buf[VARI.CNT.ui120HzCNT];
    DCLink_V_avr_sum += VARI.ADC.fDCLink_V;
    VARI.AVR.DCLink_V_avr_buf[VARI.CNT.ui120HzCNT] = VARI.ADC.fDCLink_V;

    VARI.CNT.ui120HzCNT++;
    VARI.CNT.ui120HzCNT %= 150;

    VARI.AVR.fDCLink_V_mov = DCLink_V_avr_sum*0.0066666;




//    VARI.CNT.ui120HzCNT++;

    VARI.AVR.fOUT_I_100ms_sum += VARI.ADC.fOUT_I;
    VARI.AVR.fFC_V_sum += VARI.ADC.fFC_V;
    VARI.AVR.fFC_I_100ms_sum += VARI.ADC.fFC_I;
    VARI.AVR.fClamp_V_sum += VARI.ADC.fClamp_V;
    VARI.AVR.fDCLink_V_100ms_sum += VARI.ADC.fDCLink_V;
    VARI.CNT.uiSUM100msCNT++;


    ///filter

//    VARI.FILTER.fClamp_V_LPF = 0.350930645 * VARI.FILTER.fClamp_V_old + (1-0.350930645) * VARI.ADC.fClamp_V;
//    VARI.FILTER.fClamp_V_old = VARI.ADC.fClamp_V;
//    VARI.FILTER.fFC_V_LPF = 0.350930645 * VARI.FILTER.fFC_V_old + (1-0.350930645) * VARI.ADC.fFC_V;
//    VARI.FILTER.fFC_V_old = VARI.ADC.fFC_V;

//    VARI.FILTER.fDCLink_V_LPF = IIR2Update(&VARI.FltDCLink_V, VARI.ADC.fDCLink_V);


    VARI.FILTER.fDCLink_V_LPF = IIR2Update(&VARI.FltDCLink_V, VARI.ADC.fDCLink_V);
#ifdef DC750V
//    VARI.FILTER.fDCLink_V_LPF2 = IIR2Update(&VARI.FltDCLink_V2, VARI.ADC.fDCLink_V);
    VARI.FILTER.fDCLink_V_LPF2 = IIR2Update(&VARI.FltDCLink_V2, VARI.ADC.fDCLink_V);

#endif
#ifdef DC360V
    VARI.FILTER.fDCLink_V_LPF2 = IIR2Update(&VARI.FltDCLink_V2, VARI.ADC.fDCLink_V);

//    VARI.FILTER.fDCLink_V_LPF2 = VARI.ADC.fDCLink_V * (1-0.250930645) + VARI.FILTER.fDCLink_V_old * 0.250930645;
//    VARI.FILTER.fDCLink_V_old = VARI.ADC.fDCLink_V;


#endif

    VARI.FILTER.fClamp_V_LPF = IIR2Update(&VARI.FltfClamp_V, VARI.ADC.fClamp_V);

    VARI.FILTER.fFC_I_LPF = IIR2Update(&VARI.FltFC_I, VARI.ADC.fFC_I);
    VARI.FILTER.fFC_I_LPF2 = IIR2Update(&VARI.FltFC_I2, VARI.ADC.fFC_I);
    VARI.FILTER.fI_out_LPF = IIR2Update(&VARI.FltI_out, VARI.ADC.fOUT_I);

//    VARI.FILTER.fDCLink_V_LPF2 = (b0 * VARI.ADC.fDCLink_V + (b1) * x1 + (b2) * x2 - (a1) * y1 - (a2) * y2);
//    x2 = x1;
//    x1 = VARI.ADC.fDCLink_V;
//    y2 = y1;
//    y1 = VARI.FILTER.fDCLink_V_LPF2;



}

void ADC_AVR_10ms(void)
{
    //AVR 10ms
    VARI.AVR.f10msCnt_inv = __divf32(1., (float)VARI.CNT.uiSUM10msCNT);
    VARI.CNT.uiSUM10msCNT = 0;

    VARI.AVR.fDCLink_V_10ms = VARI.AVR.fDCLink_V_10ms_sum * VARI.AVR.f10msCnt_inv;
    VARI.AVR.fDCLink_V_10ms_sum = 0.;

    VARI.AVR.fTemp_em_10ms = VARI.AVR.fTemp_em_10ms_sum * VARI.AVR.f10msCnt_inv;
    VARI.AVR.fTemp_em_10ms_sum = 0.;

    VARI.AVR.fTemp_HS1_10ms = VARI.AVR.fTemp_HS1_10ms_sum * VARI.AVR.f10msCnt_inv;
    VARI.AVR.fTemp_HS1_10ms_sum = 0.;

    VARI.AVR.fTemp_HS2_10ms = VARI.AVR.fTemp_HS2_10ms_sum * VARI.AVR.f10msCnt_inv;
    VARI.AVR.fTemp_HS2_10ms_sum = 0.;

    VARI.AVR.fFC_I_10ms = VARI.AVR.fFC_I_10ms_sum * VARI.AVR.f10msCnt_inv;
    VARI.AVR.fFC_I_10ms_sum = 0.;


}

void ADC_AVR_100ms(void)
{
    //AVR 100ms
    VARI.AVR.f100msCnt_inv = __divf32(1., (float)VARI.CNT.uiSUM100msCNT);
    VARI.CNT.uiSUM100msCNT = 0;

    VARI.AVR.fClamp_V_100ms = VARI.AVR.fClamp_V_sum * VARI.AVR.f100msCnt_inv;
    VARI.AVR.fClamp_V_sum = 0.;

    VARI.AVR.fFC_V_100ms = VARI.AVR.fFC_V_sum * VARI.AVR.f100msCnt_inv;
    VARI.AVR.fFC_V_sum = 0.;


    VARI.AVR.fDCLink_V_100ms = VARI.AVR.fDCLink_V_100ms_sum * VARI.AVR.f100msCnt_inv;
    VARI.AVR.fDCLink_V_100ms_sum = 0.;

    VARI.AVR.fFC_I_100ms = VARI.AVR.fFC_I_100ms_sum * VARI.AVR.f100msCnt_inv;
    VARI.AVR.fFC_I_100ms_sum = 0.;

    VARI.AVR.fOUT_I_100ms = VARI.AVR.fOUT_I_100ms_sum * VARI.AVR.f100msCnt_inv;
    VARI.AVR.fOUT_I_100ms_sum = 0.;


}

void ADC_AVR_500ms(void)
{
    //AVR 500ms

    VARI.CNT.uiSUM500msCNT = 0;
}

void ADC_AVR_1s(void)
{

    VARI.CNT.uiSUM1sCNT = 0;
}
