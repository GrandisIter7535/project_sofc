/*
 * firm_Init.c
 *
 *  Created on: 2021. 4. 15.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

 void firm_Init(void)
 {
          ///(&Variable,             Filter,     Cut-off Freq,   Zeta,       Ts)
#ifdef DC750V
     IIR2Init(&VARI.FltDCLink_V2,    K_LPF,      PI2 * 1000.,    0.707,      TC);
#endif
#ifdef DC360V
     IIR2Init(&VARI.FltDCLink_V2,    K_NOTCH,      PI2 * 120.,    0.083330,      TC);
#endif

     IIR2Init(&VARI.FltDCLink_V,     K_LPF,      PI2 * 3000.,    0.707,      TC);
     IIR2Init(&VARI.FltI_out,        K_LPF,      PI2 * 3000.,    0.707,      TC);
     IIR2Init(&VARI.FltFC_I,         K_LPF,      PI2 * 3000.,    0.707,      TC);//18000
     IIR2Init(&VARI.FltFC_I2,        K_NOTCH,    PI2 * 120.,     0.083330,   TC);
     IIR2Init(&VARI.FltfVC_out,      K_NOTCH,    PI2 * 120.,     0.0083330,  TC);
     IIR2Init(&VARI.FltfClamp_V,     K_LPF,      PI2 * 3000.,    0.707,      TC);

     IIR2CoeffInit(&VARI.FltDCLink_V2, VARI.FltDCLink_V2.w0, VARI.FltDCLink_V2.zeta);
     IIR2CoeffInit(&VARI.FltDCLink_V, VARI.FltDCLink_V.w0, VARI.FltDCLink_V.zeta);
     IIR2CoeffInit(&VARI.FltI_out, VARI.FltI_out.w0, VARI.FltI_out.zeta);
     IIR2CoeffInit(&VARI.FltFC_I, VARI.FltFC_I.w0, VARI.FltFC_I.zeta);
     IIR2CoeffInit(&VARI.FltFC_I2, VARI.FltFC_I2.w0, VARI.FltFC_I2.zeta);
     IIR2CoeffInit(&VARI.FltfVC_out, VARI.FltfVC_out.w0, VARI.FltfVC_out.zeta);
     IIR2CoeffInit(&VARI.FltfClamp_V, VARI.FltfClamp_V.w0, VARI.FltfClamp_V.zeta);

     VARI.AVR.fDCLink_V_mov = VARI.ADC.fDCLink_V;
     VARI.AVR.fClamp_V_mov = VARI.ADC.fClamp_V;

     int i=0;
     for(i=0; i<150; i++){
         VARI.AVR.DCLink_V_avr_buf[i] = VARI.ADC.fDCLink_V;
     }

     //PWM duty = 0, PWM disable
     EPwm1Regs.CMPA.bit.CMPA = 0;
     EPwm2Regs.CMPA.bit.CMPA = 0;
     PWM_OFF;
     PWM_Disable();

     //10ms Count value
     CNT_10ms = 180;

     VARI.CTRL.fFAN_duty = 0.2;

     VARI.CTRL.fdDdt = 0.015;
     VARI.C_CMD.fBST_Duty = 0.8;
#ifdef DC750V
     VARI.CTRL.fdVdt = 200.;
     VARI.CTRL.fOften_ref = 300.;
#endif
#ifdef DC360V
     VARI.CTRL.fdVdt = 100.;
     VARI.CTRL.fOften_ref = 200.;
#endif
     VARI.CTRL.fdIdt = 5.;

     //Control Gain
#ifdef DC360V
     VARI.CTRL.fVC_kp = 0.017;
     VARI.CTRL.fVC_ki = 0.89;
     VARI.CTRL.fVC_ka = __divf32(1., VARI.CTRL.fVC_kp);
     VARI.CTRL.fVC_kip = 0.;

     VARI.CTRL.fCC_kp = 0.2198392093;
     VARI.CTRL.fCC_ki = 0.004737408087;
     VARI.CTRL.fCC_ka = __divf32(1., VARI.CTRL.fCC_kp);
     VARI.CTRL.fCC_kip = 1.;
#endif

#ifdef DC750V
     VARI.CTRL.fVC_kp = 0.01700000139;
     VARI.CTRL.fVC_ki = 3.94784e-07;
     VARI.CTRL.fVC_ka = __divf32(1., VARI.CTRL.fVC_kp);
     VARI.CTRL.fVC_kip = 0.7;

     VARI.CTRL.fCC_kp = 0.2198392093;
     VARI.CTRL.fCC_ki = 0.004737408087;
     VARI.CTRL.fCC_ka = __divf32(1., VARI.CTRL.fCC_kp);
     VARI.CTRL.fCC_kip = 0.;
#endif



     //Control value reset
#ifdef DC750V
    #ifdef BST_VC
     VARI.C_CMD.fBST_V = 550.;//750.;
     VARI.CTRL.fBST_V_ref = VARI.ADC.fDCLink_V;
     VARI.CTRL.fFC_IL_ref = 0.;
    #endif
    #ifdef BST_CC
     VARI.C_CMD.fBST_V = 100.;
     VARI.CTRL.fBST_V_ref = 0.;
     VARI.CTRL.fFC_IL_ref = VARI.ADC.fFC_I;
    #endif
#endif

#ifdef DC360V
    #ifdef BST_VC
     VARI.C_CMD.fBST_V = 380.;
     VARI.CTRL.fBST_V_ref = VARI.ADC.fDCLink_V;
     VARI.CTRL.fFC_IL_ref = 0.;
    #endif
    #ifdef BST_CC
     VARI.C_CMD.fBST_V = 50.;
     VARI.CTRL.fBST_V_ref = 0.;
     VARI.CTRL.fFC_IL_ref = VARI.ADC.fFC_I;
    #endif
#endif

     VARI.C_CMD.fFC_I_max = 0.5;
     PARA.LIMIT.fFC_IL_max = 0.;

     VARI.CTRL.fVC_pterm = 0.;
     VARI.CTRL.fVC_iterm = 0.;
     VARI.CTRL.fVC_out = 0;
     VARI.CTRL.fCC_pterm = 0.;
     VARI.CTRL.fCC_iterm = 0.;
     VARI.CTRL.fCC_out = 0.;
     VARI.CTRL.fFC_VL = 0.;

     VARI.CTRL.fBST_duty = 0.;

     //I2C
     VARI.PROC.ucI2C_EEPROM = 1;

///////////////**********EEPROM***********//////////////////
     //A0//FC_I  //A1//FC_V  //A2//Temp_HS1
     //B0//Clamp_V  //B1//OUT_I  //B2//Temp_HS2
     //C0//DCLink_V  //C1//Temp_em

     PARA.ADCGAIN.fScaleA0 = 0.025391;//0.02526;
     PARA.ADCGAIN.fScaleA1 = 0.022626;//0.02430;
     PARA.ADCGAIN.fScaleA2 = 0.;
     PARA.ADCGAIN.fScaleB0 = 0.036714;
     PARA.ADCGAIN.fScaleB1 = 0.005389;
     PARA.ADCGAIN.fScaleB2 = 0.;
     PARA.ADCGAIN.fScaleC0 = 0.218942;
//     PARA.ADCGAIN.fScaleC1 = 0.;

     PARA.ADCGAIN.fOffsetA0 = 507.6675;//500.;
     PARA.ADCGAIN.fOffsetA1 = -0.10791;//13.30594;
     PARA.ADCGAIN.fOffsetA2 = 0.;
//     PARA.ADCGAIN.fOffsetB0 = 5.72487;
     PARA.ADCGAIN.fOffsetB0 = -1.50153;
     PARA.ADCGAIN.fOffsetB1 = 474.1016;
     PARA.ADCGAIN.fOffsetB2 = 0.;
     PARA.ADCGAIN.fOffsetC0 = -0.29181;
     PARA.ADCGAIN.fOffsetC1 = 150.;

     PARA.F_W_Level.fTemp_em = 60.;
//     PARA.F_W_Level.fTemp_sw = 80.;
     PARA.F_W_Level.fClamp_OV = 200.;
     PARA.F_W_Level.fDCLink_OC = 5.;

//     PARA.F_W_Level.uiCAN = 10;
     PARA.F_W_Level.fI2C = 3;

#ifdef DC750V
     PARA.F_W_Level.fFC_OV = 80.;//60.;
     PARA.F_W_Level.fFC_UV = 40.;
     PARA.F_W_Level.fFC_OC = 45.;
     PARA.F_W_Level.fDCLink_OV = 850.;
     PARA.F_W_Level.fDCLink_UV = 550.;
#endif
#ifdef DC360V
     PARA.F_W_Level.fFC_OV = 90.;
     PARA.F_W_Level.fFC_UV = 40.;

     PARA.F_W_Level.fFC_OC = 45.;
     PARA.F_W_Level.fDCLink_OV = 480.;
     PARA.F_W_Level.fDCLink_UV = 300.;
#endif

     PARA.F_W_Level.fFAN = 30;

     PARA.LIMIT.fFC_IL_max = 2.;//60.;
     PARA.LIMIT.fFC_IL_min = -1.0;

//     MB.Word.Set_power = 2000;
     DC_Relay1_OFF;
     DC_Relay2_OFF;

//     MB.Word.Set_ramp = 100;
 }


