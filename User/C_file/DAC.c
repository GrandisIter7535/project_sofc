/*
 * DAC.c
 *
 *  Created on: 2021. 4. 16.
 *      Author: Plasma Science
 */

#include "UserDefine.h"


int* da[DAC_CH] = {0, 0, 0, 0};
int da_type[DAC_CH] = {0, 0, 0, 0};
float da_scale[DAC_CH] = {0, 0, 0, 0};
float da_mid[DAC_CH] = {0, 0, 0, 0};
int DACdata[DAC_CH] = {0, 0, 0, 0};

void InitSerialDac(void)
{
    da[0]=(int*)&DJ1;
    da_type[0]=0;   da_scale[0]=2048./200.;  da_mid[0]=0.;

    da[1]=(int*)&VARI.ADC.fDCLink_V;
    da_type[1]=0;   da_scale[1]=2048./200.;  da_mid[1]=0.;

    da[2]=(int*)&VARI.ADC.fFC_V;
    da_type[2]=0;   da_scale[2]=2048./500;  da_mid[2]=0.;

    da[3]=(int*)&VARI.ADC.fFC_V;
    da_type[3]=0;   da_scale[3]=2048./1000;     da_mid[3]=0.;

}

unsigned int dactemp1, dactemp2, dactemp3, dactemp4;
int test_data;

void SerialDacOut(void)
{


    int data;
    float temp=0.0;

//    DA_0 = VARI.ADC.fFC_I;
//    da[0]=(int*)&DA_0;
//    DA_1 = VARI.FILTER.fFC_I_LPF2;
//    da[1]=(int*)&DA_1;

        temp = da_type[0] ? (float)(*da[0]) : *(float *)da[0];
        data = (int)((temp - da_mid[0])*da_scale[0])+0x800;
        test_data = data;
        SpiaRegs.SPITXBUF   =  0x1000 | 0x0000 |(data&0xFFF);
        dactemp1 = SpiaRegs.SPIRXBUF;

        temp = da_type[1] ? (float)(*da[1]) : *(float *)da[1];
        data = (int)((temp - da_mid[1])*da_scale[1])+0x800;
        SpiaRegs.SPITXBUF   =  0x1000 | 0x4000 |(data&0xFFF);
        dactemp2 = SpiaRegs.SPIRXBUF;

        temp = da_type[2] ? (float)(*da[2]) : *(float *)da[2];
        data = (int)((temp - da_mid[2])*da_scale[2])+0x800;
        SpiaRegs.SPITXBUF   =  0x1000 | 0x8000 |(data&0xFFF);
        dactemp3 = SpiaRegs.SPIRXBUF;

        temp = da_type[3] ? (float)(*da[3]) : *(float *)da[3];
        data = (int)((temp - da_mid[3])*da_scale[3])+0x800;
        SpiaRegs.SPITXBUF   =  0x1000 | 0xC000 |(data&0xFFF);
        dactemp4 = SpiaRegs.SPIRXBUF;
}











