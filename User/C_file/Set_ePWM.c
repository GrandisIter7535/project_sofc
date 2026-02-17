/*
 * Set_ePWM.c
 *
 *  Created on: 2021. 4. 13.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

void PWM_Disable(void)
{
    PWM_OFF;

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // Configure GPIO0 as GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;   // Configure GPIO1 as GPIO1
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // Configure GPIO2 as GPIO2
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;   // Configure GPIO3 as GPIO3
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3

    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;

    DC_Relay1_OFF;
    EDIS;
}

void PWM_Enable(void)
{
    PWM_ON;

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

void Init_EPwm(void)
{
    // EPWM Configuration

    //--------------------------------- BST PWM ----------------------------------//
    //EPWM1 setting
    // Steup TBCLK
    EPwm1Regs.TBPRD = SYSCLK / (PWM_FREQUENCY * 2);           //PWM1_TIMER_TBPRD;
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;                 // ���� �������� �ʱ�ȭ (Phase is 0)
    EPwm1Regs.TBCTR = 0x0000;                           // Counter Clear

    // �� �������� �ʱⰪ ����
    EPwm1Regs.CMPA.bit.CMPA = 0;                        // Duty-ratio : 0 %
    EPwm1Regs.CMPB.bit.CMPB = 0;                        // Duty-ratio : 0 %

    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Counter mode(CTR mode) : Up-Down count Mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Disable phase loading(0x0)
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;              // 0x0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         // 0x1  //check
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // High speed Time-Base Clock prescale : /1
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;               // Time-Base Clock prescale :            /1 TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV)

    //ePWM1 Ÿ�̸� ī���Ͱ� TBPRD�� ��ġ�� ��, CPU�� ���ͷ�Ʈ�� ��û
//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;            // ���ͷ�Ʈ ���� ���� : Ÿ�̸� ī���Ͱ� Ÿ�̸� TBPRD�� ��ġ�� ��
//    EPwm1Regs.ETSEL.bit.INTEN = 1;                      // EPWM ��� ���ͷ�Ʈ ��� Ȱ��ȭ(Enable Interrupt)
//    EPwm1Regs.ETPS.bit.INTPRD = ET_2ND;                 // ���ͷ�Ʈ ���� �ֱ� : �� ���ͷ�Ʈ ���� ���ǹ߻� �� CPU�� ���ͷ�Ʈ ��û

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                     // SOCA �̺�Ʈ Ʈ���� Enable
    EPwm1Regs.ETSEL.bit.SOCASEL= ET_CTR_PRD;            // SCCA Ʈ���� ���� :
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_2ND;                // SOCA �̺�Ʈ ���� ���� : Ʈ���� ���� �ѹ� ����

    // �� �������Ϳ� ���� ������ ��� ����
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // ������ �������� �� ��� Ȱ��ȭ (Enable Shadow Register & Function)
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       // Ÿ�̸� ī���Ͱ� 0�� ��ġ�� ��, ������ ���������� ���� Ȱ��(Active) �������Ϳ� �ݿ�
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Ÿ�̸� �̺�Ʈ�� ���� PWM ��� �� ���� ����
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;                // Ÿ�̸� ī���Ͱ� ��°�� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� High�� ����
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;                  // Ÿ�̸� ī���Ͱ� �ϰ���� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� Low�� ����

    //Dead-Time ����
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm1Regs.DBRED.bit.DBRED = 50;              // Rising dead time = (1 / TBCLK) * DBRED @ TBCLK = 100Mhz
    EPwm1Regs.DBFED.bit.DBFED = 50;              // Falling dead time = (1 / TBCLK) * DBFED
                                                        // ex) DBFED = 200 , TBCLK = 100Mhz -> 2 usec
    //EPWM2 setting
    EPwm2Regs.TZCLR.all = 0x07;
    //Time-Base setting                               // Counter Clear

    EPwm2Regs.TBPRD = SYSCLK / (PWM_FREQUENCY * 2);           //PWM1_TIMER_TBPRD;
    EPwm2Regs.TBPHS.bit.TBPHS = EPwm1Regs.TBPRD;                      // ���� �������� �ʱ�ȭ (Phase is 180)
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;      // Slave Mode
    EPwm2Regs.TBCTR = 0;                                // Counter Clear
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Counter mode(CTR mode) : Up-Down count Mode            // 0x0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         // 0x1  //check
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // High speed Time-Base Clock prescale : /1
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;               // Time-Base Clock prescale :            /1 TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV)

    // �� �������Ϳ� ���� ������ ��� ����
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // ������ �������� �� ��� Ȱ��ȭ (Enable Shadow Register & Function)
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       // Ÿ�̸� ī���Ͱ� 0�� ��ġ�� ��, ������ ���������� ���� Ȱ��(Active) �������Ϳ� �ݿ�
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Ÿ�̸� �̺�Ʈ�� ���� PWM ��� �� ���� ����
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;                // Ÿ�̸� ī���Ͱ� ��°�� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� High�� ����
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;                  // Ÿ�̸� ī���Ͱ� �ϰ���� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� Low�� ����

    // �� �������� �ʱⰪ ����
    EPwm2Regs.CMPA.bit.CMPA = 0;                        // Duty-ratio : 0 %
    EPwm2Regs.CMPB.bit.CMPB = 0;                        // Duty-ratio : 0 %

    //Dead-Time ����
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm2Regs.DBRED.bit.DBRED = 50;              // Rising dead time = (1 / TBCLK) * DBRED @ TBCLK = 100Mhz
    EPwm2Regs.DBFED.bit.DBFED = 50;              // Falling dead time = (1 / TBCLK) * DBFED
                                                        // ex) DBFED = 200 , TBCLK = 100Mhz -> 2 usec


    //--------------------------------- FAN PWM ----------------------------------//


    EPwm3Regs.TZCLR.all = 0x07;

    // �� �������� �ʱⰪ ����
    EPwm3Regs.CMPA.bit.CMPA = 0;                        // Duty-ratio : 0 %
    EPwm3Regs.CMPB.bit.CMPB = 0;                        // Duty-ratio : 0 %

    EPwm3Regs.TBPRD = SYSCLK / ((Uint32)25000 * 2);           //PWM1_TIMER_TBPRD;
    EPwm3Regs.TBPHS.bit.TBPHS = EPwm1Regs.TBPRD;                      // ���� �������� �ʱ�ȭ (Phase is 180)
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // Slave Mode
    EPwm3Regs.TBCTR = 0;                                // Counter Clear
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;      // Counter mode(CTR mode) : Up-Down count Mode            // 0x0
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;         // 0x1  //check
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;            // High speed Time-Base Clock prescale : /1
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;               // Time-Base Clock prescale :            /1 TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV)

    // �� �������Ϳ� ���� ������ ��� ����
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;         // ������ �������� �� ��� Ȱ��ȭ (Enable Shadow Register & Function)
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;       // Ÿ�̸� ī���Ͱ� 0�� ��ġ�� ��, ������ ���������� ���� Ȱ��(Active) �������Ϳ� �ݿ�
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Ÿ�̸� �̺�Ʈ�� ���� PWM ��� �� ���� ����
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;                // Ÿ�̸� ī���Ͱ� ��°�� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� High�� ����
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;                  // Ÿ�̸� ī���Ͱ� �ϰ���� �� �� �������� A�� ��ġ�� ��, PWM A ��Ʈ ���¸� Low�� ����

    // �� �������� �ʱⰪ ����
    EPwm3Regs.CMPA.bit.CMPA = 0;                        // Duty-ratio : 0 %
    EPwm3Regs.CMPB.bit.CMPB = 0;                        // Duty-ratio : 0 %

    //Dead-Time ����
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm3Regs.DBRED.bit.DBRED = 0;              // Rising dead time = (1 / TBCLK) * DBRED @ TBCLK = 100Mhz
    EPwm3Regs.DBFED.bit.DBFED = 0;              // Falling dead time = (1 / TBCLK) * DBFED
                                                        // ex) DBFED = 200 , TBCLK = 100Mhz -> 2 usec


//    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);


}

