/*
 * CAN_0049.c
 *
 *  Created on: 2021. 5. 9.
 *      Author: Plasma Science
 */
#include "UserDefine.h"


    //
    // Setup send and receive buffers
    //

    //
    // Loop Forever - Send and Receive data continuously
    //
void CAN_loop(void)
{
    uint16_t i,j,k,l, CAN_Temp;

    // Read CAN message

    for(k = byte8_tx_cnt + 1; k <= byte8_tx_cnt+byte8_rx_cnt; k++)
    {
        if(CAN_readMessage(CANA_BASE, k, &rxMsgData[(k-byte8_tx_cnt-1)*8]))
        {
            Can_rx_flag = 1;
            for(l = 0; l < 4; l++)
            {
                CAN_Temp   = (uint16_t)rxMsgData[l*2 + (k - byte8_tx_cnt - 1)*8]             & 0x00FF;
                CAN_Temp  += (uint16_t)rxMsgData[l*2 + 1 + (k - byte8_tx_cnt - 1)*8]<<8      & 0xFF00;
                CAN.Arr[sizeof(CAN_TX) + l + (k - byte8_tx_cnt - 1)*4] = CAN_Temp;

            }
            memset_fast(&rxMsgData[(k-byte8_tx_cnt-1)*8], 0, 8);
        }

    }


    for(i=0;i<sizeof(CAN_TX);i++)
        {
        txMsgData[2*i]      = CAN.Arr[i]&0x00ff;
        txMsgData[2*i+1]    = (CAN.Arr[i]&0xff00)>>8;
        }

    // Send CAN message data
    if(Can_rx_flag)
    {
        for(j=1;j<=byte8_tx_cnt;j++)
        {
            CAN_sendMessage(CANA_BASE, j, 8, &txMsgData[(j-1)*8]);
            Can_rx_flag = 0;

        }

    }
}


void Can_Struct_Data(void)
{

    MB.Word.Inv_Power           = CAN.Vari.RX.Inv_Power;
    MB.Word.Inv_cnt             = CAN.Vari.RX.Run_sec;
    MB.Word.Inv_ready           = CAN.Vari.RX.Run_ready;
    MB.Word.IGBT_Tempurature    = CAN.Vari.RX.IGBT_Tempurature;
    MB.Word.Line_Freq           = CAN.Vari.RX.Line_Freq;
    MB.Word.Line_V              = CAN.Vari.RX.Line_V;
    MB.Word.Line_Cur            = CAN.Vari.RX.Line_Cur;
    MB.Word.Inv_sequence        = CAN.Vari.RX.Inv_sequence;

#ifdef DC750V
    CAN.Vari.TX.Enter_key       = MB.Word.Start;
    CAN.Vari.TX.Run_state       = VARI.PROC.ucBST_RUN;
    CAN.Vari.TX.SV_current      = VARI.C_CMD.fFC_I_max*10;
    CAN.Vari.TX.FC_V            = VARI.AVR.fFC_V_100ms*10;
    CAN.Vari.TX.FC_I            = VARI.AVR.fFC_I_100ms*10;
    CAN.Vari.TX.ID              = VARI.PROC.ucModule_ID;
    CAN.Vari.TX.FC_power        = VARI.AVR.fFC_Power_100ms;
    CAN.Vari.TX.DC_Efficiency   = VARI.AVR.fEfficiency_500ms*10;
    CAN.Vari.TX.Temperature     = VARI.AVR.fTemp_em_10ms*10;

    CAN.Vari.TX.DC_Fault1 = (uint16_t)((VARI.FAULT.all&0x01ff)<<3)|(VARI.WARN.all&0x0007); // 0111 1111 1111 1111

    MB.Word.DCAC_fault_1 = CAN.Vari.RX.DCAC_Fault_1;
    MB.Word.DCAC_fault_2 = CAN.Vari.RX.DCAC_Fault_2;
    MB.Word.DCAC_fault_3 = CAN.Vari.RX.DCAC_Fault_3;
    MB.Word.DCAC_fault_4 = CAN.Vari.RX.DCAC_Fault_4;
    MB.Word.DCAC_fault_5 = CAN.Vari.RX.DCAC_Fault_5;
#endif


#ifdef DC360V
//        MB.Word.Max_power           = CAN.Vari.RX.Max_power;
//        MB.Word.fFC_I_CMD           = (uint16_t)PARA.LIMIT.fFC_IL_max * 10;
        MB.Word.DCAC_fault_HW       = CAN.Vari.RX.DCAC_HW_fault;
        MB.Word.DCAC_fault_SW       = CAN.Vari.RX.DCAC_SW_fault;


        CAN.Vari.TX.Start_stop      = MB.Word.Start;
#endif


//#ifdef NOT_COMM
//        VARI.C_CMD.fFC_I_max = 77.;
//        PARA.LIMIT.fFC_IL_max = 77.;
//#endif

}
