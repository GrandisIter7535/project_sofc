/*
 * Modbus.c
 *
 *  Created on: 2021. 4. 18.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

unsigned short usMBCRC16(unsigned char *pucFrame, unsigned short usLen)
{
    unsigned char   ucCRCHi = 0xFF;
    unsigned char   ucCRCLo = 0xFF;
    int             iIndex;

    while(usLen--)
    {
        iIndex = ucCRCLo ^ *(pucFrame++ );
        ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex];
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ucCRCHi << 8 | ucCRCLo;
}



void Master_Read_Request(void)
{
//Function code 0x03


    MB_CRC_Check9 = usMBCRC16((unsigned char*)&Master_Read_Data,6);
    MB_CRC_H9 = MB_CRC_Check9 & 0x00FF;
    MB_CRC_L9 = (MB_CRC_Check9 & 0xFF00) >> 8;

    if((MB_CRC_H9 == Master_Read_Data[6]) && (MB_CRC_L9 == Master_Read_Data[7]))
    {
        Master_Read_Transmit_Flag = 1;

    }
    else
    {
       MB_CRC_Check_Fault_Flag9 = 1;
    }

    TX_Buff_Num_b = 0;
}

void Master_Read_Transmit(void)
{

    unsigned char count2 = 0;
    unsigned char a2 = 0;
    unsigned char a4 = 0;
    unsigned int i, Num_data, Num_databyte, Num_add;

    Num_data = (Master_Read_Data[4]<<8)|(Master_Read_Data[5]);
    Num_databyte = (Master_Read_Data[4]<<8)|(Master_Read_Data[5])*2;
    Num_add = (Master_Read_Data[2]<<8)|(Master_Read_Data[3]);

   // unsigned char amicount2 = 0;
       //slave address x01
       //function code x04
       //byte count (2*n) n-> WORD=2B,29W
       //REG H
       //REG L
       //CRC H
       //CRC L

       Master_Transmit_Data[0] = 0x01; //slave address
       Master_Transmit_Data[1] = 0X03; //function code
       Master_Transmit_Data[2] = Num_databyte; //Byte Count 38 Byte
       for(i=0;i<Num_data;i++)
           {
           Master_Transmit_Data[2*i+3] = (MB.Arr[i+Num_add]&0xff00)>>8;
           Master_Transmit_Data[2*i+4] = MB.Arr[i+Num_add]&0x00ff;
           }


       MB_CRC_Check2 = usMBCRC16((unsigned char*)Master_Transmit_Data,Num_databyte+3);
       MB_CRC_H2 = MB_CRC_Check2 & 0x00FF;
       MB_CRC_L2 = (MB_CRC_Check2 >> 8) & 0x00FF;

      Master_Transmit_Data[Num_databyte+3] = (unsigned char)MB_CRC_H2;
      Master_Transmit_Data[Num_databyte+4] = (unsigned char)MB_CRC_L2;


       GpioDataRegs.GPASET.bit.GPIO13 = 1;
       for(a2=0;a2<=17;a2++)
       {
           Master_Read_Data[a2] = 0x00;

       }


       for(count2=0;count2<=Num_databyte+6;count2++)
       {
          while(ScibRegs.SCIFFTX.bit.TXFFST !=0){}

          ScibRegs.SCITXBUF.bit.TXDT = Master_Transmit_Data[count2];

       }

       GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;

       for(a4=0;a4<=51;a4++)
       {
           Master_Transmit_Data[a4] = 0x00;

       }


}



void Master_Write_Parsing(void)
{
//Function code x10
    unsigned int i, Num_data, Num_databyte, Num_add;
    unsigned char a7 = 0;
    unsigned char CRC_H4 = 0;
    unsigned char CRC_L4 = 0;
    Master_Write_Receive_Flag = 0;
    Num_data = (Master_Read_Data[4]<<8)|(Master_Read_Data[5]);
    Num_databyte = (Master_Read_Data[4]<<8)|(Master_Read_Data[5])*2;
    Num_add = (Master_Read_Data[2]<<8)|(Master_Read_Data[3]);

    if(Master_Read_Data[1] == MB_MULT_WRITE_FUNCCODE)
    {
        MB_CRC_Check4 = usMBCRC16((unsigned char*)&Master_Read_Data,Num_databyte+0x07);
        MB_CRC_H4 = MB_CRC_Check4 & 0x00FF;
        MB_CRC_L4 = (MB_CRC_Check4 >> 8) & 0x00FF;

        CRC_H4 = Master_Read_Data[6] + 7;
        CRC_L4 = Master_Read_Data[6] + 8;

        if((MB_CRC_H4 == Master_Read_Data[CRC_H4]) && (MB_CRC_L4 == Master_Read_Data[CRC_L4]))
        {
            if((Num_data+Num_add)<=MB_NUM_MAX_BUFFER+1)
            {
            Master_Write_Response_Flag = 1;

            Starting_Add2.Byte.D0 = Master_Read_Data[3];
            Starting_Add2.Byte.D1 = Master_Read_Data[2];

            NoRegs2.Byte.D0 = Master_Read_Data[5];
            NoRegs2.Byte.D1 = Master_Read_Data[4];

            for(i=0;i<Num_data;i++)   MB.Arr[i+Num_add] = (Master_Read_Data[2*i+7]<<8|Master_Read_Data[2*i+8]);
            }
        }
        else
        {
            MB_CRC_Check_Fault_Flag4 = 1;
        }

        TX_Buff_Num_b =0;

        for(a7=0;a7<=17;a7++)
        {
           Master_Read_Data[a7] = 0x00;

        }

    }
    else if(Master_Read_Data[1] == MB_WRITE_FUNCCODE)
    {
        MB_CRC_Check4 = usMBCRC16((unsigned char*)&Master_Read_Data,0x06);
        MB_CRC_H4 = MB_CRC_Check4 & 0x00FF;
        MB_CRC_L4 = (MB_CRC_Check4 >> 8) & 0x00FF;

        if((MB_CRC_H4 == Master_Read_Data[6]) && (MB_CRC_L4 == Master_Read_Data[7]))
        {
            if((Num_add)<=MB_NUM_MAX_BUFFER)
            {
            Master_Write_Response_Flag = 1;
            Master_SingleWrite_Response_Flag = 1;

            Starting_Add2.Byte.D0 = Master_Read_Data[3];
            Starting_Add2.Byte.D1 = Master_Read_Data[2];

            NoRegs2.Byte.D0 = Master_Read_Data[5];
            NoRegs2.Byte.D1 = Master_Read_Data[4];

            MB.Arr[Num_add] = (Master_Read_Data[4]<<8|Master_Read_Data[5]);

            }
        }
        else
        {
            MB_CRC_Check_Fault_Flag4 = 1;
        }

        TX_Buff_Num_b =0;

        for(a7=0;a7<=17;a7++)
        {
           Master_Read_Data[a7] = 0x00;

        }
    }
}
void Master_Write_Response(void)
{
    unsigned char a8 = 0;
//Function code x10
    unsigned char response_count2 = 0;

    if(Master_SingleWrite_Response_Flag)
    {
        Master_SingleWrite_Response_Flag = 0;
        Master_Transmit_Data[0] = 0x01; //slave address
        Master_Transmit_Data[1] = 0x06; //function code
        Master_Transmit_Data[2] = Starting_Add2.Byte.D1; //starting add hi
        Master_Transmit_Data[3] = Starting_Add2.Byte.D0; //starting add lo
        Master_Transmit_Data[4] = NoRegs2.Byte.D1; //num of input reg hi
        Master_Transmit_Data[5] = NoRegs2.Byte.D0; //num of input reg lo

        MB_CRC_Check10 = usMBCRC16((unsigned char*)Master_Transmit_Data,0x06);
        MB_CRC_H10 = MB_CRC_Check10 & 0x00FF;
        MB_CRC_L10 = (MB_CRC_Check10 >> 8) & 0x00FF;



        Master_Transmit_Data[6] = MB_CRC_H10; //crc
        Master_Transmit_Data[7] = MB_CRC_L10; //crc

        GpioDataRegs.GPASET.bit.GPIO13 = 1;

        for(response_count2=0;response_count2<=9;response_count2++)
        {
            while(ScibRegs.SCIFFTX.bit.TXFFST !=0){}


            ScibRegs.SCITXBUF.bit.TXDT = Master_Transmit_Data[response_count2];

        }
        GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
        for(a8=0;a8<=8;a8++)
        {
            Master_Transmit_Data[a8] = 0x00;

        }
    }
    else
    {
        Master_Transmit_Data[0] = 0x01; //slave address
        Master_Transmit_Data[1] = 0x10; //function code
        Master_Transmit_Data[2] = Starting_Add2.Byte.D1; //starting add hi
        Master_Transmit_Data[3] = Starting_Add2.Byte.D0; //starting add lo
        Master_Transmit_Data[4] = NoRegs2.Byte.D1; //num of input reg hi
        Master_Transmit_Data[5] = NoRegs2.Byte.D0; //num of input reg lo

        MB_CRC_Check10 = usMBCRC16((unsigned char*)Master_Transmit_Data,0x06);
        MB_CRC_H10 = MB_CRC_Check10 & 0x00FF;
        MB_CRC_L10 = (MB_CRC_Check10 >> 8) & 0x00FF;



        Master_Transmit_Data[6] = MB_CRC_H10; //crc
        Master_Transmit_Data[7] = MB_CRC_L10; //crc

        GpioDataRegs.GPASET.bit.GPIO13 = 1;

        for(response_count2=0;response_count2<=9;response_count2++)
        {
            while(ScibRegs.SCIFFTX.bit.TXFFST !=0){}


            ScibRegs.SCITXBUF.bit.TXDT = Master_Transmit_Data[response_count2];

        }
        GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
        for(a8=0;a8<=8;a8++)
        {
            Master_Transmit_Data[a8] = 0x00;

        }
    }
}




interrupt void scibRxFifoIsr(void)              /// UI2 & AMI2 - 485
{

    EINT;   // Enable Global interrupt INTM

    Master_Read_Data[TX_Buff_Num_b] = ScibRegs.SCIRXBUF.all;
    TX_Buff_Num_b++;

    if(Master_Read_Data[0] == MB_SLAVE_ADDRESS)
    {
        if(Master_Read_Data[1] == MB_MULT_WRITE_FUNCCODE)
        {
            if(TX_Buff_Num_b == (9 + ((Master_Read_Data[4]<<8)|(Master_Read_Data[5]))*2))
            {
                Master_Write_Receive_Flag = 1;
                TX_Buff_Num_b = 0;

            }
        }
        if(Master_Read_Data[1] == MB_WRITE_FUNCCODE)
        {
            if(TX_Buff_Num_b == 8)
            {
                Master_Write_Receive_Flag = 1;
                TX_Buff_Num_b = 0;

            }
        }
        else if(Master_Read_Data[1] == MB_READ_FUNCCODE)
        {
            if(TX_Buff_Num_b == 8)
            {
                Master_Read_Request_Flag = 1;
                TX_Buff_Num_b = 0;
            }
        }
        else if((Master_Read_Data[1] != 0X03)&&(Master_Read_Data[1] != 0x10)&&(Master_Read_Data[1] != 0x06))
        {
            if(TX_Buff_Num_b >= 8)
            {
            TX_Buff_Num_b = 0;
            }
        }
    }


    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;            // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;       // Issue PIE ack
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;            // Clear Overflow flag



}


void Modbus_Struct_Data(void)
{
#ifdef DC360V
    if(VARI.AVR.fClamp_V_100ms <10.) VARI.AVR.fClamp_V_100ms = 0.;
    MB.Word.fFC_V               = VARI.AVR.fClamp_V_100ms*10;
#endif
#ifdef DC750V
    if(VARI.AVR.fFC_V_100ms <10.) VARI.AVR.fFC_V_100ms = 0.;
    MB.Word.fFC_V         = VARI.AVR.fFC_V_100ms*10;
#endif

    MB.Word.fFC_I               = VARI.AVR.fFC_I_100ms*10;
    MB.Word.fFC_power           = VARI.AVR.fFC_Power_100ms;
    MB.Word.fEfficiency         = VARI.AVR.fEfficiency_500ms*10;
    MB.Word.fTemp_HS1           = VARI.AVR.fTemp_em_10ms*10;
    MB.Word.Heatbeats           = MB_Heartbeats;

    if(VARI.PROC.ucBST_RUN) MB.Word.ucBST_RUN_State = 1;
    else                    MB.Word.ucBST_RUN_State = 0;

    MB.Word.DCDC_fault = (uint16_t)((VARI.FAULT.all&0x01ff)<<3)|(VARI.WARN.all&0x0007); // 0111 1111 1111 1111
}

void Modbus_loop(void)
{
    //function code 0x10, request(scib)
    if(Master_Write_Receive_Flag == 1)
    {
        Master_Write_Parsing();
        Master_Write_Receive_Flag = 0;
    }

    //function code 0x10, response(scib)
    if(Master_Write_Response_Flag == 1)
    {
        Master_Write_Response();
        Master_Write_Response_Flag = 0;
    }
    //function code 0x04, request(scib)
    if(Master_Read_Request_Flag == 1)
    {
        Master_Read_Request();
        Master_Read_Request_Flag = 0;
    }

    //function code 0x04, response(scib)
    if(Master_Read_Transmit_Flag == 1)
    {
        Master_Read_Transmit();
        Master_Read_Transmit_Flag = 0;
    }
}
