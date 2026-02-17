/*
 * UserDefine.h
 *
 *  Created on: 2021. 4. 12.
 *      Author: user
 */



/*******************************************************************/
// 1Phase Mode
// [DCDC_BST] operation
// [Define = DC360V] DCDC converter operating 1Phase

//#define DC360V          1   //DClink Voltage 360V control Mode
/*******************************************************************/

/*******************************************************************/
// 3Phase Mode
// [DCDC_BST] operation
// [Define = DC750V] DCDC converter operating 3Phase

#define DC750V          1   //DClink Voltage 750V control Mode
/*******************************************************************/

/*******************************************************************/
// Voltage Control Mode
// [DCDC_BST] operation
// [Define = VC] DCDC converter operated

#define BST_VC          1   //DClink Voltage 750V control Mode
/*******************************************************************/

/*******************************************************************/
// Current Control Mode
// [DCDC_BST] operation
// [Define = CC] DCDC converter operated

//#define BST_CC          1   //DClink Voltage 750V control Mode
/*******************************************************************/

/*******************************************************************/
// NOT RS-485_RTU Communication Control Mode
// [DCDC_BST] operation
// [Define = VC] DCDC converter operated

#define NOT_COMM          1   //J-tag or easyDSP control Mode
/*******************************************************************/

#define SYSCLK              100000000L    //100MHz
#ifdef DC360V
#define PWM_FREQUENCY       36000
#endif
#ifdef DC750V
#define PWM_FREQUENCY       37500L
#endif



//#define PWM_10ms_inv        (float)__divf32(1, PWM_FREQUENCY * 0.01) * EPwm1Regs.ETPS.bit.SOCAPRD
//#define PWM_100ms_inv       PWM_10ms_inv * 0.1
//#define PWM_500ms_inv       PWM_100ms_inv * 0.2
//#define PWM_1s_inv          PWM_500ms_inv * 0.5


#define TC                  __divf32(1, PWM_FREQUENCY)*4
#define PI                  3.141592654
#define PI2                 PI * 2
#define ROOT2               1.414213562


#define PWM_OFF             GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;
#define PWM_ON              GpioDataRegs.GPASET.bit.GPIO17 = 1;


#define DIV_0x10000     (0.0000152587890625)
#define DAC_CH          4

#define STATE_BST_STOP  0
#define STATE_BST_INIT  1
#define STATE_BST_VC    2
#define STATE_BST_CC    3
#define STATE_FAULT     99

#ifdef DC360V
//#define Turn_Ratio      __divf32(3 , 8)
#define Turn_Ratio      __divf32(3 , 7)
#endif

#ifdef DC750V
#define Turn_Ratio      __divf32(2 , 10)
#endif

#define PriToSec        __divf32(1, Turn_Ratio) * 2 * 2
#define SecToPri        __divf32(1, PriToSec)
#define PriToSec_inv    SecToPri
#define SecToPri_inv    PriToSec


//filter
#define K_ALLPASS   0
#define K_LPF       1
#define K_HPF       2
#define K_BPF       3
#define K_NOTCH     4

//DC Relay
#define DC_Relay1_ON        GpioDataRegs.GPASET.bit.GPIO10 = 1;
#define DC_Relay1_OFF       GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
#define DC_Relay2_ON        GpioDataRegs.GPASET.bit.GPIO7 = 1;
#define DC_Relay2_OFF       GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
#define DC_Relay3_ON        GpioDataRegs.GPASET.bit.GPIO6 = 1;
#define DC_Relay3_OFF       GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

//////////I2C_EEPROM
//
// Error messages for read and write functions
//

#define EEPROM_SLAVE_ADDRESS        0x50

#define ERROR_BUS_BUSY              0x1000
#define ERROR_NACK_RECEIVED         0x2000
#define ERROR_ARBITRATION_LOST      0x3000
#define ERROR_STOP_NOT_READY        0x5555
#define I2C_SUCCESS                 0x0000

#define MAX_BUFFER_SIZE             16
#define I2C_FIFO_LEVEL              16

#define MAX_7_BIT_ADDRESS 127U
#define MAX_10_BIT_ADDRESS 1023U


//////////////////MODBUS
#define BAUDRATE2 38400L
#define MB_SLAVE_ADDRESS            0X01
#define MB_READ_FUNCCODE            0X03
#define MB_WRITE_FUNCCODE           0X06
#define MB_MULT_WRITE_FUNCCODE      0X10
#define MB_NUM_MAX_BUFFER           (sizeof(MB.Word)-1)


//////////////////CAN
#define TX_MSG_DATA_LENGTH    sizeof(CAN_TX)*2        //Struct Byte Count
#define RX_MSG_DATA_LENGTH    sizeof(CAN_RX)*2        //Struct Byte Count

#include "f28004x_cla_typedefs.h"   // f28004x CLA Type definitions
#include "f28004x_device.h"         // f28004x Headerfile Include File
#include "f28004x_examples.h"       // f28004x Examples Include File

#include "math.h"
#include "F28x_Project.h"
#include "device.h"


#include "float.h"
#include "FPU.h"
#include "IQmathLib.h"


#include "extern_Vars.h"
#include "extern_Struct.h"
#include "extern_Func.h"

