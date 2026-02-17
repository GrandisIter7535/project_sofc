/***************************************************************
    easy28x_bitfield.h
    v9.1 (Mar 2020) : First release based C2000Ware_2_01_00_00
    v9.2 (Apr 2020) : 1. supports F28002x
                      2. enables pull-up of 28004x Rx pins for better noise immunity
                      3. MCU name change : F2837xD -> F2837xD_CPU1_CPU2, F2837xD_CPU1_ONLY -> F2837xD_CPU1
					  4. F2832x, F2833x and C2834x are moved to easy28x_gen2_bitfield.h
    v9.3 (May 2020) : supports F2838xS and F2838xD
	v9.4 (Sep 2020) : supports one time reading for 4B/8B data
	v9.5 (Dec 2020) : 1. reduce uninitialized variables
                      2. CPU2 RAM booting for F2837xD and F2838xD
	copyright (c) 2020 by DaeWoong Chung
	All Rights Reserved.
****************************************************************/
#ifndef _EASY28X_BITFIELD_H__
#define _EASY28X_BITFIELD_H__

extern void	easyDSP_SCI_Init(void);
extern __interrupt void easy_RXINT_ISR(void);
extern void easyDSP_SPI_Flashrom_Init(void);        // only for C2834x
extern void easyDSP_Boot_Sync(void);

// internal function declaration
inline void AddRing(unsigned char y);

/////////////////////////////////////////////////////////////////////////////////////////
// NOTICE : Please select or modify below MCU type, CPU_CLK, LSP_CLK, BAUDRATE,
//          interrupt nesting, ram function activation according to your target system
///////////////////////////////////////////////////////////////////////////////////////////
// select target MCU. only one.
// 1 = selected, 0 = not selected
#define F28002x             0
#define F28004x             1
#define F2807x              0
#define F2837xS             0
#define F2837xD_CPU1        0           // use only CPU1. name changed from F2837xD_CPU1_ONLY
#define F2837xD_CPU1_CPU2   0           // use both CPU1 and CPU2. name changed from F2837xD
#define F2838xS_CPU1            0       // use only CPU1
#define F2838xS_CPU1_CM         0       // use CPU1+CM
#define F2838xD_CPU1            0       // use only CPU1
#define F2838xD_CPU1_CPU2       0       // use CPU1+CPU2
#define F2838xD_CPU1_CM         0       // use CPU1+CM
#define F2838xD_CPU1_CPU2_CM    0       // use CPU1+CPU2+CM

////////////////////////////////////////////////////////////////////////////////////////////
#define CPU_CLK           100000000L      // 100MHz for ex. 28002x/4x
//#define CPU_CLK           120000000L      // 120MHz for ex. 2807x
//#define CPU_CLK           200000000L      // 200MHz for ex. 2837x and F2838x
////////////////////////////////////////////////////////////////////////////////////////////
#define LSP_CLK             (CPU_CLK/4)     // NOTE : LSP_CLK should be same to CPU_CLK in MotorWare¢â
////////////////////////////////////////////////////////////////////////////////////////////
#define BAUDRATE            38400L
////////////////////////////////////////////////////////////////////////////////////////////
// interrupt nesting assuming easyDSP ISR has the lowest priority. If not, please change the code accordingly
#define INT_NESTING_START   {           \
    IER |= 0x0100;                      \
    PieCtrlRegs.PIEACK.all = 0xFFFF;    \
    asm("       NOP");                  \
    EINT;                               \
    }
#define INT_NESTING_END         DINT
//#define INT_NESTING_START                                                  // in case of no nesting
//#define INT_NESTING_END       (PieCtrlRegs.PIEACK.all = PIEACK_GROUP9)     // in case of no nesting
////////////////////////////////////////////////////////////////////////////////////////////
// if _FLASH is not predefined by CCS configuration, you can do it here
#ifndef _FLASH
//#define _FLASH
#endif
////////////////////////////////////////////////////////////////////////////////////////////
// use #pragma if fast SCI ISR run on the ram is required
#ifdef _FLASH
#pragma CODE_SECTION(easy_RXINT_ISR, ".TI.ramfunc");
#pragma CODE_SECTION(AddRing, ".TI.ramfunc");
#endif

#endif	// of _EASY28X_BITFIELD_H__
