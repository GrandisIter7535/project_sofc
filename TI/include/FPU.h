//###########################################################################
//  This software is licensed for use with Texas Instruments C28x
//  family DSCs.  This license was provided to you prior to installing
//  the software.  You may review this license by consulting a copy of
//  the agreement in the doc directory of this library.
// ------------------------------------------------------------------------
//          Copyright (C) 2010 Texas Instruments, Incorporated.
//                          All Rights Reserved.
// ==========================================================================
//
// FILE:   FPU.h
//
// TITLE:  Prototypes and Definitions for the C28x FPU Library
//
//###########################################################################
// $TI Release: C28x Floating Point Unit Library V1.30 $
// $Release Date: January 04, 2012 $
//###########################################################################

#ifndef C28X_FPU_LIB_H
#define C28X_FPU_LIB_H


#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Standard C28x Data Types
//-----------------------------------------------------------------------------


#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int                 int16;
typedef long                int32;
typedef long long           int64;
typedef unsigned int        Uint16;
typedef unsigned long       Uint32;
typedef unsigned long long  Uint64;
typedef float               float32;
typedef long double         float64;
#endif


//-----------------------------------------------------------------------------
// Float32 Definitions and Prototypes
//-----------------------------------------------------------------------------

typedef struct {
  float32  *InBuf;
  float32  *OutBuf;
  float32  *CosSinBuf;
  float32  *MagBuf;
  float32  *PhaseBuf;
  Uint16   FFTSize;
  Uint16   FFTStages;
} RFFT_F32_STRUCT;

typedef struct {
  Uint16   *InBuf;
  void	   *Tail;
} RFFT_ADC_F32_STRUCT;

typedef struct {
	float32 dat[2];
} complex_float;

typedef struct {
	float32	*InPtr;
	float32	*OutPtr;
	float32	*CoefPtr;
	float32	*CurrentInPtr;
	float32	*CurrentOutPtr;
	Uint16	Stages;
	Uint16  FFTSize;
}CFFT_F32_STRUCT;


extern void CFFT_f32(CFFT_F32_STRUCT *);    			// Complex FFT
extern void CFFT_f32u(CFFT_F32_STRUCT *); 				// Complex FFT, unaligned input
extern void CFFT_f32_sincostable(CFFT_F32_STRUCT *);	// Twiddle Factor Generation
extern void CFFT_f32_mag(CFFT_F32_STRUCT *);			// Complex FFT, magnitude
extern void CFFT_f32s_mag(CFFT_F32_STRUCT *);			// Complex FFT, scaled magnitude
extern void CFFT_f32_phase(CFFT_F32_STRUCT *);			// Complex FFT,	phase
extern void ICFFT_f32(CFFT_F32_STRUCT *);				// Inverse Complex FFT, aligned input

extern void RFFT_f32(RFFT_F32_STRUCT *);                // Real FFT, aligned input
extern void RFFT_f32u(RFFT_F32_STRUCT *);               // Real FFT, unaligned input
extern void RFFT_adc_f32(RFFT_ADC_F32_STRUCT *);        // Real FFT with adc input, aligned input
extern void RFFT_adc_f32u(RFFT_ADC_F32_STRUCT *);		// Real FFT with adc input, unaligned input
extern void RFFT_f32_mag(RFFT_F32_STRUCT *);            // Real FFT, magnitude
extern void RFFT_f32s_mag(RFFT_F32_STRUCT *);           // Real FFT, sclaed magnitude
extern void RFFT_f32_phase(RFFT_F32_STRUCT *);          // Real FFT, phase
extern void RFFT_f32_sincostable(RFFT_F32_STRUCT *);    // Real FFT, twiddle calculation

extern void abs_SP_CV(float32 *, const complex_float *, const Uint16);
extern void abs_SP_CV_2(float32 *, const complex_float *, const Uint16);
extern void add_SP_CSxCV(complex_float *, const complex_float *, const complex_float, const Uint16);
extern void add_SP_CVxCV(complex_float *, const complex_float *, const complex_float *, const Uint16);
extern void iabs_SP_CV(float32 *, const complex_float *, const Uint16);
extern void iabs_SP_CV_2(float32 *, const complex_float *, const Uint16);
extern Uint16 maxidx_SP_RV_2(float32 *, Uint16);
extern complex_float mean_SP_CV_2(const complex_float *, const Uint16);
extern float32 median_noreorder_SP_RV(const float32 *, Uint16);
extern float32 median_SP_RV(float32 *, Uint16);
extern void memcpy_fast(void *, const void *, Uint16);
extern void memset_fast(void*, int16, Uint16);
extern complex_float mpy_SP_CSxCS(complex_float, complex_float);
extern void mpy_SP_CVxCV(complex_float *, const complex_float *, const complex_float *, const Uint16);
extern void mpy_SP_CVxCVC(complex_float *, const complex_float *, const complex_float *, const Uint16);
extern void mpy_SP_RSxRV_2(float32 *, const float32 *, const float32, const Uint16);
extern void mpy_SP_RSxRVxRV_2(float32 *, const float32 *, const float32 *, const float32, const Uint16);
extern void mpy_SP_RVxCV(complex_float *, const complex_float *, const float32 *, const Uint16);
extern void mpy_SP_RVxRV_2(float32 *, const float32 *, const float32 *, const Uint16);
extern void qsort_SP_RV(void *, Uint16);
extern float32 rnd_SP_RS(float32);
extern void sub_SP_CSxCV(complex_float *, const complex_float *, const complex_float, const Uint16);
extern void sub_SP_CVxCV(complex_float *, const complex_float *, const complex_float *, const Uint16);

inline static float32 __ffsqrtf(float32 x)
{
    float32 dst,tmp2, tmp3;

    dst = __eisqrtf32(x);
    tmp2 = x * 0.5;
    tmp3 = dst * dst;
    tmp3 = tmp3 * tmp2;
    tmp3 = 1.5 - tmp3;
    dst = dst * tmp3;
    tmp3 = dst * tmp2;
    tmp3 = dst * tmp3;
    tmp3 = 1.5 - tmp3;
    dst = dst * tmp3;
    dst = x * dst;
    return dst;
}

#define NULL    0
 
  
//-----------------------------------------------------------------------------
//Define the structure of the FIRFILT_GEN Filter Module 
//-----------------------------------------------------------------------------
typedef struct { 
    float *coeff_ptr;        /* Pointer to Filter coefficient */
    float *dbuffer_ptr;      /* Delay buffer ptr              */
    int	cbindex;			 /* Circular Buffer Index         */
    int order;               /* Order of the Filter           */
    float input;             /* Latest Input sample           */ 
    float output;            /* Filter Output                 */
    void (*init)(void *);    /* Ptr to Init funtion           */
    void (*calc)(void *);    /* Ptr to calc fn                */  
    }FIR_FP;

typedef FIR_FP 	*FIR_FP_handle; 							//Define a Handles for the Filter Modules


#define FIR_FP_DEFAULTS { (float *)NULL, \
             (float *)NULL,  \
             0,              \
             50,             \
             0,				 \
             0,				 \
             (void (*)(void *))FIR_FP_init,\
             (void (*)(void *))FIR_FP_calc}    

extern void FIR_FP_calc(void *);
extern void FIR_FP_init(void *);

extern void FIR_FP_calc_c(FIR_FP *);
extern void FIR_FP_init_c(FIR_FP *);


//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of C28X_FPU_LIB_H

//===========================================================================
// End of file.
//===========================================================================
