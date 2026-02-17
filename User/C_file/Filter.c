/*
 * Filter.c
 *
 *  Created on: 2021. 4. 17.
 *      Author: Plasma Science
 */
#include "UserDefine.h"

void IIR2CoeffInit(IIR2 *p_gIIR, float w0, float zeta )
{
   float a0, a1, b0, b1, b2;
   float INV_alpha, dt;
   int type;

   // Continuous-time Filter Coefficients
   p_gIIR->w0 = w0;
   p_gIIR->zeta = zeta;
   dt = p_gIIR->delT ;
   type = p_gIIR->type;

   a0 = w0*w0;
   a1 = 2*zeta*w0;
   switch(type)
   {
      case K_LPF:
         b0 = w0*w0;
         b1 = 0;
         b2 = 0;
         break;
      case K_HPF:
         b0 = 0;
         b1 = 0;
         b2 = (float)1;
         break;
      case K_BPF:
         b0 = 0;

         b1 = (float)2*zeta*w0;
         b2 = 0;
         break;
      case K_NOTCH:
         b0 = w0*w0;
         b1 = 0;
         b2 = (float)1;
         break;
      case K_ALLPASS:
      default:
         b0 = w0*w0;
         b1 = -(float)2*zeta*w0;
         b2 = (float)1;
   }

   // Discrete-time Filter Coefficients
   INV_alpha = __divf32((float)1., ((float)4 + (float)2*dt*a1 + dt*dt*a0));               // 계수의 분모 : 1/(4+2*T*a1+T^2*w0^2)
   p_gIIR->coeff[0] = ((float)4*b2 + (float)2*dt*b1 + dt*dt*b0)*INV_alpha;     // b2 계수의 분자
   p_gIIR->coeff[1] = ((float)2*dt*dt*b0 - (float)8*b2)*INV_alpha;             // b1 계수의 분자
   p_gIIR->coeff[2] = -((float)2*dt*dt*a0 - (float)8)*INV_alpha;               // a1 계수의 분자
   p_gIIR->coeff[3] = ((float)4*b2 - (float)2*dt*b1 + dt*dt*b0)*INV_alpha;     // b0 계수의 분자
   p_gIIR->coeff[4] = -((float)4 - (float)2*dt*a1 + dt*dt*a0)*INV_alpha;       // a2 계수의 분자

   return;
}

void IIR2Init(IIR2 *p_gIIR, int type, float w0, float zeta, float delT)
{
   // Initialize Filter Coefficients
   IIR2CoeffInit(p_gIIR, w0, zeta);
    p_gIIR->type = type;
    p_gIIR->delT = delT;
   // Initialize Storage Elements
   p_gIIR->reg[0] = 0;
   p_gIIR->reg[1] = 0;

   return;
}

float IIR2Update(IIR2 *p_gIIR, const float x)
{
   float y;

   y = p_gIIR->reg[0] + p_gIIR->coeff[0]*x;
   p_gIIR->reg[0] = p_gIIR->reg[1] + p_gIIR->coeff[1]*x + p_gIIR->coeff[2]*y;
   p_gIIR->reg[1] = p_gIIR->coeff[3]*x + p_gIIR->coeff[4]*y;

   return(y);
}
