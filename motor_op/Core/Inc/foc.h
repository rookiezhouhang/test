#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "IQmathLib.h"

void RevPark(float vd, float vq, float theta, float *valpha, float *vbeta);
uint8_t SectorJudge(float valpha, float vbeta);
void VectorActiontime(uint8_t N, float valpha, float vbeta, uint32_t udc, uint32_t tpwm, float *Ta, float *Tb);
void CCRCalculate(uint8_t N, float ta, float tb, uint32_t tpwm, uint32_t *ccr1, uint32_t *ccr2, uint32_t *ccr3);
void Svpwm(float vd, float vq, float theta, uint32_t udc, uint32_t tpwm);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
