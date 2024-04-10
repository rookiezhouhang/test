#include "foc.h"
#include "tim.h"
//#include "math.h"

void RevPark(float vd, float vq, float theta, float *valpha, float *vbeta)
{
	_iq Cosine,Sine;
	Cosine = _IQcosPU(theta);
	Sine = _IQsinPU(theta);
	
	*valpha = _IQtoF(_IQmpy(_IQ(vd),Cosine) - _IQmpy(_IQ(vq),Sine));
	*vbeta = _IQtoF(_IQmpy(_IQ(vd),Sine) + _IQmpy(_IQ(vq),Cosine));
//	*valpha = vd*cos(theta) - vq*sin(theta);
//	*vbeta = vd*sin(theta) + vq*cos(theta);
}

uint8_t SectorJudge(float valpha, float vbeta)
{
	float A = vbeta;
	float B = _IQtoF(_IQmpy(_IQ(0.8660254),_IQ(valpha)) - _IQmpy(_IQ(0.5),_IQ(vbeta)));
	float C = _IQtoF(_IQmpy(_IQ(-0.8660254),_IQ(valpha)) - _IQmpy(_IQ(0.5),_IQ(vbeta)));
	
	uint8_t N = 0;
	if(A > 0)
		N = N + 1;
	if(B > 0)
		N = N + 2;
	if(C > 0)
		N = N + 4;
	
	return N;
}

void VectorActiontime(uint8_t N, float valpha, float vbeta, uint32_t udc, uint32_t tpwm, float *Ta, float *Tb)
{
	uint32_t u = 1.7320508 * tpwm / udc;//X Y Z共有的系数
	
	_iq x,y,z; 
	x = _IQmpy(_IQ(u),_IQ(valpha));
	y = _IQmpy(_IQ(u),(_IQmpy(_IQ(0.8660254),_IQ(valpha)) - _IQmpy(_IQ(0.5),_IQ(vbeta))));
	z = _IQmpy(_IQ(u),(_IQmpy(_IQ(-0.8660254),_IQ(valpha)) - _IQmpy(_IQ(0.5),_IQ(vbeta))));
	
	float X = _IQtoF(x);
	float Y = _IQtoF(y);
	float Z = _IQtoF(z);
	
	switch (N){
		case 3:
			*Ta = Y; 
			*Tb = X;
			break;
		case 1:
			*Ta = -Y; 
			*Tb = -Z;
			break;
		case 5:
			*Ta = X; 
			*Tb = Z;
			break;
		case 4:
			*Ta = -X; 
			*Tb = -Y;
			break;
		case 6 :
			*Ta = Z; 
			*Tb = Y;
			break;
		case 2:
			*Ta = -Z; 
			*Tb = -X;
			break;
		default:
			break;
	}
		
}

void CCRCalculate(uint8_t N, float ta, float tb, uint32_t tpwm, uint32_t *ccr1, uint32_t *ccr2, uint32_t *ccr3)
{
	float temp = ta + tb;
	if(temp > tpwm){
		ta = ta/temp *tpwm;
		tb = tb/temp *tpwm;
	}
	
	float value1 = (tpwm - ta - tb)/4.0f;
	float value2 = value1 + ta/2.0f;
	float value3 = value2 + tb/2.0f;
	
	switch(N){
		case 3:
			*ccr1 = value1;
			*ccr2 = value2;
			*ccr3 = value3;
			break;
		case 1:
			*ccr1 = value2;
			*ccr2 = value1;
			*ccr3 = value3;
			break;
		case 5:
			*ccr1 = value3;
			*ccr2 = value1;
			*ccr3 = value2;
			break;
		case 4:
			*ccr1 = value3;
			*ccr2 = value2;
			*ccr3 = value1;
			break;
		case 6:
			*ccr1 = value2;
			*ccr2 = value3;
			*ccr3 = value1;
			break;
		case 2:
			*ccr1 = value1;
			*ccr2 = value3;
			*ccr3 = value2;
			break;
	}
}

void Svpwm(float vd, float vq, float theta, uint32_t udc, uint32_t tpwm)
{
	float valpha = 0;
	float vbeta = 0;
	float ta = 0;
	float tb = 0;
	uint32_t ccr1 = 0;
	uint32_t ccr2 = 0;
	uint32_t ccr3 = 0;
	
	RevPark(vd,vq,theta,&valpha,&vbeta);//计算valpha和vbeta
	uint8_t N = SectorJudge(valpha,vbeta);//扇区判断
	VectorActiontime(N,valpha,vbeta,udc,tpwm,&ta,&tb);//计算矢量作用时间
	CCRCalculate(N,ta,tb,tpwm,&ccr1,&ccr2,&ccr3);//计算CCR值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,ccr1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,ccr2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,ccr3);
//	TIM1->CCR1 = ccr1;
//	TIM1->CCR2 = ccr2;
//	TIM1->CCR3 = ccr3;
}
