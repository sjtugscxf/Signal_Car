#ifndef ADC_H
#define ADC_H

#include "typedef.h"

void ADC_Init(void);

U16 ADC_accX(void);
U16 ADC_accY(void);
U16 ADC_accZ(void);
U16 ADC_gyroY(void);
U16 ADC_gyroP(void);

#endif