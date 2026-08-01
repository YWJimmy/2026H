#ifndef TEST_LINE_ADC_H
#define TEST_LINE_ADC_H

#include <stdbool.h>

bool Test_LineAdc_Init(void);
void Test_LineAdc_Update(void);
void Test_LineAdc_Stop(void);
bool Test_LineAdc_IsInitialized(void);

#endif /* TEST_LINE_ADC_H */
