#ifndef TEST_LINE_SENSOR_H
#define TEST_LINE_SENSOR_H

#include <stdbool.h>

bool Test_LineSensor_Init(void);
void Test_LineSensor_Update(void);
void Test_LineSensor_Stop(void);
bool Test_LineSensor_IsInitialized(void);

#endif /* TEST_LINE_SENSOR_H */
