#ifndef PROJECT_SERVO_LEVEL_H
#define PROJECT_SERVO_LEVEL_H

#include <stdint.h>

/*
 * Mechanically calibrated rod-horizontal pulse.
 * Task 3 and Task 4 use the same calibrated physical horizontal pulse.
 * Every MCU reset, task exit, and application reset returns to this level.
 */
#define PROJECT_SERVO_HORIZONTAL_US ((uint16_t)1650U)

#endif /* PROJECT_SERVO_LEVEL_H */
