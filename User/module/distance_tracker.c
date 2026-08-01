#include "distance_tracker.h"

#include "bsp_encoder.h"
#include "chassis_config.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static bool s_initialized = false;
static bool s_has_source_sequence = false;

static DistanceTrackerStatus_t s_status;

static uint32_t DistanceTracker_AbsI16(
    int16_t value)
{
    if (value >= 0)
    {
        return (uint32_t)value;
    }

    return (uint32_t)(-(int32_t)value);
}

/*
 * 先做整圈商和余数，再换算微米，降低大计数乘法溢出风险。
 */
static int64_t DistanceTracker_SignedCountsToUm(
    int64_t count)
{
    int64_t revolutions;
    int64_t remainder;

    revolutions =
        count / (int64_t)BSP_ENCODER_COUNTS_PER_REV;

    remainder =
        count % (int64_t)BSP_ENCODER_COUNTS_PER_REV;

    return revolutions *
               (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM +
           remainder *
               (int64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM /
               (int64_t)BSP_ENCODER_COUNTS_PER_REV;
}

static uint64_t DistanceTracker_UnsignedCountsToUm(
    uint64_t count)
{
    uint64_t revolutions;
    uint64_t remainder;

    revolutions =
        count / (uint64_t)BSP_ENCODER_COUNTS_PER_REV;

    remainder =
        count % (uint64_t)BSP_ENCODER_COUNTS_PER_REV;

    return revolutions *
               (uint64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM +
           remainder *
               (uint64_t)CHASSIS_WHEEL_CIRCUMFERENCE_UM /
               (uint64_t)BSP_ENCODER_COUNTS_PER_REV;
}

static int32_t DistanceTracker_SaturateI64ToI32(
    int64_t value)
{
    if (value > (int64_t)INT32_MAX)
    {
        return INT32_MAX;
    }

    if (value < (int64_t)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)value;
}

static uint32_t DistanceTracker_SaturateU64ToU32(
    uint64_t value)
{
    if (value > (uint64_t)UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)value;
}

static void DistanceTracker_Recalculate(void)
{
    int64_t center_um_x2;
    uint64_t traveled_um_x2;
    int64_t difference_um;

    s_status.left_signed_um =
        DistanceTracker_SignedCountsToUm(
            s_status.left_count);

    s_status.right_signed_um =
        DistanceTracker_SignedCountsToUm(
            s_status.right_count);

    center_um_x2 =
        DistanceTracker_SignedCountsToUm(
            s_status.center_count_x2);

    traveled_um_x2 =
        DistanceTracker_UnsignedCountsToUm(
            s_status.traveled_count_x2);

    s_status.center_signed_um =
        center_um_x2 / 2LL;

    s_status.traveled_um =
        traveled_um_x2 / 2ULL;

    s_status.left_signed_mm =
        DistanceTracker_SaturateI64ToI32(
            s_status.left_signed_um / 1000LL);

    s_status.right_signed_mm =
        DistanceTracker_SaturateI64ToI32(
            s_status.right_signed_um / 1000LL);

    s_status.center_signed_mm =
        DistanceTracker_SaturateI64ToI32(
            s_status.center_signed_um / 1000LL);

    s_status.traveled_mm =
        DistanceTracker_SaturateU64ToU32(
            s_status.traveled_um / 1000ULL);

    difference_um =
        s_status.left_signed_um -
        s_status.right_signed_um;

    s_status.wheel_difference_mm =
        DistanceTracker_SaturateI64ToI32(
            difference_um / 1000LL);
}

bool DistanceTracker_Init(void)
{
    s_initialized = true;
    DistanceTracker_Reset();
    return true;
}

void DistanceTracker_Reset(void)
{
    memset(&s_status, 0, sizeof(s_status));

    s_status.initialized = s_initialized;
    s_status.valid = false;

    s_has_source_sequence = false;
}

bool DistanceTracker_Update(
    int16_t left_delta,
    int16_t right_delta,
    uint32_t source_sequence,
    uint32_t timestamp_ms)
{
    if (!s_initialized)
    {
        return false;
    }

    if (s_has_source_sequence &&
        (source_sequence ==
         s_status.source_sequence))
    {
        return false;
    }

    s_status.left_count +=
        (int64_t)left_delta;

    s_status.right_count +=
        (int64_t)right_delta;

    s_status.center_count_x2 +=
        (int64_t)left_delta +
        (int64_t)right_delta;

    s_status.traveled_count_x2 +=
        (uint64_t)DistanceTracker_AbsI16(
            left_delta) +
        (uint64_t)DistanceTracker_AbsI16(
            right_delta);

    DistanceTracker_Recalculate();

    s_status.source_sequence =
        source_sequence;

    s_status.timestamp_ms =
        timestamp_ms;

    s_status.update_count++;
    s_status.valid = true;

    s_has_source_sequence = true;
    return true;
}

bool DistanceTracker_IsInitialized(void)
{
    return s_initialized;
}

bool DistanceTracker_GetStatus(
    DistanceTrackerStatus_t *status)
{
    if ((!s_initialized) ||
        (status == NULL))
    {
        return false;
    }

    *status = s_status;
    return true;
}
