#ifndef DISTANCE_TRACKER_H
#define DISTANCE_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * 路程统计直接使用当前底盘理论参数：
 * - 编码器：1468 count/rev
 * - 轮径：65 mm
 * - 轮周长：204204 um
 *
 * 本版本不增加实车比例标定。
 */
typedef struct
{
    bool initialized;
    bool valid;

    /* 原始累计计数。 */
    int64_t left_count;
    int64_t right_count;
    int64_t center_count_x2;
    uint64_t traveled_count_x2;

    /* 左右轮有符号距离。 */
    int64_t left_signed_um;
    int64_t right_signed_um;
    int32_t left_signed_mm;
    int32_t right_signed_mm;

    /*
     * 车辆中心有符号路程：
     * 前进为正，后退为负。
     */
    int64_t center_signed_um;
    int32_t center_signed_mm;

    /*
     * 总累计路程：
     * 左右轮绝对增量取平均，前进和后退均累计。
     */
    uint64_t traveled_um;
    uint32_t traveled_mm;

    /* 左右轮距离差，便于判断跑偏。 */
    int32_t wheel_difference_mm;

    uint32_t source_sequence;
    uint32_t update_count;
    uint32_t timestamp_ms;
} DistanceTrackerStatus_t;

bool DistanceTracker_Init(void);

/**
 * @brief 清零软件路程，不清零硬件编码器和底盘PI。
 */
void DistanceTracker_Reset(void);

/**
 * @brief 使用底盘已经采纳的同一帧编码器增量更新路程。
 *
 * 不允许为了路程统计再次调用BSP_Encoder_Sample()。
 */
bool DistanceTracker_Update(
    int16_t left_delta,
    int16_t right_delta,
    uint32_t source_sequence,
    uint32_t timestamp_ms);

bool DistanceTracker_IsInitialized(void);
bool DistanceTracker_GetStatus(
    DistanceTrackerStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* DISTANCE_TRACKER_H */
