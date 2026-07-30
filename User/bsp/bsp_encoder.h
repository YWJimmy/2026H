#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief MG513X 车轮旋转一圈对应的硬件编码器计数。
 *
 * 当前定义为 TIM Encoder Mode（TI1 and TI2，四倍频解码）后的计数值。
 * 上层计算距离时直接使用 1468，不要再次乘以 4。
 */
#define BSP_ENCODER_COUNTS_PER_REV    ((int32_t)1468)

/**
 * @brief 左右编码器的一次同步采样快照。
 *
 * 每次调用 BSP_Encoder_Sample() 时，同时读取左右硬件计数器，
 * 更新左右累计值，并生成一份不可被其他调用者“消费”的统一快照。
 */
typedef struct
{
    int16_t left_delta;
    int16_t right_delta;

    int32_t left_total;
    int32_t right_total;

    uint32_t sequence;
    uint32_t timestamp_ms;
} BspEncoderSample_t;

/**
 * @brief 初始化左右编码器 BSP。
 *
 * 左编码器：TIM3_CH1/CH2，PC6/PC7。
 * 右编码器：TIM4_CH1/CH2，PD12/PD13。
 *
 * 调用前必须完成 MX_TIM3_Init() 和 MX_TIM4_Init()。
 */
bool BSP_Encoder_Init(void);

/**
 * @brief 查询编码器 BSP 是否已经初始化成功。
 */
bool BSP_Encoder_IsInitialized(void);

/**
 * @brief 清零硬件计数器、增量基准、累计值和快照序号。
 */
void BSP_Encoder_Reset(void);

/**
 * @brief 同时读取左右编码器并生成一份统一快照。
 *
 * 底盘闭环、里程统计和调试输出应共享本函数生成的同一份快照，
 * 不应在同一控制周期内分别调用左右增量接口。
 *
 * @param sample 输出快照。
 * @return true 采样成功。
 * @return false BSP未初始化或参数为空。
 */
bool BSP_Encoder_Sample(BspEncoderSample_t *sample);

/**
 * @brief 获取最近一次 BSP_Encoder_Sample() 生成的快照。
 *
 * 本函数只复制缓存，不读取硬件计数器，也不改变增量基准。
 */
bool BSP_Encoder_GetLatestSample(BspEncoderSample_t *sample);

/**
 * @brief 兼容旧测试：读取左侧自上次左侧读取以来的增量。
 *
 * @warning 不要与 BSP_Encoder_Sample() 混合用于正式控制。
 */
int16_t BSP_Encoder_GetLeftDelta(void);

/**
 * @brief 兼容旧测试：读取右侧自上次右侧读取以来的增量。
 *
 * @warning 不要与 BSP_Encoder_Sample() 混合用于正式控制。
 */
int16_t BSP_Encoder_GetRightDelta(void);

/**
 * @brief 获取左编码器软件累计计数。
 */
int32_t BSP_Encoder_GetLeftTotal(void);

/**
 * @brief 获取右编码器软件累计计数。
 */
int32_t BSP_Encoder_GetRightTotal(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
