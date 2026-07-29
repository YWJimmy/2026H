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
 * 因此上层计算距离时直接使用 1468，不要再次乘以 4。
 */
#define BSP_ENCODER_COUNTS_PER_REV    ((int32_t)1468)

/**
 * @brief 初始化左右编码器 BSP。
 *
 * 左编码器：TIM3_CH1/CH2，PC6/PC7。
 * 右编码器：TIM4_CH1/CH2，PD12/PD13。
 *
 * 调用前必须完成 MX_TIM3_Init() 和 MX_TIM4_Init()。
 * 初始化成功后，两个硬件编码器计数器均从 0 开始运行。
 *
 * @return true  TIM3、TIM4 均启动成功。
 * @return false 定时器配置不符合要求，或任一路启动失败。
 */
bool BSP_Encoder_Init(void);

/**
 * @brief 查询编码器 BSP 是否已经初始化成功。
 */
bool BSP_Encoder_IsInitialized(void);

/**
 * @brief 清零左右硬件计数器、增量基准和软件累计计数。
 */
void BSP_Encoder_Reset(void);

/**
 * @brief 读取左编码器自上次调用以来的增量。
 *
 * 返回值已处理 16 位计数器回绕和方向修正。
 * 正值定义为对应车轮驱动小车向前。
 * 调用本函数时会同步更新左侧软件累计计数。
 *
 * @return 本次增量；BSP 未初始化时返回 0。
 */
int16_t BSP_Encoder_GetLeftDelta(void);

/**
 * @brief 读取右编码器自上次调用以来的增量。
 *
 * 规则同 BSP_Encoder_GetLeftDelta()。
 */
int16_t BSP_Encoder_GetRightDelta(void);

/**
 * @brief 获取左编码器的软件累计计数。
 *
 * 累计值仅在调用 BSP_Encoder_GetLeftDelta() 时更新。
 */
int32_t BSP_Encoder_GetLeftTotal(void);

/**
 * @brief 获取右编码器的软件累计计数。
 *
 * 累计值仅在调用 BSP_Encoder_GetRightDelta() 时更新。
 */
int32_t BSP_Encoder_GetRightTotal(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
