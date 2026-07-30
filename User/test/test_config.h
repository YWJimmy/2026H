#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#define TEST_MODE_LINE_ADC                  1U
#define TEST_MODE_LINE_SENSOR               2U
#define TEST_MODE_LINE_FOLLOW               3U
#define TEST_MODE_MOTOR_OPEN_LOOP           4U

/*
 * 保留历史TEST_MODE_WHEEL_SPEED名称，并将其明确作为mm/s模式别名。
 */
#define TEST_MODE_WHEEL_SPEED_MMPS          5U
#define TEST_MODE_WHEEL_SPEED               TEST_MODE_WHEEL_SPEED_MMPS

#define TEST_MODE_CHASSIS_STRAIGHT          6U
#define TEST_MODE_ENCODER                   7U
#define TEST_MODE_LINE_UART                 8U
#define TEST_MODE_VISION_UART               9U
#define TEST_MODE_WHEEL_SPEED_CPS           10U

/*
 * 当前继续进行轮速PI闭环V2的mm/s阶跃测试。
 *
 * 常用切换：
 * TEST_MODE_LINE_FOLLOW       UART4八路巡线分析/闭环测试
 * TEST_MODE_LINE_UART         UART4八路巡线原始通信测试
 * TEST_MODE_VISION_UART       USART6 K230D视觉通信测试
 * TEST_MODE_WHEEL_SPEED_CPS   count/s单位轮速闭环测试
 */
#define PROJECT_TEST_MODE                   TEST_MODE_WHEEL_SPEED_MMPS

#endif /* TEST_CONFIG_H */
