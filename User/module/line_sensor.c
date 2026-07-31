#include "line_sensor.h"

#include "line_sensor_config.h"

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
#include "bsp_line_adc.h"
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
#include "bsp_line_uart.h"
#else
#error "The selected line sensor backend is not implemented"
#endif

#include <stddef.h>
#include <string.h>

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
static const uint8_t s_channel_map[LINE_SENSOR_COUNT] =
    LINE_SENSOR_ADC_CHANNEL_MAP_INIT;
static const uint16_t s_default_white_raw[LINE_SENSOR_COUNT] =
    LINE_SENSOR_WHITE_RAW_INIT;
static const uint16_t s_default_black_raw[LINE_SENSOR_COUNT] =
    LINE_SENSOR_BLACK_RAW_INIT;
static uint16_t s_white_raw[LINE_SENSOR_COUNT];
static uint16_t s_black_raw[LINE_SENSOR_COUNT];
#else
static const uint8_t s_channel_map[LINE_SENSOR_COUNT] =
    LINE_SENSOR_UART_CHANNEL_MAP_INIT;
#endif

static LineSensorFrame_t s_frame;
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
static uint8_t s_previous_black_mask = 0U;
#endif
static uint32_t s_last_bsp_sequence = 0U;
static bool s_initialized = false;
static bool s_running = false;
static bool s_has_frame = false;

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
static uint16_t LineSensor_AbsDiff(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}
#endif

static bool LineSensor_ChannelMapIsValid(void)
{
    uint8_t seen_mask = 0U;
    uint8_t logical;

    for (logical = 0U; logical < LINE_SENSOR_COUNT; logical++)
    {
        uint8_t physical = s_channel_map[logical];
        uint8_t bit;

        if (physical >= LINE_SENSOR_COUNT)
        {
            return false;
        }

        bit = (uint8_t)(1U << physical);
        if ((seen_mask & bit) != 0U)
        {
            return false;
        }
        seen_mask |= bit;
    }

    return seen_mask == 0xFFU;
}

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
static uint16_t LineSensor_Normalize(uint16_t raw,
                                     uint16_t white_raw,
                                     uint16_t black_raw)
{
    int32_t numerator;
    int32_t denominator;
    int32_t result;

    denominator = (int32_t)black_raw - (int32_t)white_raw;
    if (denominator == 0)
    {
        return 0U;
    }

    numerator = ((int32_t)raw - (int32_t)white_raw) *
                (int32_t)LINE_SENSOR_STRENGTH_MAX;
    result = numerator / denominator;

    if (result < 0)
    {
        result = 0;
    }
    else if (result > (int32_t)LINE_SENSOR_STRENGTH_MAX)
    {
        result = (int32_t)LINE_SENSOR_STRENGTH_MAX;
    }

    return (uint16_t)result;
}
#endif

bool LineSensor_Init(void)
{
    s_initialized = false;
    s_running = false;
    s_has_frame = false;
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    s_previous_black_mask = 0U;
#endif
    s_last_bsp_sequence = 0U;
    memset(&s_frame, 0, sizeof(s_frame));

    if (!LineSensor_ChannelMapIsValid())
    {
        return false;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    LineSensor_ResetCalibration();
    if (!BSP_LineAdc_Init())
    {
        return false;
    }
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    if (!BSP_LineUart_Init())
    {
        return false;
    }
#endif

    s_initialized = true;
    if (!LineSensor_Start())
    {
        s_initialized = false;
        return false;
    }

    return true;
}

bool LineSensor_Start(void)
{
    if (!s_initialized)
    {
        return false;
    }

    if (s_running)
    {
        return true;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    if (!BSP_LineAdc_Start())
    {
        return false;
    }
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    if (!BSP_LineUart_IsInitialized())
    {
        /*
         * BSP_LineUart_Stop() intentionally aborts UART activity and marks
         * the backend uninitialized.  A task reset performs that safe stop
         * before the next start, so restore the backend here to make the
         * LineSensor Stop -> Start lifecycle restartable.
         */
        if (!BSP_LineUart_Init())
        {
            return false;
        }
    }
#endif

    s_running = true;
    return true;
}

bool LineSensor_Stop(void)
{
    bool result = true;

    if (!s_initialized)
    {
        return false;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    result = BSP_LineAdc_Stop();
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    BSP_LineUart_Stop();
#endif

    s_running = false;
    s_has_frame = false;
    return result;
}

bool LineSensor_Update(void)
{
    LineSensorFrame_t next_frame;
    uint8_t next_black_mask = 0U;
    uint8_t logical;

    if (!s_initialized || !s_running)
    {
        return false;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    {
        BspLineAdcSnapshot_t snapshot;

        if (!BSP_LineAdc_GetSnapshot(&snapshot))
        {
            return false;
        }

        if (snapshot.sequence == s_last_bsp_sequence)
        {
            return false;
        }

        memset(&next_frame, 0, sizeof(next_frame));
        next_frame.sequence = snapshot.sequence;
        next_frame.timestamp_ms = snapshot.timestamp_ms;

        for (logical = 0U; logical < LINE_SENSOR_COUNT; logical++)
        {
            uint8_t physical = s_channel_map[logical];
            uint8_t bit = (uint8_t)(1U << logical);
            uint16_t strength;
            bool was_black;
            bool is_black;

            next_frame.raw[logical] = snapshot.raw[physical];

            if (LineSensor_AbsDiff(s_white_raw[logical],
                                   s_black_raw[logical]) <
                LINE_SENSOR_MIN_CALIBRATION_SPAN)
            {
                continue;
            }

            next_frame.valid_mask |= bit;
            strength = LineSensor_Normalize(next_frame.raw[logical],
                                             s_white_raw[logical],
                                             s_black_raw[logical]);
            next_frame.strength[logical] = strength;

            was_black = (s_previous_black_mask & bit) != 0U;
            is_black = was_black;
            if (was_black)
            {
                if (strength <= LINE_SENSOR_WHITE_ON_THRESHOLD)
                {
                    is_black = false;
                }
            }
            else if (strength >= LINE_SENSOR_BLACK_ON_THRESHOLD)
            {
                is_black = true;
            }

            if (is_black)
            {
                next_black_mask |= bit;
            }
        }

        next_frame.black_mask = next_black_mask;
        s_previous_black_mask = next_black_mask;
        s_last_bsp_sequence = snapshot.sequence;
    }
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    {
        BspLineUartSnapshot_t snapshot;

        BSP_LineUart_Process();
        if (!BSP_LineUart_GetSnapshot(&snapshot))
        {
            return false;
        }

        if (snapshot.sequence == 0U)
        {
            return false;
        }

        if (snapshot.sequence == s_last_bsp_sequence)
        {
            /* 数据由有效变为超时时，发布一次 INVALID 帧。 */
            if (s_has_frame && !snapshot.data_valid &&
                (s_frame.valid_mask != 0U))
            {
                s_frame.valid_mask = 0U;
                s_frame.black_mask = 0U;
                memset(s_frame.strength, 0, sizeof(s_frame.strength));
                return true;
            }
            return false;
        }

        memset(&next_frame, 0, sizeof(next_frame));
        next_frame.sequence = snapshot.sequence;
        next_frame.timestamp_ms = snapshot.timestamp_ms;

        for (logical = 0U; logical < LINE_SENSOR_COUNT; logical++)
        {
            uint8_t physical = s_channel_map[logical];
            uint8_t logical_bit = (uint8_t)(1U << logical);
            uint16_t raw_bit =
                (uint16_t)((snapshot.raw_state >> physical) & 0x01U);

            next_frame.raw[logical] = raw_bit;

            if ((LINE_SENSOR_UART_CONFIG_CONFIRMED == 0U) ||
                !snapshot.data_valid)
            {
                continue;
            }

            next_frame.valid_mask |= logical_bit;
            if (raw_bit == (uint16_t)LINE_SENSOR_UART_BLACK_LEVEL)
            {
                next_frame.strength[logical] = LINE_SENSOR_STRENGTH_MAX;
                next_black_mask |= logical_bit;
            }
        }

        next_frame.black_mask = next_black_mask;
        s_last_bsp_sequence = snapshot.sequence;
    }
#endif

    s_frame = next_frame;
    s_has_frame = true;
    return true;
}

bool LineSensor_IsInitialized(void)
{
    return s_initialized;
}

bool LineSensor_IsRunning(void)
{
    return s_initialized && s_running;
}

bool LineSensor_IsBackendConfigConfirmed(void)
{
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    return LINE_SENSOR_UART_CONFIG_CONFIRMED != 0U;
#else
    return true;
#endif
}

const char *LineSensor_GetBackendName(void)
{
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    return "ADC8";
#elif LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_UART8
    return "UART8";
#else
    return "UNKNOWN";
#endif
}

bool LineSensor_GetFrame(LineSensorFrame_t *frame)
{
    if ((frame == NULL) || !s_has_frame)
    {
        return false;
    }

    *frame = s_frame;
    return true;
}

bool LineSensor_SetCalibration(uint8_t logical_channel,
                               uint16_t white_raw,
                               uint16_t black_raw)
{
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    if ((logical_channel >= LINE_SENSOR_COUNT) ||
        (LineSensor_AbsDiff(white_raw, black_raw) <
         LINE_SENSOR_MIN_CALIBRATION_SPAN))
    {
        return false;
    }

    s_white_raw[logical_channel] = white_raw;
    s_black_raw[logical_channel] = black_raw;
    return true;
#else
    (void)logical_channel;
    (void)white_raw;
    (void)black_raw;
    return false;
#endif
}

bool LineSensor_GetCalibration(uint8_t logical_channel,
                               uint16_t *white_raw,
                               uint16_t *black_raw)
{
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    if ((logical_channel >= LINE_SENSOR_COUNT) ||
        (white_raw == NULL) || (black_raw == NULL))
    {
        return false;
    }

    *white_raw = s_white_raw[logical_channel];
    *black_raw = s_black_raw[logical_channel];
    return true;
#else
    (void)logical_channel;
    (void)white_raw;
    (void)black_raw;
    return false;
#endif
}

void LineSensor_ResetCalibration(void)
{
#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    memcpy(s_white_raw, s_default_white_raw, sizeof(s_white_raw));
    memcpy(s_black_raw, s_default_black_raw, sizeof(s_black_raw));
    s_previous_black_mask = 0U;
#endif
}
