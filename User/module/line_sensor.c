#include "line_sensor.h"

#include "line_sensor_config.h"

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
#include "bsp_line_adc.h"
#else
#error "The selected line sensor backend is not implemented"
#endif

#include <string.h>

static const uint8_t s_channel_map[LINE_SENSOR_COUNT] =
    LINE_SENSOR_CHANNEL_MAP_INIT;
static const uint16_t s_default_white_raw[LINE_SENSOR_COUNT] =
    LINE_SENSOR_WHITE_RAW_INIT;
static const uint16_t s_default_black_raw[LINE_SENSOR_COUNT] =
    LINE_SENSOR_BLACK_RAW_INIT;

static uint16_t s_white_raw[LINE_SENSOR_COUNT];
static uint16_t s_black_raw[LINE_SENSOR_COUNT];
static LineSensorFrame_t s_frame;
static uint8_t s_previous_black_mask = 0U;
static uint32_t s_last_bsp_sequence = 0U;
static bool s_initialized = false;
static bool s_running = false;
static bool s_has_frame = false;

static uint16_t LineSensor_AbsDiff(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

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

bool LineSensor_Init(void)
{
    s_initialized = false;
    s_running = false;
    s_has_frame = false;
    s_previous_black_mask = 0U;
    s_last_bsp_sequence = 0U;
    memset(&s_frame, 0, sizeof(s_frame));

    if (!LineSensor_ChannelMapIsValid())
    {
        return false;
    }

    LineSensor_ResetCalibration();

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    if (!BSP_LineAdc_Init())
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
#endif

    s_running = true;
    return true;
}

bool LineSensor_Stop(void)
{
    bool result;

    if (!s_initialized)
    {
        return false;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    result = BSP_LineAdc_Stop();
#else
    result = false;
#endif

    s_running = false;
    s_has_frame = false;
    return result;
}

bool LineSensor_Update(void)
{
    BspLineAdcSnapshot_t snapshot;
    LineSensorFrame_t next_frame;
    uint8_t next_black_mask = 0U;
    uint8_t logical;

    if (!s_initialized || !s_running)
    {
        return false;
    }

#if LINE_SENSOR_BACKEND == LINE_SENSOR_BACKEND_ADC8
    if (!BSP_LineAdc_GetSnapshot(&snapshot))
    {
        return false;
    }
#else
    return false;
#endif

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
    if ((logical_channel >= LINE_SENSOR_COUNT) ||
        (LineSensor_AbsDiff(white_raw, black_raw) <
         LINE_SENSOR_MIN_CALIBRATION_SPAN))
    {
        return false;
    }

    s_white_raw[logical_channel] = white_raw;
    s_black_raw[logical_channel] = black_raw;
    return true;
}

bool LineSensor_GetCalibration(uint8_t logical_channel,
                               uint16_t *white_raw,
                               uint16_t *black_raw)
{
    if ((logical_channel >= LINE_SENSOR_COUNT) ||
        (white_raw == NULL) || (black_raw == NULL))
    {
        return false;
    }

    *white_raw = s_white_raw[logical_channel];
    *black_raw = s_black_raw[logical_channel];
    return true;
}

void LineSensor_ResetCalibration(void)
{
    memcpy(s_white_raw, s_default_white_raw, sizeof(s_white_raw));
    memcpy(s_black_raw, s_default_black_raw, sizeof(s_black_raw));
    s_previous_black_mask = 0U;
}
