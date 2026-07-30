#include "bsp_line_adc.h"

#include "adc.h"
#include "main.h"
#include "tim.h"

#include <string.h>

#define BSP_LINE_ADC_TIM_PRESCALER       83U
#define BSP_LINE_ADC_TIM_PERIOD          999U

static uint16_t s_dma_buffer[BSP_LINE_ADC_CHANNEL_COUNT];
static uint16_t s_latest_raw[BSP_LINE_ADC_CHANNEL_COUNT];

static volatile bool s_initialized = false;
static volatile bool s_running = false;
static volatile bool s_fault = false;
static volatile uint32_t s_start_ms = 0U;
static volatile uint32_t s_sequence = 0U;
static volatile uint32_t s_last_frame_ms = 0U;
static volatile uint32_t s_error_count = 0U;

static uint32_t LineAdc_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void LineAdc_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool LineAdc_ConfigIsValid(void)
{
    if ((hadc1.Instance != ADC1) ||
        (hadc1.DMA_Handle == NULL) ||
        (htim2.Instance != TIM2))
    {
        return false;
    }

    if ((hadc1.Init.ScanConvMode != ENABLE) ||
        (hadc1.Init.ContinuousConvMode != DISABLE) ||
        (hadc1.Init.ExternalTrigConvEdge != ADC_EXTERNALTRIGCONVEDGE_RISING) ||
        (hadc1.Init.ExternalTrigConv != ADC_EXTERNALTRIGCONV_T2_TRGO) ||
        (hadc1.Init.NbrOfConversion != BSP_LINE_ADC_CHANNEL_COUNT) ||
        (hadc1.Init.DMAContinuousRequests != ENABLE))
    {
        return false;
    }

    if ((hadc1.DMA_Handle->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (hadc1.DMA_Handle->Init.MemInc != DMA_MINC_ENABLE) ||
        (hadc1.DMA_Handle->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) ||
        (hadc1.DMA_Handle->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD) ||
        (hadc1.DMA_Handle->Init.Mode != DMA_CIRCULAR))
    {
        return false;
    }

    if ((htim2.Init.Prescaler != BSP_LINE_ADC_TIM_PRESCALER) ||
        (htim2.Init.Period != BSP_LINE_ADC_TIM_PERIOD))
    {
        return false;
    }

    return true;
}

bool BSP_LineAdc_Init(void)
{
    uint32_t primask;

    HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_RESET);

    primask = LineAdc_EnterCritical();
    memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
    memset(s_latest_raw, 0, sizeof(s_latest_raw));
    s_initialized = false;
    s_running = false;
    s_fault = false;
    s_start_ms = 0U;
    s_sequence = 0U;
    s_last_frame_ms = 0U;
    s_error_count = 0U;
    LineAdc_ExitCritical(primask);

    if (!LineAdc_ConfigIsValid())
    {
        return false;
    }

    s_initialized = true;
    return true;
}

bool BSP_LineAdc_Start(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask;

    if (!s_initialized)
    {
        return false;
    }

    if (s_running)
    {
        return true;
    }

    if (s_fault)
    {
        (void)HAL_TIM_Base_Stop(&htim2);
        (void)HAL_ADC_Stop_DMA(&hadc1);
    }

    primask = LineAdc_EnterCritical();
    memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
    memset(s_latest_raw, 0, sizeof(s_latest_raw));
    s_sequence = 0U;
    s_last_frame_ms = 0U;
    s_start_ms = HAL_GetTick();
    s_fault = false;
    LineAdc_ExitCritical(primask);

    HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_SET);

    status = HAL_ADC_Start_DMA(&hadc1,
                               (uint32_t *)s_dma_buffer,
                               BSP_LINE_ADC_CHANNEL_COUNT);
    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_RESET);
        s_error_count++;
        return false;
    }

    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

    s_running = true;
    status = HAL_TIM_Base_Start(&htim2);
    if (status != HAL_OK)
    {
        s_running = false;
        (void)HAL_ADC_Stop_DMA(&hadc1);
        HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_RESET);
        s_error_count++;
        return false;
    }

    return true;
}

bool BSP_LineAdc_Stop(void)
{
    HAL_StatusTypeDef tim_status = HAL_OK;
    HAL_StatusTypeDef adc_status = HAL_OK;
    bool was_active;

    if (!s_initialized)
    {
        return false;
    }

    was_active = s_running || s_fault;
    s_running = false;

    if (was_active)
    {
        tim_status = HAL_TIM_Base_Stop(&htim2);
        adc_status = HAL_ADC_Stop_DMA(&hadc1);
    }

    HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_RESET);
    s_fault = false;

    return (tim_status == HAL_OK) && (adc_status == HAL_OK);
}

bool BSP_LineAdc_IsInitialized(void)
{
    return s_initialized;
}

bool BSP_LineAdc_IsRunning(void)
{
    return s_initialized && s_running && !s_fault;
}

bool BSP_LineAdc_IsDataValid(void)
{
    uint32_t now_ms;

    if (!BSP_LineAdc_IsRunning() || (s_sequence == 0U))
    {
        return false;
    }

    now_ms = HAL_GetTick();
    return ((uint32_t)(now_ms - s_start_ms) >= BSP_LINE_ADC_STABILIZE_MS);
}

bool BSP_LineAdc_GetRaw(uint16_t raw[BSP_LINE_ADC_CHANNEL_COUNT])
{
    BspLineAdcSnapshot_t snapshot;

    if ((raw == NULL) || !BSP_LineAdc_GetSnapshot(&snapshot))
    {
        return false;
    }

    memcpy(raw, snapshot.raw, sizeof(snapshot.raw));
    return true;
}

bool BSP_LineAdc_GetChannel(uint8_t channel, uint16_t *raw)
{
    BspLineAdcSnapshot_t snapshot;

    if ((raw == NULL) || (channel >= BSP_LINE_ADC_CHANNEL_COUNT) ||
        !BSP_LineAdc_GetSnapshot(&snapshot))
    {
        return false;
    }

    *raw = snapshot.raw[channel];
    return true;
}

bool BSP_LineAdc_GetSnapshot(BspLineAdcSnapshot_t *snapshot)
{
    uint32_t primask;

    if ((snapshot == NULL) || !BSP_LineAdc_IsDataValid())
    {
        return false;
    }

    primask = LineAdc_EnterCritical();
    memcpy(snapshot->raw, s_latest_raw, sizeof(snapshot->raw));
    snapshot->sequence = s_sequence;
    snapshot->timestamp_ms = s_last_frame_ms;
    LineAdc_ExitCritical(primask);

    return true;
}

uint32_t BSP_LineAdc_GetSequence(void)
{
    return s_sequence;
}

uint32_t BSP_LineAdc_GetErrorCount(void)
{
    return s_error_count;
}

void BSP_LineAdc_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    if ((!s_initialized) || (!s_running) || (hadc != &hadc1))
    {
        return;
    }

    for (i = 0U; i < BSP_LINE_ADC_CHANNEL_COUNT; i++)
    {
        s_latest_raw[i] = s_dma_buffer[i];
    }

    s_last_frame_ms = HAL_GetTick();
    s_sequence++;
}

void BSP_LineAdc_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((!s_initialized) || (hadc != &hadc1))
    {
        return;
    }

    s_error_count++;
    s_fault = true;
    s_running = false;

    __HAL_TIM_DISABLE(&htim2);
    HAL_GPIO_WritePin(LINE_EN_GPIO_Port, LINE_EN_Pin, GPIO_PIN_RESET);
}
