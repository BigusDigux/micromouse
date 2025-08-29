#include "encoder.h"

volatile int32_t encoder_left_count = 0;
volatile int32_t encoder_right_count = 0;

// ===== Pin defines =====
#define LEFT_ENC_A_PORT GPIOB
#define LEFT_ENC_A_PIN  GPIO_PIN_11
#define LEFT_ENC_B_PORT GPIOB
#define LEFT_ENC_B_PIN  GPIO_PIN_10

#define RIGHT_ENC_A_PORT GPIOA
#define RIGHT_ENC_A_PIN  GPIO_PIN_8
#define RIGHT_ENC_B_PORT GPIOB
#define RIGHT_ENC_B_PIN  GPIO_PIN_15

void ENCODER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Left A pin EXTI
    GPIO_InitStruct.Pin = LEFT_ENC_A_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LEFT_ENC_A_PORT, &GPIO_InitStruct);

    // Left B pin input
    GPIO_InitStruct.Pin = LEFT_ENC_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LEFT_ENC_B_PORT, &GPIO_InitStruct);

    // Right A pin EXTI
    GPIO_InitStruct.Pin = RIGHT_ENC_A_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(RIGHT_ENC_A_PORT, &GPIO_InitStruct);

    // Right B pin input
    GPIO_InitStruct.Pin = RIGHT_ENC_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(RIGHT_ENC_B_PORT, &GPIO_InitStruct);

    // Enable EXTI IRQ
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void ENCODER_ResetLeft(void){
	encoder_left_count = 0;
}

void ENCODER_ResetRight(void){
	encoder_right_count = 0;
}

int32_t ENCODER_GetLeft(void)
{
    return encoder_left_count;
}

int32_t ENCODER_GetRight(void)
{
    return encoder_right_count;
}

// EXTI callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LEFT_ENC_A_PIN)
    {
        if (HAL_GPIO_ReadPin(LEFT_ENC_B_PORT, LEFT_ENC_B_PIN))
            encoder_left_count++;
        else
            encoder_left_count--;
    }
    else if (GPIO_Pin == RIGHT_ENC_A_PIN)
    {
        if (HAL_GPIO_ReadPin(RIGHT_ENC_B_PORT, RIGHT_ENC_B_PIN))
            encoder_right_count++;
        else
            encoder_right_count--;
    }
}
