/**
 * @file    Board.cpp
 * @brief   板载资源相关
 *
 * @author  Tang-yucheng
 * @date    2023-12-30 （创建）
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Board.h"

/* 全局变量创建 --------------------------------------------------------------------------------------------------------*/
Class_Board_MC01 Board;

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
void Class_Board_MC01::Init()
{
    /* 使能供电 */
    HAL_GPIO_WritePin(GPIOC, Power_OUT1_EN_Pin | Power_OUT2_EN_Pin | Power_5V_EN_Pin, GPIO_PIN_SET);

    /* 配置RS485引脚 */
    HAL_GPIO_WritePin(GPIOC, RS485_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, RS485_DIR2_Pin, GPIO_PIN_RESET);

    /* 蜂鸣器初始化 */
    __HAL_TIM_SET_PRESCALER(&htim2, 84 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Class_Board_MC01::Buzzer_Sing(float freq)
{
    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(1000000 / freq));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(1000000 / freq * 0.5f));
}

void Class_Board_MC01::Buzzer_Stop()
{
    /* PWM输出置零 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}