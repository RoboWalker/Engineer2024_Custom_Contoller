/**
 * @file    User_Main.cpp
 * @brief   初始化，主函数实现
 *
 * @author  Tang-yucheng
 * @date    2023-11-17 （创建）
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#pragma GCC push_options
#pragma GCC optimize ("O0")
#include "User_Main.h"
#include "Board.h"
#include "IMU.h"
#include "tim.h"
#include "Controller_xyz.h"
#include "Controller_rpy.h"
#include "Button.h"
#include "Callback_Button.h"


void User_setup(void)
{
    /* 开发板初始化 */
    Board.Init();

    /* 使能CAN外设 */
    CAN_Init(&CAN1_Manage_Object);
    CAN_Init(&CAN2_Manage_Object);

    /* 蜂鸣器启动，提示初始化完成 */
    Board.Buzzer_Sing(700.0f);
    HAL_Delay(100);
    Board.Buzzer_Stop();
    HAL_Delay(100);
    Board.Buzzer_Sing(900.0f);
    HAL_Delay(100);
    Board.Buzzer_Stop();
    HAL_Delay(100);
    Board.Buzzer_Sing(1700.0f);
    HAL_Delay(100);
    Board.Buzzer_Stop();
    HAL_Delay(100);
    Board.Buzzer_Sing(900.0f);
    HAL_Delay(100);
    Board.Buzzer_Stop();
    HAL_Delay(100);
    Board.Buzzer_Sing(700.0f);
    HAL_Delay(100);
    Board.Buzzer_Stop();
    HAL_Delay(100);

    /* xyz控制器初始化 */
    Controller_xyz.init();
    Controller_rpy.init();

    /* 使能系统心跳定时器 */
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
}


void User_loop(void)
{
    Controller_xyz.Calculate_and_Control();
    Controller_rpy.Calculate_and_Control();
}

#pragma GCC pop_options