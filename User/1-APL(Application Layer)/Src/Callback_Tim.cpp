/**
 * @file    Callback_Tim.cpp
 * @brief   TIM回调函数重写
 *
 * @author  xuejl
 * @date    2023-11-18 （创建）
 *
 */

#pragma GCC push_options
#pragma GCC optimize ("O0")

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Tim.h"

/***********************************************************************************************************************
 * @brief TIM更新中断回调函数重写
 *
 * @param htim  TIM外设句柄
 **********************************************************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
        //Timer6: 3000Hz

        static int prescaler = 0;
        if (prescaler++ == 300)
        {
            prescaler = 0;
            Controller_xyz.motor[0].Start();
            Controller_xyz.motor[1].Start();
            Controller_xyz.motor[2].Start();
            Controller_rpy.motor[0].Start();
            Controller_rpy.motor[1].Start();
            Controller_rpy.motor[2].Start();
        }

        static uint32_t motor_i = 0;
        Controller_xyz.DataSend_to_Motor(motor_i % 3);
        Controller_rpy.DataSend_to_Motor(motor_i % 3);
        motor_i ++;

        static int prescaler2 = 0;

        if (prescaler2++ == 120)
        {
            prescaler2 = 0;
            auto tcpxyz = Controller_xyz.Get_TCP_XYZ();
            auto tcprpy = Controller_rpy.Get_TCP_RPY();
            memcpy(Video_Transmission_Link.CustomControlData.data, &tcpxyz[0], 12);
            memcpy(Video_Transmission_Link.CustomControlData.data + 12, &tcprpy[0], 12);
            Video_Transmission_Link.data_transmit_for_custom_control();
        }

        static int prescaler3 = 0;
        if (prescaler3++ == 30)
        {
            prescaler3 = 0;
            Button_PC6.Check_and_Callback();
            Button_PC7_Callback();
            Button_PC8_Callback();
            Button_PC9.Check_and_Callback();
        }
    }
    else if (htim->Instance == htim7.Instance)
    {

    }
}

#pragma GCC pop_options