/**
 * @file    Callback_Usart.cpp
 * @brief   USART回调函数重写
 *
 * @author  Tang-yucheng
 * @date    2023-12-2 （创建）
 *
 */

#pragma GCC push_options
#pragma GCC optimize ("O0")

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Usart.h"

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
/***********************************************************************************************************************
 * @brief UART-RX事件中断回调函数重写
 *
 * @param huart  UART外设句柄
 * @param Size   当前接收长度
 **********************************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if (huart->Instance == huart3.Instance)
    {
        /* 开启新一次串口接收（DMA-IDLE） */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Data_Size);
    }
    else if (huart->Instance == huart4.Instance)
    {
        /* IMU数据解析 */
        IMU_Gimbal.DataGet(Size);

        /* 开启新一次串口接收（DMA-IDLE） */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Data_Size);
    }
}

/***********************************************************************************************************************
 * @brief UART-错误回调函数重写
 *
 * @param huart  UART外设句柄
 **********************************************************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (HAL_UART_GetError(huart) & HAL_UART_ERROR_PE){        /*!< Parity error            */
        //奇偶校验错误
        __HAL_UART_CLEAR_PEFLAG(huart);
    } else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_NE) { /*!< Noise error             */
        //噪声错误
        __HAL_UART_CLEAR_NEFLAG(huart);
    } else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_FE) { /*!< Frame error             */
        //帧格式错误
        __HAL_UART_CLEAR_FEFLAG(huart);
    } else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) { /*!< Overrun error           */
        //数据太多串口来不及接收错
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    //当这个串口发生了错误，一定要在重新使能接收中断
    if (huart->Instance == huart3.Instance)
    {
        /* 开启新一次串口接收（DMA-IDLE） */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Data_Size);
    }
    else if (huart->Instance == huart4.Instance)
    {
        /* 开启新一次串口接收（DMA-IDLE） */
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Data_Size);
    }
}

/***********************************************************************************************************************
 * @brief UART-USB回调函数重写
 *
 * @param Size   当前接收长度
 **********************************************************************************************************************/
void UART_USB_RxEventCallback(uint16_t Size)
{
//    Sentry_Chassis.DataGet(UART_USB_RX_Buffer, Size);
}

#pragma GCC pop_options