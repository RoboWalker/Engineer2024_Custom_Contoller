/**
 * @file    Callback_Can.cpp
 * @brief   CAN回调函数重写
 *
 * @author  Tang-yucheng
 * @date    2023-11-18 （创建）
 *
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Callback_Can.h"

/* 函数定义 -----------------------------------------------------------------------------------------------------------*/
/***********************************************************************************************************************
 * @brief CAN RX0中断回调函数重写
 *
 * @param hcan  CAN外设句柄
 **********************************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    if(hcan->Instance == CAN1)
    {
        CAN_Receive_Data(&CAN1_Manage_Object, CAN_FILTER_FIFO0);

        switch(CAN1_Manage_Object.Rx_Buffer.Header.StdId)
        {
            case (0x01):
            {
                Controller_xyz.motor[0].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x02):
            {
                Controller_xyz.motor[1].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x03):
            {
                Controller_xyz.motor[2].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }

        }
    }
    else if(hcan->Instance == CAN2)
    {
        CAN_Receive_Data(&CAN2_Manage_Object, CAN_FILTER_FIFO0);

        switch(CAN2_Manage_Object.Rx_Buffer.Header.StdId)
        {
            case (0x04):
            {
                Controller_rpy.motor[0].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x05):
            {
                Controller_rpy.motor[1].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x06):
            {
                Controller_rpy.motor[2].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                if (Controller_rpy.rollMotorStart)
                {
                    Controller_rpy.Set_Roll_Zero();
                    Controller_rpy.rollMotorStart = false;
                }
                break;
            }
        }
    }
}

/***********************************************************************************************************************
 * @brief CAN RX1中断回调函数重写
 *
 * @param hcan  CAN外设句柄
 **********************************************************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
    if(hcan->Instance == CAN1)
    {
        CAN_Receive_Data(&CAN1_Manage_Object, CAN_FILTER_FIFO1);

        switch(CAN1_Manage_Object.Rx_Buffer.Header.StdId)
        {
            case (0x01):
            {
                Controller_xyz.motor[0].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x02):
            {
                Controller_xyz.motor[1].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x03):
            {
                Controller_xyz.motor[2].Process_CAN_FeedBack(&CAN1_Manage_Object.Rx_Buffer);
                break;
            }
        }
    }
    else if(hcan->Instance == CAN2)
    {
        CAN_Receive_Data(&CAN2_Manage_Object, CAN_FILTER_FIFO1);

        switch(CAN2_Manage_Object.Rx_Buffer.Header.StdId)
        {
            case (0x04):
            {
                Controller_rpy.motor[0].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x05):
            {
                Controller_rpy.motor[1].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                break;
            }
            case (0x06):
            {
                Controller_rpy.motor[2].Process_CAN_FeedBack(&CAN2_Manage_Object.Rx_Buffer);
                break;
            }
        }
    }
}