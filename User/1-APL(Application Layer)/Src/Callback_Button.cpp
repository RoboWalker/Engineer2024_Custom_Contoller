//
// Created by pocket on 24-1-21.
//

#include "Callback_Button.h"

void Button_PC6_Callback(void){
    //将data[24]的最低位取反
    Video_Transmission_Link.CustomControlData.data[24] ^= 0x01;
}

void Button_PC7_Callback(void){
    //将按键2状态写入data[24].bit1
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET)
    {
        Video_Transmission_Link.CustomControlData.data[24] |= (0x01 << 1);
    }
    else
    {
        Video_Transmission_Link.CustomControlData.data[24] &= ~(0x01 << 1);
    }
}

void Button_PC8_Callback(void){
    //将按键2状态写入data[24].bit1
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        Video_Transmission_Link.CustomControlData.data[24] |= (0x01 << 2);
    }
    else
    {
        Video_Transmission_Link.CustomControlData.data[24] &= ~(0x01 << 2);
    }
}

void Button_PC9_Callback(void){
    Controller_rpy.groundMode = !Controller_rpy.groundMode;
}