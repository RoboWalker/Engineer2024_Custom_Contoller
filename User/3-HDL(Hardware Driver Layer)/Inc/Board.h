/**
 * @file    Board.h
 * @brief   板载资源相关
 *
 * @author  Tang-yucheng
 * @date    2023-12-30 （创建）
 *
 */

#ifndef __HDL_BOARD_H
#define __HDL_BOARD_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

/* 类定义 -------------------------------------------------------------------------------------------------------------*/
/**
 * @brief DM01开发板类
 */
class Class_Board_MC01
{
public:
    void Init();
    void Buzzer_Sing(float freq);
    void Buzzer_Stop();
private:

};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern Class_Board_MC01 Board;

#endif  /* HDL_Board.h */
