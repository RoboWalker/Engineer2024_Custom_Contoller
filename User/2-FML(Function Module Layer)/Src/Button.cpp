//
// Created by pocket on 24-1-21.
//
#include "Button.h"
#include "Callback_Button.h"

/**
 * @brief   按键类构造函数
 * @param GPIOx
 * @param GPIO_Pin
 * @param callback
 * @param up_trigger True:上升沿触发 False:下降沿触发
 */
Button_Edgetrigger_Class::Button_Edgetrigger_Class(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, void (*callback)(void), bool up_trigger){
    this->_GPIOx = GPIOx;
    this->_GPIO_Pin = GPIO_Pin;
    this->_callback = callback;
    this->_up_trigger = up_trigger;
}

/**
* @brief   检查按键是否按下(基于有限状态机的边缘检测)，若按下则执行回调函数。10ms调用一次
*/
void Button_Edgetrigger_Class::Check_and_Callback(void){
    switch(g_keyStatus)
    {
        //按键释放(初始状态)
        case KS_RELEASE:
        {
            //检测到低电平，先进行消抖
            if (HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin) == GPIO_PIN_SET)
            {
                g_keyStatus = KS_PRESS_SHAKE;
                g_DebounceCnt = 0;
            }
        }
            break;

            //按下抖动
        case KS_PRESS_SHAKE:
        {
            g_DebounceCnt++;

            //确实是抖动
            if (HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin) == GPIO_PIN_RESET)
            {
                g_keyStatus = KS_RELEASE;
            }
                //消抖完成
            else if (g_DebounceCnt == 5)
            {
                g_keyStatus = KS_PRESS;
                //callback
                _callback();
            }
        }
            break;

            //稳定按下
        case KS_PRESS:
        {
            //检测到高电平，先进行消抖
            if (HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin) == GPIO_PIN_RESET)
            {
                g_keyStatus = KS_RELEASE_SHAKE;
                g_DebounceCnt = 0;
            }
        }
            break;

            //松开抖动
        case KS_RELEASE_SHAKE:
        {
            g_DebounceCnt++;

            //确实是抖动
            if (HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin) == GPIO_PIN_SET)
            {
                g_keyStatus = KS_PRESS;
            }
                //消抖完成
            else if (g_DebounceCnt == 5)
            {
                g_keyStatus = KS_RELEASE;
            }
        }
            break;

        default:break;
    }
}

Button_Edgetrigger_Class Button_PC6(GPIOC, GPIO_PIN_6, Button_PC6_Callback, true);
Button_Edgetrigger_Class Button_PC9(GPIOC, GPIO_PIN_9, Button_PC9_Callback, true);