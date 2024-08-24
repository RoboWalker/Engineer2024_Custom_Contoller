//
// Created by pocket on 24-1-21.
//

#ifndef CUSTOM_CONTROLLER_BUTTON_H
#define CUSTOM_CONTROLLER_BUTTON_H

#include "main.h"

#define ENUM_ITEM(ITEM) ITEM,

#define KEY_STATUS_ENUM(STATUS)                   \
	STATUS(KS_RELEASE)       /*稳定松开状态*/       \
	STATUS(KS_PRESS_SHAKE)   /*按下抖动状态*/       \
	STATUS(KS_PRESS)         /*稳定按下状态*/       \
	STATUS(KS_RELEASE_SHAKE) /*松开抖动状态*/       \
	STATUS(KS_NUM)           /*状态总数(无效状态)*/  \

typedef enum
{
    KEY_STATUS_ENUM(ENUM_ITEM)
}KEY_STATUS;

class Button_Edgetrigger_Class{
public:
    Button_Edgetrigger_Class(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, void (*callback)(void), bool up_trigger);
    void Check_and_Callback(void);

private:
    GPIO_TypeDef* _GPIOx;
    uint32_t _GPIO_Pin;
    void (*_callback)(void);
    bool _up_trigger;
    KEY_STATUS g_keyStatus = KS_RELEASE; //当前按键的状态
    KEY_STATUS g_lastKeyStatus = KS_NUM; //上一状态
    int g_DebounceCnt = 0; //消抖时间计数
};

extern Button_Edgetrigger_Class Button_PC6;
extern Button_Edgetrigger_Class Button_PC9;

#endif //CUSTOM_CONTROLLER_BUTTON_H
