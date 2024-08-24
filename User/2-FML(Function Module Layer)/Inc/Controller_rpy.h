//
// Created by xuejl on 2024/1/14.
//

#ifndef CUSTOM_CONTROLLER_CONTROLLER_RPY_H
#define CUSTOM_CONTROLLER_CONTROLLER_RPY_H

#include "Motor_DM.h"
#include <vector>

class Class_Controller_rpy {
public:
    Motor_DM motor[3] = {
            Motor_DM(&hcan2, 0x04, 0x04),
            Motor_DM(&hcan2, 0x05, 0x05),
            Motor_DM(&hcan2, 0x06, 0x06)
    };
    Class_Controller_rpy();
    void init();
    void Calculate_and_Control();
    void DataSend_to_Motor(uint32_t i);
    std::vector<float> Get_TCP_RPY();
    void Set_Roll_Zero();
    int groundMode = false; //取地面的矿石模式
    bool rollMotorStart = true; //roll轴电机的启动状态，用于重置roll轴电机的零点

private:
    std::vector<float> TCP_RPY = {0.0, 0.0, 0.0}; //操作手坐标系下的末端rpy
    float Target_Motor_Torque[3] = {0.0, 0.0, 0.0};
    float Target_Motor_Kp[3] = {0.0, 0.0, 0.0};
    float Target_Motor_Kd[3] = {0.0, 0.0, 0.0};// 坐标
    float roll_Zero = 0.0; //roll轴零点偏置，在第一次读取roll轴电机角度时设置
};

extern Class_Controller_rpy Controller_rpy;

#endif //CUSTOM_CONTROLLER_CONTROLLER_RPY_H
