//
// Created by xuejl on 2024/1/12.
//

#ifndef CUSTOM_CONTROLLER_CONTROLLER_XYZ_H
#define CUSTOM_CONTROLLER_CONTROLLER_XYZ_H

#include "Motor_DM.h"
#include <vector>
#include "Eigen/Dense"

class Class_Controller_xyz {
public:
    Motor_DM motor[3] = {
            Motor_DM(&hcan1, 0x01, 0x01),
            Motor_DM(&hcan1, 0x02, 0x02),
            Motor_DM(&hcan1, 0x03, 0x03)
    };


    Class_Controller_xyz();
    void init();
    void Calculate_and_Control();
    void DataSend_to_Motor(uint32_t i);
    std::vector<float> Get_TCP_XYZ();

private:

    //机械臂参数
    float L_arm = 0.1; // 大臂长度m
    float a_arm = 0.224; // 小臂长度m
    float R = 0.1; // 定平台半径m
    float r = 0.04; // 动平台半径m
    float fai1 = 0;
    float fai2 = 2 * M_PI / 3;
    float fai3 = 4 * M_PI / 3;

    std::vector<float> TCP_XYZ = {0.0, 0.0, 0.0}; //操作手坐标系下的末端坐标
    Eigen::Vector3f Calc_Cord_TCP_XYZ = {0.0, 0.0, 0.0}; //计算坐标系下的末端坐标
    float Target_Motor_Torque[3] = {0.0, 0.0, 0.0};
    float Target_Motor_Kp[3] = {0.0, 0.0, 0.0};
    float Target_Motor_Kd[3] = {0.0, 0.0, 0.0};
    Eigen::Matrix3f J; //Jacobian Matrix

    void Forward_Kinematics();
    void Calculate_Gravity_Compensation();
    void Get_Jacobian_Matrix();
};

extern Class_Controller_xyz Controller_xyz;

#endif //CUSTOM_CONTROLLER_CONTROLLER_XYZ_H
