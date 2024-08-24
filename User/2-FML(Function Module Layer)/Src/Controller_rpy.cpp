//
// Created by xuejl on 2024/1/14.
//
#include "Controller_rpy.h"

Class_Controller_rpy::Class_Controller_rpy() {

}

void Class_Controller_rpy::init() {
    for (auto & i : motor) {
        i.Start();
        HAL_Delay(1);
    }
}

void Class_Controller_rpy::Calculate_and_Control() {
    if (groundMode){
        TCP_RPY[0] = 3.14159266f - roll_Zero; //roll
        TCP_RPY[1] = 1.57f; //pitch
        TCP_RPY[2] = 0.0f; //yaw
    }
    else{
        TCP_RPY[0] = - motor[2].Info.pos - roll_Zero + 3.14159266f; //roll
        TCP_RPY[1] = - motor[1].Info.pos; //pitch
        TCP_RPY[2] = - motor[0].Info.pos; //yaw
    }
}

void Class_Controller_rpy::DataSend_to_Motor(uint32_t i){
    motor[i].Set_MIT_PD(Target_Motor_Kp[i], Target_Motor_Kd[i]);
    motor[i].Set_MIT_PVT(0, 0, Target_Motor_Torque[i]);
    motor[i].CTRL_MIT();
}

std::vector<float> Class_Controller_rpy::Get_TCP_RPY() {
    return TCP_RPY;
}

void Class_Controller_rpy::Set_Roll_Zero() {
    roll_Zero = - motor[2].Info.pos;
}

Class_Controller_rpy Controller_rpy;