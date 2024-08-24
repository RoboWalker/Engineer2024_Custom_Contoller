/**
 * @file    Controller_xyz
 * @brief   自定义控制器xyz方向控制器
 *
 * @author  xuejl
 * @date    2024-1-12 （创建）
 *
 */

#include "Controller_xyz.h"

Class_Controller_xyz Controller_xyz;

Class_Controller_xyz::Class_Controller_xyz() {

}


void Class_Controller_xyz::init() {
    for (auto & i : motor) {
        i.Start();
        HAL_Delay(1);
    }
}

void Class_Controller_xyz::DataSend_to_Motor(uint32_t i){
    motor[i].Set_MIT_PD(Target_Motor_Kp[i], Target_Motor_Kd[i]);
    motor[i].Set_MIT_PVT(0, 0, Target_Motor_Torque[i]);
    motor[i].CTRL_MIT();
}


/**
 * @brief 正运动学。
 */
void Class_Controller_xyz::Forward_Kinematics(){
    float cita1 = - motor[0].Info.pos;
    float cita2 = - motor[1].Info.pos;
    float cita3 = - motor[2].Info.pos;

    Eigen::Vector3f point_C1(R * std::cos(fai1), R * std::sin(fai1), 0);
    Eigen::Vector3f point_C2(R * std::cos(fai2), R * std::sin(fai2), 0);
    Eigen::Vector3f point_C3(R * std::cos(fai3), R * std::sin(fai3), 0);
    Eigen::Vector3f point_D1((R + L_arm * std::cos(cita1) - r) * std::cos(fai1),
                             (R + L_arm * std::cos(cita1) - r) * std::sin(fai1),
                             -L_arm * std::sin(cita1));
    Eigen::Vector3f point_D2((R + L_arm * std::cos(cita2) - r) * std::cos(fai2),
                             (R + L_arm * std::cos(cita2) - r) * std::sin(fai2),
                             -L_arm * std::sin(cita2));
    Eigen::Vector3f point_D3((R + L_arm * std::cos(cita3) - r) * std::cos(fai3),
                             (R + L_arm * std::cos(cita3) - r) * std::sin(fai3),
                             -L_arm * std::sin(cita3));

    Eigen::Vector3f vector_OF = (point_D2 + point_D3) / 2;
    float D2D3_a = (point_D3 - point_D2).norm();
    float D1D3_b = (point_D1 - point_D3).norm();
    float D1D2_c = (point_D1 - point_D2).norm();

    float helen_p = (D2D3_a + D1D3_b + D1D2_c) / 2;
    float area_s = std::sqrt(helen_p * (helen_p - D2D3_a) * (helen_p - D1D3_b) * (helen_p - D1D2_c));
    float D2E = D2D3_a * D1D3_b * D1D2_c / (4 * area_s);
    float EF = std::sqrt(std::pow(D2E, 2) - std::pow(D2D3_a / 2, 2));

    Eigen::Vector3f vector_D2D1 = point_D1 - point_D2;
    Eigen::Vector3f vector_D2D3 = point_D3 - point_D2;
    Eigen::Vector3f vector_D3D2 = point_D2 - point_D3;

    Eigen::Vector3f n_FE = vector_D2D1.cross(vector_D2D3).cross(vector_D3D2) /
                           (vector_D2D1.cross(vector_D2D3).cross(vector_D3D2)).norm();
    Eigen::Vector3f vector_FE = n_FE * EF;

    float EP = std::sqrt(std::pow(a_arm, 2) - std::pow(D2E, 2));
    Eigen::Vector3f n_EP = vector_D2D1.cross(vector_D2D3) / (vector_D2D1.cross(vector_D2D3)).norm();
    Eigen::Vector3f vector_EP = n_EP * EP;

    Eigen::Vector3f vector_OP = vector_OF + vector_FE + vector_EP;

    this->Calc_Cord_TCP_XYZ = vector_OP;
    this->TCP_XYZ = {vector_OP(2), -vector_OP(1), vector_OP(0)};
}

/**
 * @brief 计算雅可比矩阵。调用此函数之前需要调用Forward_Kinematics()函数
 */
void Class_Controller_xyz::Get_Jacobian_Matrix() {
    float phi_i[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
    float cita1 = - motor[0].Info.pos;
    float cita2 = - motor[1].Info.pos;
    float cita3 = - motor[2].Info.pos;
    float theta[3] = {cita1, cita2, cita3};
    Eigen::Matrix3f rotation_matrix1;
    rotation_matrix1 << std::cos(phi_i[0]), -std::sin(phi_i[0]), 0,
            std::sin(phi_i[0]), std::cos(phi_i[0]), 0,
            0, 0, 1;

    Eigen::Matrix3f rotation_matrix2;
    rotation_matrix2 << std::cos(phi_i[1]), -std::sin(phi_i[1]), 0,
            std::sin(phi_i[1]), std::cos(phi_i[1]), 0,
            0, 0, 1;

    Eigen::Matrix3f rotation_matrix3;
    rotation_matrix3 << std::cos(phi_i[2]), -std::sin(phi_i[2]), 0,
            std::sin(phi_i[2]), std::cos(phi_i[2]), 0,
            0, 0, 1;

    Eigen::Vector3f vector_OP = this->Calc_Cord_TCP_XYZ; // 已经提前计算好的vector_OP
    Eigen::Vector3f matrix_s1 = vector_OP - rotation_matrix1 * (Eigen::Vector3f(L_arm * std::cos(theta[0]), 0, -L_arm * std::sin(theta[0])) + Eigen::Vector3f(R - r, 0, 0));
    Eigen::Vector3f matrix_s2 = vector_OP - rotation_matrix2 * (Eigen::Vector3f(L_arm * std::cos(theta[1]), 0, -L_arm * std::sin(theta[1])) + Eigen::Vector3f(R - r, 0, 0));
    Eigen::Vector3f matrix_s3 = vector_OP - rotation_matrix3 * (Eigen::Vector3f(L_arm * std::cos(theta[2]), 0, -L_arm * std::sin(theta[2])) + Eigen::Vector3f(R - r, 0, 0));

    Eigen::Vector3f matrix_b1 = rotation_matrix1 * Eigen::Vector3f(L_arm * std::sin(theta[0]), 0, L_arm * std::cos(theta[0]));
    Eigen::Vector3f matrix_b2 = rotation_matrix2 * Eigen::Vector3f(L_arm * std::sin(theta[1]), 0, L_arm * std::cos(theta[1]));
    Eigen::Vector3f matrix_b3 = rotation_matrix3 * Eigen::Vector3f(L_arm * std::sin(theta[2]), 0, L_arm * std::cos(theta[2]));

    Eigen::RowVector3f s1_T = matrix_s1.transpose();
    Eigen::RowVector3f s2_T = matrix_s2.transpose();
    Eigen::RowVector3f s3_T = matrix_s3.transpose();
    Eigen::Matrix3f temp_S = Eigen::Matrix3f::Zero();
    temp_S.row(0) = s1_T;
    temp_S.row(1) = s2_T;
    temp_S.row(2) = s3_T;
    this->J = -temp_S.inverse() *
             ((Eigen::Matrix3f() << matrix_s1.transpose() * matrix_b1, 0, 0,
                     0, matrix_s2.transpose() * matrix_b2, 0,
                     0, 0, matrix_s3.transpose() * matrix_b3).finished());
}

void Class_Controller_xyz::Calculate_Gravity_Compensation() {
    Eigen::Vector3f gravity = {22, 0, 0};
    Eigen::Vector3f torque = this->J.transpose() * gravity;
    if (this->motor[0].Info.pos < -0.01 * M_PI) {
        this->Target_Motor_Torque[0] = -torque(0) * 1.1;
    }
    else{
        this->Target_Motor_Torque[0] = -torque(0);
    }
    this->Target_Motor_Torque[1] = -torque(1);
    this->Target_Motor_Torque[2] = -torque(2);
}

void Class_Controller_xyz::Calculate_and_Control() {
    if (this->motor[0].Info.pos < -0.5 * M_PI){
        this->Target_Motor_Kp[0] = 2.0;
        this->Target_Motor_Kd[0] = 0.5;
        this->Target_Motor_Torque[0] = 0.0;
    }
    else if(this->motor[1].Info.pos < -0.5 * M_PI){
        this->Target_Motor_Kp[1] = 2.0;
        this->Target_Motor_Kd[1] = 0.5;
        this->Target_Motor_Torque[1] = 0.0;
    }
    else if(this->motor[2].Info.pos < -0.5 * M_PI){
        this->Target_Motor_Kp[2] = 2.0;
        this->Target_Motor_Kd[2] = 0.5;
        this->Target_Motor_Torque[2] = 0.0;
    }
    else{
        this->Forward_Kinematics();
        this->Get_Jacobian_Matrix();
        this->Target_Motor_Kp[0] = 0.0;
        this->Target_Motor_Kd[0] = 0.0;
        this->Target_Motor_Kp[1] = 0.0;
        this->Target_Motor_Kd[1] = 0.0;
        this->Target_Motor_Kp[2] = 0.0;
        this->Target_Motor_Kd[2] = 0.0;
        this->Calculate_Gravity_Compensation();
    }
}

std::vector<float> Class_Controller_xyz::Get_TCP_XYZ() {
    return this->TCP_XYZ;
}