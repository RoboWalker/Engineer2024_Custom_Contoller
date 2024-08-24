//
// Created by 31439 on 2023/11/12.
//

#include "Motor_DM.h"
#include "Eigen/Core"

Motor_DM Motor_LF(&hcan1,1,0);
Motor_DM Motor_LB(&hcan1,2,0);
Motor_DM Motor_RB(&hcan2,3,0);
Motor_DM Motor_RF(&hcan2,4,0);


uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///

    float span = x_max - x_min;

    float offset = x_min;

    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}


//等比例整形映射浮点函数
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///

    float span = x_max - x_min;

    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

Motor_DM::Motor_DM(CAN_HandleTypeDef *hcan, uint16_t can_id,uint32_t master_id) {
    this->_hcan = hcan;
    this->_CAN_ID = can_id;
    this->_Master_ID = master_id;
}

void Motor_DM::CTRL_MIT() {
    uint32_t send_mail_box;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

    pos_tmp = float_to_uint(this->Data_MIT.p_int, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(this->Data_MIT.v_int, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(this->Data_MIT.kp_int, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(this->Data_MIT.kd_int, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(this->Data_MIT.t_int, T_MIN, T_MAX, 12);

    CAN_Tx_Header.StdId=this->_CAN_ID;
    CAN_Tx_Header.IDE=CAN_ID_STD;
    CAN_Tx_Header.RTR=CAN_RTR_DATA;
    CAN_Tx_Header.DLC=0x08;
    CAN_Tx_Buffer[0] = (pos_tmp >> 8);
    CAN_Tx_Buffer[1] = pos_tmp;
    CAN_Tx_Buffer[2] = (vel_tmp >> 4);
    CAN_Tx_Buffer[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    CAN_Tx_Buffer[4] = kp_tmp;
    CAN_Tx_Buffer[5] = (kd_tmp >> 4);
    CAN_Tx_Buffer[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    CAN_Tx_Buffer[7] = tor_tmp;

    int test_time = 2000;
    while(HAL_CAN_AddTxMessage(this->_hcan, &this->CAN_Tx_Header, CAN_Tx_Buffer, &send_mail_box) != HAL_OK){
        if(test_time-- == 0){
            break;
        }
    }
}

void Motor_DM::Start() {
    uint32_t send_mail_box;

    CAN_Tx_Header.StdId=this->_CAN_ID;
    CAN_Tx_Header.IDE=CAN_ID_STD;
    CAN_Tx_Header.RTR=CAN_RTR_DATA;
    CAN_Tx_Header.DLC=0x08;
    CAN_Tx_Buffer[0] = 0xFF;
    CAN_Tx_Buffer[1] = 0xFF;
    CAN_Tx_Buffer[2] = 0xFF;
    CAN_Tx_Buffer[3] = 0xFF;
    CAN_Tx_Buffer[4] = 0xFF;
    CAN_Tx_Buffer[5] = 0xFF;
    CAN_Tx_Buffer[6] = 0xFF;
    CAN_Tx_Buffer[7] = 0xFC;

    HAL_CAN_AddTxMessage(this->_hcan, &CAN_Tx_Header, CAN_Tx_Buffer, &send_mail_box);
}

void Motor_DM::Lock() {
    uint32_t send_mail_box;

    CAN_Tx_Header.StdId=_CAN_ID;
    CAN_Tx_Header.IDE=CAN_ID_STD;
    CAN_Tx_Header.RTR=CAN_RTR_DATA;
    CAN_Tx_Header.DLC=0x08;
    CAN_Tx_Buffer[0] = 0xFF;
    CAN_Tx_Buffer[1] = 0xFF;
    CAN_Tx_Buffer[2] = 0xFF;
    CAN_Tx_Buffer[3] = 0xFF;
    CAN_Tx_Buffer[4] = 0xFF;
    CAN_Tx_Buffer[5] = 0xFF;
    CAN_Tx_Buffer[6] = 0xFF;
    CAN_Tx_Buffer[7] = 0xFD;

    HAL_CAN_AddTxMessage(this->_hcan, &CAN_Tx_Header, CAN_Tx_Buffer, &send_mail_box);
}

uint8_t Motor_DM::Process_CAN_FeedBack(Struct_CAN_Rx_Buffer *buffer) {
//    CDC_Transmit_FS((uint8_t*)"Break\n",7);
    if(buffer->Header.StdId == this->_Master_ID)
    {
        int _id = (buffer->Data[0]&0x0F);
        if(_id == this->_CAN_ID)
        {
            this->Info.id = _id;
            this->Info.state = (buffer->Data[0])>>4;
            this->Info.p_int=(buffer->Data[1]<<8)|buffer->Data[2];
            this->Info.v_int=(buffer->Data[3]<<4)|(buffer->Data[4]>>4);
            this->Info.t_int=((buffer->Data[4]&0xF)<<8)|buffer->Data[5];
            this->Info.pos = uint_to_float(this->Info.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
            this->Info.vel = uint_to_float(this->Info.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
            this->Info.toq = uint_to_float(this->Info.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
            this->Info.Tmos = (float)(buffer->Data[6]);
            this->Info.Tcoil = (float)(buffer->Data[7]);
            this->Info.time = HAL_GetTick();
            return 1;
        }
        return 0;
    }
    return 0;
}


void Motor_DM::Set_MIT_PD(float kp, float kd) {
    this->Data_MIT.kp_int = kp;
    this->Data_MIT.kd_int = kd;
}

void Motor_DM::Set_MIT_PVT(float position, float velocity, float torque) {
    this->Data_MIT.p_int = position;
    this->Data_MIT.v_int = velocity;
    this->Data_MIT.t_int = torque;
}

