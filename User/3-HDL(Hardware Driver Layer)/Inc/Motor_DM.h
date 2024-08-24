#ifndef DM_SPINE_MOTOR_DM_H
#define DM_SPINE_MOTOR_DM_H

#include "User_Can.h"

#define P_MIN (-12.5f)
#define P_MAX 12.5f
#define V_MIN (-30.0f)
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN (-10.0f)
#define T_MAX 10.0f
#define Master_ID 0

typedef struct
{
    uint32_t time;
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float toq;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} __attribute__((packed)) Motor_Inf;

typedef struct
{
    float p_int;
    float v_int;
    float kp_int;
    float kd_int;
    float t_int;
}  __attribute__((packed))  Motor_MIT_Data_t;

class Motor_DM{
public:
    Motor_DM(CAN_HandleTypeDef* hcan,uint16_t can_id,uint32_t master_id);
    void CTRL_MIT();
    void Start();
    void Lock();
    uint8_t Process_CAN_FeedBack(Struct_CAN_Rx_Buffer* buffer);
    void Set_MIT_PD(float  kp,float kd);
    void Set_MIT_PVT(float position,float velocity,float torque);
    Motor_MIT_Data_t Data_MIT = {0};
    Motor_Inf   Info = {0};
private:
    CAN_HandleTypeDef* _hcan;
    uint16_t _CAN_ID;
    uint32_t _Master_ID;
    CAN_TxHeaderTypeDef CAN_Tx_Header;
    uint8_t CAN_Tx_Buffer[8];
};


#endif //DM_SPINE_MOTOR_DM_H

