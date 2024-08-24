/**
 * @file    Motor_DJI.h
 * @brief   大疆电机驱动
 *
 * @author  Tang-yucheng
 * @date    2023-11-17 （创建）
 *
 */

#ifndef __HDL_MOTOR_DJI_H
#define __HDL_MOTOR_DJI_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "Pid.h"
#include "User_Can.h"

/* 宏定义 -------------------------------------------------------------------------------------------------------------*/
#define DJI_C620_L_ID_TX                0x200       // CAN-ID TX: C620 (1-4)
#define DJI_GM6020_H_ID_TX              0x2ff       // CAN-ID TX: GM6020 (5-7)
#define DJI_GM6020_L_C620_H_ID_TX       0x1ff       // CAN-ID TX: GM6020 (1-4) / C620 (5-8)

/* 枚举类型定义 --------------------------------------------------------------------------------------------------------*/
/**
 * @brief 大疆电机状态枚举类型
 */
enum Enum_DJI_Motor_Status
{
    DJI_Motor_Status_DISABLE = 0,
    DJI_Motor_Status_ENABLE,
};

/**
 * @brief 大疆电机的ID枚举类型
 */
enum Enum_DJI_Motor_ID
{
    DJI_Motor_ID_UNDEFINED = 0,
    DJI_Motor_ID_0x201 = 0x201,
    DJI_Motor_ID_0x202 = 0x202,
    DJI_Motor_ID_0x203 = 0x203,
    DJI_Motor_ID_0x204 = 0x204,
    DJI_Motor_ID_0x205 = 0x205,
    DJI_Motor_ID_0x206 = 0x206,
    DJI_Motor_ID_0x207 = 0x207,
    DJI_Motor_ID_0x208 = 0x208,
    DJI_Motor_ID_0x209 = 0x209,
    DJI_Motor_ID_0x20A = 0x20A,
    DJI_Motor_ID_0x20B = 0X20B,
};

/**
 * @brief 大疆电机控制方式
 */
enum Enum_DJI_Motor_Control_Method
{
    DJI_Motor_Control_Method_OPENLOOP = 0,
    DJI_Motor_Control_Method_TORQUE,
    DJI_Motor_Control_Method_OMEGA,
    DJI_Motor_Control_Method_ANGLE,
};

/* 结构体定义 ----------------------------------------------------------------------------------------------------------*/
/**
 * @brief 大疆电机源数据
 */
struct Struct_DJI_Motor_CAN_Data
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Torque_Reverse;
    int8_t Temperature;
    uint8_t Reserved;
} __attribute__((packed));

/**
 * @brief 大疆电机经过处理数据，均为标准单位
 */
struct Struct_DJI_Motor_Data
{
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_Temperature;
    float Pre_Omega;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
};

/* 类定义 -------------------------------------------------------------------------------------------------------------*/
/**
 * @brief DJI-GM6020无刷电机, 电压控制
 */
class Class_DJI_Motor_GM6020
{
public:
    /* 变量 */
    Class_PID PID_Angle;            /*!< PID位置环控制 */
    Class_PID PID_Omega;            /*!< PID速度环控制 */
    Class_PID PID_Torque;           /*!< PID力矩环控制 */

    /* 函数 */
    void Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
              Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_ANGLE,
              int32_t __Encoder_Offset = 0, float __Omega_Max = 320.0f * RPM_TO_RADPS);
    void DataGet();
    void AliveCheck();
    void Control();

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline int32_t Get_Now_Round();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();
    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);

protected:
    /* 常量 */
    Struct_CAN_Manage_Object * CAN_Manage_Object;   /*!< 绑定的CAN */
    Enum_DJI_Motor_ID CAN_ID;                       /*!< 收数据绑定的CAN-ID，C6系列0x201-0x208，GM系列0x205-0x20b */
    uint8_t * CAN_Tx_Data;                          /*!< 发送缓存区 */
    uint32_t Encoder_Offset;                        /*!< 编码器偏移 */
    float Omega_Max;                                /*!< 最大速度, 需根据不同负载测量后赋值, 也就开环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意 */
    float Current_Conversion = 0.0002f;             /*!< 电机实际力矩电流换算值 */
    uint16_t Encoder_Num_Per_Round = 8192;          /*!< 一圈编码器刻度 */
    uint16_t Output_Max = 30000;                    /*!< 最大输出电压 */
    float Lamda = 1.0f;
    float Lamda_Omega = 10.0f;
    float Filter_lambda = 0.1f;

    /* 读变量 */
    Struct_DJI_Motor_Data Data;                     /*!< 电机对外接口信息 */

    /* 读写变量 */
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method =    /*!< 电机控制方式 */
                                  DJI_Motor_Control_Method_ANGLE;
    float Target_Angle = 0.0f;                      /*!< 目标角度 (rad) */
    float Target_Omega = 0.0f;                      /*!< 目标速度 (rad/s) */
    float Target_Torque = 0.0f;                     /*!< 目标力矩电流 (A) */
    float Iq_Set = 0.0f;                            /*!< 输出力矩电流 (A) */
    float U_In = 24.0f;                             /*!< 输入电压 (V)（来自功率检测） */
    float U_Out = 0.0f;                             /*!< 输出电压 (V) */
    int16_t U_Set = 0;                              /*!< CAN发送电压（非标准化） */

    /* 内部变量 */
    uint32_t Flag = 0;                              /*!< 当前时刻的电机接收flag */
    uint32_t Pre_Flag = 0;                          /*!< 前一时刻的电机接收flag */
    Enum_DJI_Motor_Status DJI_Motor_Status =        /*!< 电机状态 */
                          DJI_Motor_Status_DISABLE;
};

/**
 * @brief DJI-C610无刷电调, 电流控制（力矩）
 */
class Class_DJI_Motor_C610
{
public:
    /* 变量 */
    Class_PID PID_Angle;            /*!< PID位置环控制 */
    Class_PID PID_Omega;            /*!< PID速度环控制 */

    /* 函数 */
    void Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
              Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_OMEGA,
              float __Gearbox_Rate = 36.0f, float __Torque_Max = 10000.0f);
    void DataGet();
    void AliveCheck();
    void Control();

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();
    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);
protected:
    /* 常量 */
    Struct_CAN_Manage_Object * CAN_Manage_Object;   /*!< 绑定的CAN */
    Enum_DJI_Motor_ID CAN_ID;                       /*!< 收数据绑定的CAN-ID，C6系列0x201-0x208，GM系列0x205-0x20b */
    uint8_t * CAN_Tx_Data;                          /*!< 发送缓存区 */
    uint32_t Encoder_Offset;                        /*!< 编码器偏移 */
    float Torque_Max;                               /*!< 最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意 */
    float Current_Conversion = 0.0002f;             /*!< 电机实际力矩电流换算值 */
    float Gearbox_Rate = 36.0f;                     /*!< 减速比, 默认带减速箱 */
    uint16_t Encoder_Num_Per_Round = 8192;          /*!< 一圈编码器刻度 */
    uint16_t Output_Max = 10000;                    /*!< 最大输出扭矩 */

    /* 读变量 */
    Struct_DJI_Motor_Data Data;                     /*!< 电机对外接口信息 */

    /* 读写变量 */
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method =    /*!< 电机控制方式 */
                                  DJI_Motor_Control_Method_ANGLE;
    float Target_Angle = 0.0f;                      /*!< 目标角度 (rad) */
    float Target_Omega = 0.0f;                      /*!< 目标速度 (rad/s) */
    float Target_Torque = 0.0f;                     /*!< 目标力矩电流 (A) */
    float Out = 0.0f;                               /*!< 输出电压（非标准化） */

    /* 内部变量 */
    uint32_t Flag = 0;                              /*!< 当前时刻的电机接收flag */
    uint32_t Pre_Flag = 0;                          /*!< 前一时刻的电机接收flag */
    Enum_DJI_Motor_Status DJI_Motor_Status =        /*!< 电机状态 */
                          DJI_Motor_Status_DISABLE;
};

/**
 * @brief DJI-C620无刷电调, 电流控制（力矩）
 */
class Class_DJI_Motor_C620
{
public:
    /* 变量 */
    Class_PID PID_Angle;            /*!< PID位置环控制 */
    Class_PID PID_Omega;            /*!< PID速度环控制 */

    /* 函数 */
    void Init(Struct_CAN_Manage_Object * CAN_Manage_Obj, Enum_DJI_Motor_ID __CAN_ID,
              Enum_DJI_Motor_Control_Method __Control_Method = DJI_Motor_Control_Method_OMEGA,
              float __Gearbox_Rate = 3591.0f / 187.0f, float __Torque_Max = 16384.0f);
    void DataGet();
    void AliveCheck();
    void Control();

    inline uint16_t Get_Output_Max();
    inline Enum_DJI_Motor_Status Get_DJI_Motor_Status();
    inline float Get_Now_Angle();
    inline float Get_Now_Omega();
    inline float Get_Now_Torque();
    inline uint8_t Get_Now_Temperature();
    inline Enum_DJI_Motor_Control_Method Get_Control_Method();
    inline float Get_Target_Angle();
    inline float Get_Target_Omega();
    inline float Get_Target_Torque();
    inline float Get_Out();
    inline void Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __Control_Method);
    inline void Set_Target_Angle(float __Target_Angle);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Target_Torque(float __Target_Torque);
    inline void Set_Out(float __Out);
protected:
    /* 常量 */
    Struct_CAN_Manage_Object * CAN_Manage_Object;   /*!< 绑定的CAN */
    Enum_DJI_Motor_ID CAN_ID;                       /*!< 收数据绑定的CAN-ID，C6系列0x201-0x208，GM系列0x205-0x20b */
    uint8_t * CAN_Tx_Data;                          /*!< 发送缓存区 */
    uint32_t Encoder_Offset;                        /*!< 编码器偏移 */
    float Torque_Max;                               /*!< 最大扭矩, 需根据不同负载测量后赋值, 也就开环和扭矩环输出用得到, 不过我感觉应该没有奇葩喜欢开环输出这玩意 */
    float Current_Conversion = 0.0002f;             /*!< 电机实际力矩电流换算值 */
    float Gearbox_Rate;                             /*!< 减速比, 默认带减速箱 */
    uint16_t Encoder_Num_Per_Round = 8192;          /*!< 一圈编码器刻度 */
    uint16_t Output_Max = 16384;                    /*!< 最大输出扭矩 */

    /* 读变量 */
    Struct_DJI_Motor_Data Data;                     /*!< 电机对外接口信息 */

    /* 读写变量 */
    Enum_DJI_Motor_Control_Method DJI_Motor_Control_Method =    /*!< 电机控制方式 */
                                  DJI_Motor_Control_Method_ANGLE;
    float Target_Angle = 0.0f;                      /*!< 目标角度 (rad) */
    float Target_Omega = 0.0f;                      /*!< 目标速度 (rad/s) */
    float Target_Torque = 0.0f;                     /*!< 目标力矩电流 (A) */
    float Out = 0.0f;                               /*!< 输出电压（非标准化） */

    /* 内部变量 */
    uint32_t Flag = 0;                              /*!< 当前时刻的电机接收flag */
    uint32_t Pre_Flag = 0;                          /*!< 前一时刻的电机接收flag */
    Enum_DJI_Motor_Status DJI_Motor_Status =        /*!< 电机状态 */
                          DJI_Motor_Status_DISABLE;
};

/* 变量声明 ------------------------------------------------------------------------------------------------------------*/
extern uint8_t DJI_C620_L_CAN1_Tx_Data[8];
extern uint8_t DJI_C620_L_CAN2_Tx_Data[8];
extern uint8_t DJI_GM6020_H_CAN1_Tx_Data[8];
extern uint8_t DJI_GM6020_H_CAN2_Tx_Data[8];
extern uint8_t DJI_GM6020_L_C620_H_CAN1_Tx_Data[8];
extern uint8_t DJI_GM6020_L_C620_H_CAN2_Tx_Data[8];

/* 函数声明 ------------------------------------------------------------------------------------------------------------*/
void DJI_CAN_SendData();

/* 接口函数定义 ---------------------------------------------------------------------------------------------------------*/
/**
 * @brief 获取最大输出电压
 *
 * @return uint16_t 最大输出电压
 */
uint16_t Class_DJI_Motor_GM6020::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_GM6020::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_GM6020::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_GM6020::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_GM6020::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_GM6020::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取当前圈数
 *
 * @return int32_t 当前圈数
 */
int32_t Class_DJI_Motor_GM6020::Get_Now_Round()
{
    return (Data.Total_Round);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_GM6020::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_GM6020::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_GM6020::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_GM6020::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_GM6020::Get_Out()
{
    return (U_Set);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_GM6020::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_GM6020::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_GM6020::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_GM6020::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_GM6020::Set_Out(float __Out)
{
    U_Set = __Out;
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_DJI_Motor_C610::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_C610::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_C610::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_C610::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C610::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_C610::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_C610::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_C610::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_C610::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C610::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_C610::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_C610::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_C610::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_C610::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_C610::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_C610::Set_Out(float __Out)
{
    Out = __Out;
}

/**
 * @brief 获取最大输出电流
 *
 * @return uint16_t 最大输出电流
 */
uint16_t Class_DJI_Motor_C620::Get_Output_Max()
{
    return (Output_Max);
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_DJI_Motor_Status 电机状态
 */
Enum_DJI_Motor_Status Class_DJI_Motor_C620::Get_DJI_Motor_Status()
{
    return (DJI_Motor_Status);
}

/**
 * @brief 获取当前的角度, rad
 *
 * @return float 当前的角度, rad
 */
float Class_DJI_Motor_C620::Get_Now_Angle()
{
    return (Data.Now_Angle);
}

/**
 * @brief 获取当前的速度, rad/s
 *
 * @return float 当前的速度, rad/s
 */
float Class_DJI_Motor_C620::Get_Now_Omega()
{
    return (Data.Now_Omega);
}

/**
 * @brief 获取当前的扭矩, 直接采用反馈值
 *
 * @return 当前的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C620::Get_Now_Torque()
{
    return (Data.Now_Torque);
}

/**
 * @brief 获取当前的温度, 摄氏度
 *
 * @return uint8_t 当前的温度, 摄氏度
 */
uint8_t Class_DJI_Motor_C620::Get_Now_Temperature()
{
    return (Data.Now_Temperature);
}

/**
 * @brief 获取电机控制方式
 *
 * @return Enum_DJI_Motor_Control_Method 电机控制方式
 */
Enum_DJI_Motor_Control_Method Class_DJI_Motor_C620::Get_Control_Method()
{
    return (DJI_Motor_Control_Method);
}

/**
 * @brief 获取目标的角度, rad
 *
 * @return float 目标的角度, rad
 */
float Class_DJI_Motor_C620::Get_Target_Angle()
{
    return (Target_Angle);
}

/**
 * @brief 获取目标的速度, rad/s
 *
 * @return float 目标的速度, rad/s
 */
float Class_DJI_Motor_C620::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 获取目标的扭矩, 直接采用反馈值
 *
 * @return float 目标的扭矩, 直接采用反馈值
 */
float Class_DJI_Motor_C620::Get_Target_Torque()
{
    return (Target_Torque);
}

/**
 * @brief 获取输出量
 *
 * @return float 输出量
 */
float Class_DJI_Motor_C620::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定电机控制方式
 *
 * @param __DJI_Motor_Control_Method 电机控制方式
 */
void Class_DJI_Motor_C620::Set_DJI_Motor_Control_Method(Enum_DJI_Motor_Control_Method __DJI_Motor_Control_Method)
{
    DJI_Motor_Control_Method = __DJI_Motor_Control_Method;
}

/**
 * @brief 设定目标的角度, rad
 *
 * @param __Target_Angle 目标的角度, rad
 */
void Class_DJI_Motor_C620::Set_Target_Angle(float __Target_Angle)
{
    Target_Angle = __Target_Angle;
}

/**
 * @brief 设定目标的速度, rad/s
 *
 * @param __Target_Omega 目标的速度, rad/s
 */
void Class_DJI_Motor_C620::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 * @brief 设定目标的扭矩, 直接采用反馈值
 *
 * @param __Target_Torque 目标的扭矩, 直接采用反馈值
 */
void Class_DJI_Motor_C620::Set_Target_Torque(float __Target_Torque)
{
    Target_Torque = __Target_Torque;
}

/**
 * @brief 设定输出量
 *
 * @param __Output_Voltage 输出量
 */
void Class_DJI_Motor_C620::Set_Out(float __Out)
{
    Out = __Out;
}

#endif  /* HDL_Motor_DJI */