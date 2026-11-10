#ifndef __SGUANFOC_H
#define __SGUANFOC_H

/* ============================================================================
 *  SguanFOC —— 无感 SMO + PLL（扁平化移植版）
 *  函数名 / 结构体字段名 / 运行逻辑 与 SguanFOC_Library(STM32G4下桥臂双电阻) 一致
 *  唯一区别：去掉 sguan-> 与 ->transfer. 两层嵌套，直接操作全局结构体
 *  不含：HFI / NLFO / Hall / 编码器 / LADRC / SMC / STA / DOB / 弱磁 / 前馈
 * ==========================================================================*/
#include <stdint.h>

/* ---------------------------- 系统常量（Sguan_math.h）--------------------- */
#define Value_PI            3.14159265358979f
#define Value_2PI           6.28318530717959f
#define Value_PI_2          1.57079632679490f
#define Value_SQRT3         1.73205080756888f
#define Value_SQRT3_2       0.86602540378444f
#define Value_INV_SQRT3     0.57735026918963f
#define Value_N_INF         (-1e30f)

/* ---------------------------- 配置 ---------------------------------------- */
#define CONFIG_PWM          0           /* 0=SVPWM(七段式)  1=SPWM(零序注入) */
#define CONFIG_SPEED_USE_POT 1          /* 1=JDR4电位器给目标转速            */
#define SPEED_REF_RPM       1000.0f     /* 电位器关闭时的固定目标 rpm        */

/* ---------------------------- 时间与 PWM ---------------------------------- */
#define PMSM_RUN_T          (1.0f/20000.0f) /* 控制周期 s（20kHz，170MHz/(2*4249)）*/
#define PMSM_PWM_ARR        4249            /* TIM1 Period（中心对齐）      */

/* ---------------------------- 电机与硬件（必须按实际改）------------------- */
#define MOTOR_POLES         7               /* 极对数                        */
#define MOTOR_RS            0.140f          /* 相电阻 Ω（实测 140 mΩ）        */
#define MOTOR_LD            0.000050f       /* D 轴电感 H（实测 50 µH）       */
#define MOTOR_LQ            0.000050f       /* Q 轴电感 H（表贴式 Ld=Lq）     */
#define MOTOR_FLUX          0.00158f        /* 磁链 Wb                       */
#define MOTOR_VBUS          12.0f           /* 标称母线 V                    */

/* ADC 标定（对应 Offset_Current_Init 里的 Final_Gain 计算）*/
#define MCU_VOLTAGE         3.3f            /* ADC 参考电压 V                */
#define ADC_PRECISION       4096.0f         /* 12 位 ADC                     */
#define AMPLIFIER           10.0f            /* 运放增益（按原理图填）        */
#define SAMPLING_RS         0.005f          /* 采样电阻 Ω（按原理图填）      */
#define ADC_VBUS_TO_VOLT    0.017953f       /* 母线 ADC -> V                 */
#define ADC_CUR_OFFSET      2048.0f         /* 零漂初值（上电会重新标定）    */

/* ---------------------------- 无感起动参数 -------------------------------- */
#define SL_ALIGN_I          0.8f            /* 对齐电流 A                    */
#define SL_ALIGN_MS         500             /* 对齐时间 ms                   */
#define SL_OPEN_IQ          2.0f            /* 强拖 Iq A                     */
#define SL_OPEN_RPM         1200.0f         /* 强拖目标 rpm（须 > AbsMax 1050）*/
#define SL_LTD_R            1.0f          /* 强拖 LTD 速度因子（越大加速越快）*/
#define SL_SW_THETA         0.15f           /* 切换角度阈值（电角度标幺）    */
#define SL_SW_RPM           250.0f          /* 切换转速一致性阈值 rpm        */
#define SL_SW_CNT           300             /* 连续满足拍数                  */
#define SL_SW_TIMEOUT       4000            /* 切换超时拍数                  */

/* ============================================================================
 *  数据结构（对应 SguanFOC 各 .h，字段名完全保留）
 * ==========================================================================*/

/* --- LTD 最速控制（Sguan_LTD.h）--- */
typedef struct{
    float num;              // (中间量)传递函数的分子系数
    float den;              // (中间量)传递函数的分母系数
    float Last_i;           // (数据)历史的输入数值
    float Input;            // (输入数据)阶跃输入的数值
    float Output;           // (输出数据)输出平滑的数值
}LTD_GO_STRUCT;

typedef struct{
    LTD_GO_STRUCT go;
    float T;                // (系统时钟)离散周期
    float r;                // (参数设计)r速度因子_跟踪微分器
}LTD_STRUCT;

/* --- LPF 一阶低通（Sguan_Filter.h）--- */
typedef struct{
    float I_i, O_i;         // (数据)历史输入/输出
    float Input, Output;
}LPF_GO_STRUCT;

typedef struct{
    LPF_GO_STRUCT filter;
    float F_num, F_den;     // (中间量)传递函数系数
    float T;                // (系统时钟)离散周期
    float Wc;               // (参数设计)截止频率
}LPF_STRUCT;

/* --- SMO 滑模观测器（Sguan_SMO.h）--- */
typedef struct{
    float I_i;              // (数据)积分项的历史输入
    float F_i;              // (数据)滤波项的历史输入
    float F_o;              // (数据)滤波项的历史输出
    float Input_Ix;         // (输入数据)x轴的实际电流
    float Input_Ux;         // (输入数据)x轴的实际电压
    float Input_Iy;         // (输入数据)y轴的预测电流
    float Output_Ex;        // (输出数据)x轴的反电动势预测
    float Output_Ix;        // (输出数据)x轴的预测电流
    uint8_t IntegralFrozen_flag;
}SMO_GO_STRUCT;

typedef struct{
    float I_num;            // (中间量)积分项传递函数分子
    float F_num;            // (中间量)滤波项传递函数分子
    float F_den;            // (中间量)滤波项传递函数分母
    float Gain0;            // (中间量)1/Ld
    float Gain1;            // (中间量)Rs/Ld
    float Gain2;            // (中间量)(Ld-Lq)/Ld
    float Input_We;         // (输入数据)电机电角速度
}SMO_DATA_STRUCT;

typedef struct{
    SMO_GO_STRUCT   alpha;
    SMO_GO_STRUCT   beta;
    SMO_DATA_STRUCT data;
    float T;                // (系统时钟)离散周期
    float Rs;               // (电机参数)相电阻
    float Ld;               // (电机参数)D轴电感
    float Lq;               // (电机参数)Q轴电感
    float Wc;               // (参数设计)低通截止频率
    float h;                // (参数设计)滑模解调增益
    float IntMax;           // (参数设计)积分限幅
    float IntMin;           // (参数设计)积分限幅
}SMO_STRUCT;

/* --- PLL（Sguan_PLL.h）--- */
typedef struct{
    float X_num[2];         // (中间量)PI传递函数分子
    float Y_num;            // (中间量)积分传递函数分子
    float We_i;             // (数据)历史误差
    float Re_i;             // (数据)历史角速度
    float OutWe;            // (输出)机械角速度 rad/s
    float OutRe;            // (输出)机械角度 rad
    float Error;            // (输入)鉴相误差
}PLL_GO_STRUCT;

typedef struct{
    PLL_GO_STRUCT go;
    float T;                // (系统时钟)离散周期
    float Kp;               // (参数设计)比例项
    float Ki;               // (参数设计)积分项
    uint8_t is_position_mode;
}PLL_STRUCT;

/* --- PID（Sguan_PID.h）--- */
typedef struct{
    float i[2];             // (数据)历史误差
    float Io;               // (数据)积分项
    float Do;               // (数据)微分项
    float I_num;            // (中间量)积分系数
    float D_num;            // (中间量)微分系数分子
    float D_den;            // (中间量)微分系数分母
    float Ref;              // (输入)给定
    float Fbk;              // (输入)反馈
    float Output;           // (输出)输出
    uint8_t IntegralFrozen_flag;
}PID_RUN_STRUCT;

typedef struct{
    PID_RUN_STRUCT run;
    float T;                // (系统时钟)离散周期
    float Kp;               // (参数设计)比例项
    float Ki;               // (参数设计)积分项
    float Kd;               // (参数设计)微分项
    float Wc;               // (参数设计)微分滤波截止频率
    float OutMax;           // (参数设计)输出上限
    float OutMin;           // (参数设计)输出下限
    float IntMax;           // (参数设计)积分上限
    float IntMin;           // (参数设计)积分下限
}PID_STRUCT;

/* --- 电机本体 --- */
typedef struct{
    uint8_t Poles;          // (电机参数)极对数
    float Rs;               // (电机参数)相电阻
    float Ld;               // (电机参数)D轴电感
    float Lq;               // (电机参数)Q轴电感
    float Flux;             // (电机参数)磁链
    float VBUS;             // (电机参数)标称母线
    uint32_t Duty;          // (参数设计)PWM满量程
    int8_t PWM_Dir;         // (参数设计)PWM极性 1=正 -1=反(PWM2)
    int8_t Motor_Dir;       // (参数设计)电机方向 1=正 -1=反
    uint8_t Current_Num;    // (参数设计)0=AB 1=AC 2=BC
    float Current_Dir0;     // (参数设计)采样通道0方向
    float Current_Dir1;     // (参数设计)采样通道1方向
}MOTOR_STRUCT;

/* --- FOC 主数据（MOTOR_FOC_STRUCT）--- */
typedef struct{
    float Target_Speed;     // (期望速度)机械角速度 rad/s
    float Target_Id;        // (期望电流)D轴
    float Target_Iq;        // (期望电流)Q轴
    float Target_VF_Uq;     // (期望电压)固定Q轴电压
    float Target_IF_Iq;     // (期望电流)固定Q轴电流

    float Speed_in;         // (输入量)速度环输入值（LTD输出）
    float Ud_in;            // (输入值)D轴电压
    float Uq_in;            // (输入值)Q轴电压

    float Ualpha;           // (中间量)alpha轴电压
    float Ubeta;            // (中间量)beta轴电压

    float Du;               // (数据)U相占空比
    float Dv;               // (数据)V相占空比
    float Dw;               // (数据)W相占空比

    float sine;             // (数据)正弦值
    float cosine;           // (数据)余弦值

    float Real_VBUS;        // (数据)实际母线电压
}MOTOR_FOC_STRUCT;

/* --- 电流（MOTOR_CURRENT_STRUCT）--- */
typedef struct{
    float Real_Ia, Real_Ib, Real_Ic;
    float Real_Ialpha, Real_Ibeta;
    float Real_Id, Real_Iq;

    float Current_offset0, Current_offset1;
    float Final_Gain;
}MOTOR_CURRENT_STRUCT;

/* --- 角度（MOTOR_ENCODER_STRUCT，只留无感项）--- */
typedef struct{
    float Real_Speed;       // (数据)机械角速度 rad/s
    float Real_Pos;         // (数据)机械角度 rad
    float Real_We;          // (数据)电角速度 rad/s
    float Real_Re;          // (数据)电角度 rad
    float Real_offset;      // (数据)角度偏置
}MOTOR_ENCODER_STRUCT;

/* --- 运行参数（对应 MOTOR_VALUE_STRUCT 的无感部分）--- */
typedef struct{
    uint8_t Status;         // (状态机)当前状态
    float   ThetaErr;       // (数据)观测角-开环角（标幺）

    /* ===== 无感权重模式（与 SguanFOC 的 value.* 一致）===== */
    float   Low_angle;      // (中间量)低速域电角度 rad（0~2π）
    float   High_angle;     // (中间量)高速域电角度 rad
    float   Low_Wm;         // (中间量)低速域机械角速度 rad/s
    float   High_Wm;        // (中间量)高速域机械角速度 rad/s
    float   Sensorless_Gain;// (中间量)观测器切换权重 0~1

    float   Sensorless_Start;   // (参数设计)高速域观测器开始运行界限 rad/s
    float   Sensorless_AbsMin;  // (参数设计)过渡区下限 rad/s
    float   Sensorless_AbsMax;  // (参数设计)过渡区上限 rad/s
}MOTOR_VALUE_STRUCT;

/* ============================================================================
 *  状态机（Sguan_MotorStatus.h）
 * ==========================================================================*/
#define MOTOR_STATUS_START          0x01    // 开始初始化（电流零漂标定）
#define MOTOR_STATUS_ALIGN          0x02    // 对齐（本工程新增，锁 θ=0）
#define MOTOR_STATUS_OPEN           0x03    // I/f 强拖
#define MOTOR_STATUS_IDLE           0x04    // 无感闭环（正常运行）

/* ============================================================================
 *  全局实例
 * ==========================================================================*/
extern MOTOR_FOC_STRUCT     foc;
extern MOTOR_CURRENT_STRUCT current;
extern MOTOR_ENCODER_STRUCT encoder;
extern MOTOR_STRUCT         motor;
extern MOTOR_VALUE_STRUCT   value;

extern SMO_STRUCT   SMO;
extern PLL_STRUCT   PLL_encoder;
extern PID_STRUCT   Current_D;
extern PID_STRUCT   Current_Q;
extern PID_STRUCT   Velocity;
extern LPF_STRUCT   LPF_D;
extern LPF_STRUCT   LPF_Q;
extern LPF_STRUCT   LPF_encoder;
extern LTD_STRUCT   LTD;

/* ============================================================================
 *  数学（Sguan_math.h）
 * ==========================================================================*/
void  Value_Limit(float *a, float max, float min);
float Value_normalize(float angle);
float Value_fabsf(float x);
float Value_Sign(float value);
void  Value_Correct(float *angle, float error);
float Value_Gain_Get(float *gain, float real_speed, float abs_max, float abs_min);
void  Value_Rad_Loop(float *Output, float Input, float Last_in, float T);

float fast_sin(float x);
void  fast_sin_cos(float x, float *sin_x, float *cos_x);

void  clarke(float *i_alpha, float *i_beta, float i_a, float i_b);
void  park(float *i_d, float *i_q, float i_alpha, float i_beta,
           float sine, float cosine);
void  ipark(float *u_alpha, float *u_beta, float u_d, float u_q,
            float sine, float cosine);

/* ============================================================================
 *  调制（Sguan_SVPWM.h / Sguan_SPWM.h）
 * ==========================================================================*/
void  SVPWM(float u_alpha, float u_beta, float *d_u, float *d_v, float *d_w);
void  SPWM (float u_alpha, float u_beta, float *d_u, float *d_v, float *d_w);

/* ============================================================================
 *  控制器（Sguan_PID.h / Sguan_SMO.h / Sguan_PLL.h / Sguan_LTD.h）
 * ==========================================================================*/
void  PID_Init(PID_STRUCT *pid);
void  PID_Loop(PID_STRUCT *pid);

void  SMO_Init(SMO_STRUCT *smo);
void  SMO_Loop(SMO_STRUCT *smo);

void  PLL_Init(PLL_STRUCT *pll);
void  PLL_Loop(PLL_STRUCT *pll);

void  LPF_Init(LPF_STRUCT *lpf);
void  LPF_Loop(LPF_STRUCT *lpf);

void  LTD_Init(LTD_STRUCT *ltd);
void  LTD_Loop(LTD_STRUCT *ltd);

/* ============================================================================
 *  顶层
 * ==========================================================================*/
void  SguanFOC_Init(void);
void  SguanFOC_High_Loop(void);

#endif /* __SGUANFOC_H */
