/**
 * @file    Foc.c
 * @brief   SguanFOC 无感（I/f 强拖 + SMO + PLL）扁平移植
 * @note    函数名、结构体字段名、执行顺序 与 SguanFOC_Library(STM32G4下桥臂双电阻) 一致
 *          唯一区别：去掉 sguan-> 与 ->transfer. 两层嵌套
 *          去掉的内容：HFI / NLFO / Hall / 编码器 / LADRC / SMC / STA / DOB / 弱磁 / 前馈
 */
#include "Foc.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include <math.h>

/* ============================ 外部数据 =================================== */
extern volatile uint32_t ADC_InjectedValues[4];     /* [0]母线 [1]Ia [2]Ic [3]电位器 */
extern TIM_HandleTypeDef htim1;

/* ============================ 全局实例 =================================== */
MOTOR_FOC_STRUCT     foc;
MOTOR_CURRENT_STRUCT current;
MOTOR_ENCODER_STRUCT encoder;
MOTOR_STRUCT         motor;
MOTOR_VALUE_STRUCT   value;

SMO_STRUCT  SMO;
PLL_STRUCT  PLL_encoder;
PID_STRUCT  Current_D;
PID_STRUCT  Current_Q;
PID_STRUCT  Velocity;
LPF_STRUCT  LPF_D;
LPF_STRUCT  LPF_Q;
LPF_STRUCT  LPF_encoder;
LTD_STRUCT  LTD;

/* ============================ 内部静态量 ================================ */
static uint8_t  Encoder_Tick_Index = 0;     /* 0=开环 1=SMO无感 */
static uint16_t s_AlignCnt = 0;
/* 权重模式不再需要切换确认计数（Sensorless_Gain 本身就是平滑过渡）*/
static uint16_t s_OffCnt   = 0;
static float    s_OffSum0  = 0.0f;
static float    s_OffSum1  = 0.0f;
static uint8_t  s_InitDone = 0;

/* ============================ 1. 数学（Sguan_math.c）==================== */
void Value_Limit(float *a, float max, float min)
{
    if (*a > max)      *a = max;
    else if (*a < min) *a = min;
}

float Value_normalize(float angle)
{
    while (angle >= Value_2PI) angle -= Value_2PI;
    while (angle <  0.0f)      angle += Value_2PI;
    return angle;
}

float Value_fabsf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

float Value_Sign(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

void Value_Correct(float *angle, float error)
{
    while (error >  Value_PI) error -= Value_2PI;
    while (error < -Value_PI) error += Value_2PI;
    *angle = error;
}

/* 无感双域权重：abs_min -> 0, abs_max -> 1 */
float Value_Gain_Get(float *gain, float real_speed, float abs_max, float abs_min)
{
    float k;

    k = (Value_fabsf(real_speed) - abs_min) / (abs_max - abs_min);
    Value_Limit(&k, 1.0f, 0.0f);
    *gain = k;

    return Value_fabsf(real_speed);
}

void Value_Rad_Loop(float *Output, float Input, float Last_in, float T)
{
    *Output += (Input + Last_in) * (T / 2.0f);
    *Output  = Value_normalize(*Output);
}

float fast_sin(float x)
{
    return sinf(x);
}

void fast_sin_cos(float x, float *sin_x, float *cos_x)
{
    *sin_x = sinf(x);
    *cos_x = cosf(x);
}

/* ============================ 2. 坐标变换 =============================== */
void clarke(float *i_alpha, float *i_beta, float i_a, float i_b)
{
    *i_alpha = i_a;
    *i_beta  = (i_a + 2.0f * i_b) * Value_INV_SQRT3;
}

void park(float *i_d, float *i_q,
          float i_alpha, float i_beta,
          float sine, float cosine)
{
    *i_d =  i_alpha * cosine + i_beta * sine;
    *i_q =  i_beta  * cosine - i_alpha * sine;
}

void ipark(float *u_alpha, float *u_beta,
           float u_d, float u_q,
           float sine, float cosine)
{
    *u_alpha = u_d * cosine - u_q * sine;
    *u_beta  = u_q * cosine + u_d * sine;
}

/* ============================ 3. SVPWM（Sguan_SVPWM.c）================== */
void SVPWM(float u_alpha, float u_beta, float *d_u, float *d_v, float *d_w)
{
    const float ts = 1.0f;

    float u1 = u_beta;
    float u2 = -Value_SQRT3_2 * u_alpha - 0.5f * u_beta;
    float u3 =  Value_SQRT3_2 * u_alpha - 0.5f * u_beta;

    uint8_t sector = (uint8_t)((u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2));

    float t_a, t_b, t_c, k_svpwm;

    if (sector == 5)
    {
        float t4 =  u3, t6 =  u1, sum = t4 + t6;
        if (sum > ts) { k_svpwm = ts / sum; t4 = k_svpwm * t4; t6 = k_svpwm * t6; }
        { float t7 = (ts - t4 - t6) * 0.5f;
          t_a = t4 + t6 + t7; t_b = t6 + t7; t_c = t7; }
    }
    else if (sector == 1)
    {
        float t2 = -u3, t6 = -u2, sum = t2 + t6;
        if (sum > ts) { k_svpwm = ts / sum; t2 = k_svpwm * t2; t6 = k_svpwm * t6; }
        { float t7 = (ts - t2 - t6) * 0.5f;
          t_a = t6 + t7; t_b = t2 + t6 + t7; t_c = t7; }
    }
    else if (sector == 3)
    {
        float t2 = u1, t3 = u2, sum = t2 + t3;
        if (sum > ts) { k_svpwm = ts / sum; t2 = k_svpwm * t2; t3 = k_svpwm * t3; }
        { float t7 = (ts - t2 - t3) * 0.5f;
          t_a = t7; t_b = t2 + t3 + t7; t_c = t3 + t7; }
    }
    else if (sector == 2)
    {
        float t1 = -u1, t3 = -u3, sum = t1 + t3;
        if (sum > ts) { k_svpwm = ts / sum; t1 = k_svpwm * t1; t3 = k_svpwm * t3; }
        { float t7 = (ts - t1 - t3) * 0.5f;
          t_a = t7; t_b = t3 + t7; t_c = t1 + t3 + t7; }
    }
    else if (sector == 6)
    {
        float t1 = u2, t5 = u3, sum = t1 + t5;
        if (sum > ts) { k_svpwm = ts / sum; t1 = k_svpwm * t1; t5 = k_svpwm * t5; }
        { float t7 = (ts - t1 - t5) * 0.5f;
          t_a = t5 + t7; t_b = t7; t_c = t1 + t5 + t7; }
    }
    else if (sector == 4)
    {
        float t4 = -u2, t5 = -u1, sum = t4 + t5;
        if (sum > ts) { k_svpwm = ts / sum; t4 = k_svpwm * t4; t5 = k_svpwm * t5; }
        { float t7 = (ts - t4 - t5) * 0.5f;
          t_a = t4 + t5 + t7; t_b = t7; t_c = t5 + t7; }
    }
    else
    {
        t_a = 0.5f; t_b = 0.5f; t_c = 0.5f;
    }

    *d_u = t_a;
    *d_v = t_b;
    *d_w = t_c;
}

/* ============================ 4. SPWM（Sguan_SPWM.c）==================== */
void SPWM(float u_alpha, float u_beta, float *d_u, float *d_v, float *d_w)
{
    float u_a = u_alpha * Value_INV_SQRT3;
    float u_b = (-0.5f * u_alpha + Value_SQRT3_2 * u_beta) * Value_INV_SQRT3;
    float u_c = (-0.5f * u_alpha - Value_SQRT3_2 * u_beta) * Value_INV_SQRT3;

    float u_max = u_a, u_min = u_a, u_zero;
    if (u_b > u_max) u_max = u_b;
    if (u_c > u_max) u_max = u_c;
    if (u_b < u_min) u_min = u_b;
    if (u_c < u_min) u_min = u_c;

    u_zero = -(u_max + u_min) * 0.5f;

    *d_u = u_a + u_zero + 0.5f;
    *d_v = u_b + u_zero + 0.5f;
    *d_w = u_c + u_zero + 0.5f;
}

/* ============================ 5. PID（Sguan_PID.c）===================== */
void PID_Init(PID_STRUCT *pid)
{
    double temp0 = ((double)pid->T) * ((double)pid->Ki) / 2.0;
    double temp1 = ((double)pid->T) * ((double)pid->Wc);
    double temp2 = ((double)pid->Kd) * ((double)pid->Wc);
    double den   = -2.0 + temp1;

    pid->run.I_num = (float)temp0;
    pid->run.D_num = (float)((2.0 * temp2) / den);
    pid->run.D_den = (float)((2.0 + temp1) / den);

    pid->run.i[0] = 0.0f;
    pid->run.i[1] = 0.0f;
    pid->run.Io   = 0.0f;
    pid->run.Do   = 0.0f;
    pid->run.Ref    = 0.0f;
    pid->run.Fbk    = 0.0f;
    pid->run.Output = 0.0f;
    pid->run.IntegralFrozen_flag = 0;
}

void PID_Loop(PID_STRUCT *pid)
{
    pid->run.i[0] = pid->run.Ref - pid->run.Fbk;

    if (pid->Ki)
    {
        if (pid->run.IntegralFrozen_flag)
        {
            if ((pid->run.i[0] * pid->run.Io < 0.0f) ||
                ((pid->run.Io < pid->IntMax) && (pid->run.Io > pid->IntMin)))
            {
                pid->run.IntegralFrozen_flag = 0;
            }
        }
        else
        {
            pid->run.Io += pid->run.I_num * (pid->run.i[0] + pid->run.i[1]);

            if (pid->run.Io > pid->IntMax)
            {
                pid->run.Io = pid->IntMax;
                pid->run.IntegralFrozen_flag = 1;
            }
            else if (pid->run.Io < pid->IntMin)
            {
                pid->run.Io = pid->IntMin;
                pid->run.IntegralFrozen_flag = 1;
            }
        }
    }

    if (pid->Kd)
    {
        pid->run.Do = pid->run.D_num * (pid->run.i[0] + pid->run.i[1])
                    - pid->run.D_den * pid->run.Do;
    }

    pid->run.Output = pid->run.i[0] * pid->Kp + pid->run.Io + pid->run.Do;
    Value_Limit(&pid->run.Output, pid->OutMax, pid->OutMin);

    pid->run.i[1] = pid->run.i[0];
}

/* ============================ 6. SMO（Sguan_SMO.c）===================== */
static void SMO_RUN(SMO_STRUCT *smo, SMO_GO_STRUCT *run)
{
    float I_input, part0, part1, part2, part3, error, Ex;

    part0 = run->Input_Ux * smo->data.Gain0;
    part1 = run->Output_Ix * smo->data.Gain1;
    part2 = smo->data.Input_We * smo->data.Gain2 * run->Input_Iy;
    part3 = run->Output_Ex * smo->data.Gain0;

    I_input = part0 - part1 - part2 - part3;

    if (run->IntegralFrozen_flag)
    {
        if ((I_input * run->Output_Ix < 0.0f) ||
            ((run->Output_Ix < smo->IntMax) && (run->Output_Ix > smo->IntMin)))
        {
            run->IntegralFrozen_flag = 0;
        }
    }
    else
    {
        run->Output_Ix += smo->data.I_num * (I_input + run->I_i);

        if (run->Output_Ix > smo->IntMax)
        {
            run->Output_Ix = smo->IntMax;
            run->IntegralFrozen_flag = 1;
        }
        else if (run->Output_Ix < smo->IntMin)
        {
            run->Output_Ix = smo->IntMin;
            run->IntegralFrozen_flag = 1;
        }
    }

    error = run->Output_Ix - run->Input_Ix;
    error = Value_Sign(error);   /* ★ 滑模必须用符号函数，不能用限幅！*/
    Ex = error * smo->h;
    run->Output_Ex = smo->data.F_num * (Ex + run->F_i) - smo->data.F_den * run->F_o;

    run->I_i = I_input;
    run->F_i = Ex;
    run->F_o = run->Output_Ex;
}

void SMO_Init(SMO_STRUCT *smo)
{
    double temp0 = ((double)smo->T) * ((double)smo->Wc);
    double den   = 2.0 + temp0;

    smo->data.I_num = (float)(((double)smo->T) / 2.0);
    smo->data.F_num = (float)(temp0 / den);
    smo->data.F_den = (float)((-2.0 + temp0) / den);

    smo->data.Gain0 = (float)(1.0 / ((double)smo->Ld));
    smo->data.Gain1 = (float)(((double)smo->Rs) / ((double)smo->Ld));
    smo->data.Gain2 = (float)((((double)smo->Ld) - ((double)smo->Lq)) /
                              ((double)smo->Ld));

    smo->alpha.I_i = 0.0f;  smo->alpha.F_i = 0.0f;  smo->alpha.F_o = 0.0f;
    smo->alpha.Input_Ix = 0.0f;  smo->alpha.Input_Ux = 0.0f;
    smo->alpha.Input_Iy = 0.0f;  smo->alpha.Output_Ex = 0.0f;
    smo->alpha.Output_Ix = 0.0f; smo->alpha.IntegralFrozen_flag = 0;

    smo->beta.I_i = 0.0f;   smo->beta.F_i = 0.0f;   smo->beta.F_o = 0.0f;
    smo->beta.Input_Ix = 0.0f;   smo->beta.Input_Ux = 0.0f;
    smo->beta.Input_Iy = 0.0f;   smo->beta.Output_Ex = 0.0f;
    smo->beta.Output_Ix = 0.0f;  smo->beta.IntegralFrozen_flag = 0;

    smo->data.Input_We = 0.0f;
}

void SMO_Loop(SMO_STRUCT *smo)
{
    SMO_RUN(smo, &smo->alpha);
    SMO_RUN(smo, &smo->beta);
    smo->alpha.Input_Iy = smo->beta.Output_Ix;
    smo->beta.Input_Iy  = smo->alpha.Output_Ix;
}

/* ============================ 7. PLL（Sguan_PLL.c）===================== */
void PLL_Init(PLL_STRUCT *pll)
{
    double temp0 = ((double)pll->T) * ((double)pll->Ki);

    pll->go.X_num[0] = (float)((2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.X_num[1] = (float)((-2.0 * ((double)pll->Kp) + temp0) / 2.0);
    pll->go.Y_num    = (float)(((double)pll->T) / 2.0);

    pll->is_position_mode = 0;
    pll->go.We_i  = 0.0f;
    pll->go.Re_i  = 0.0f;
    pll->go.OutWe = 0.0f;
    pll->go.OutRe = 0.0f;
    pll->go.Error = 0.0f;
}

void PLL_Loop(PLL_STRUCT *pll)
{
    pll->go.OutWe += pll->go.X_num[0] * pll->go.Error
                   + pll->go.X_num[1] * pll->go.We_i;	
	
    pll->go.OutRe += pll->go.Y_num * (pll->go.OutWe + pll->go.Re_i);
    if (!pll->is_position_mode)
    {
        pll->go.OutRe = Value_normalize(pll->go.OutRe);
    }

    pll->go.We_i = pll->go.Error;
    pll->go.Re_i = pll->go.OutWe;
		
		
}

/* ============================ 8. LPF（Sguan_Filter.c）================== */
void LPF_Init(LPF_STRUCT *lpf)
{
    double temp0 = ((double)lpf->T) * ((double)lpf->Wc);
    double den   = 2.0 + temp0;

    lpf->F_num = (float)(temp0 / den);
    lpf->F_den = (float)((-2.0 + temp0) / den);

    lpf->filter.I_i = 0.0f;
    lpf->filter.O_i = 0.0f;
    lpf->filter.Input  = 0.0f;
    lpf->filter.Output = 0.0f;
}

void LPF_Loop(LPF_STRUCT *lpf)
{
    lpf->filter.Output = lpf->F_num * (lpf->filter.Input + lpf->filter.I_i)
                       - lpf->F_den * lpf->filter.O_i;

    lpf->filter.I_i = lpf->filter.Input;
    lpf->filter.O_i = lpf->filter.Output;
}

/* ============================ 9. LTD（Sguan_LTD.c）===================== */
void LTD_Init(LTD_STRUCT *ltd)
{
    double temp1 = ((double)ltd->T) * ((double)ltd->r);
    double den   = 2.0 + temp1;

    ltd->go.num = (float)(temp1 / den);
    ltd->go.den = (float)((-2.0 + temp1) / den);

    ltd->go.Input  = 0.0f;
    ltd->go.Output = 0.0f;
    ltd->go.Last_i = 0.0f;
}

void LTD_Loop(LTD_STRUCT *ltd)
{
    ltd->go.Output = ltd->go.num * (ltd->go.Input + ltd->go.Last_i)
                   - ltd->go.den * ltd->go.Output;
    ltd->go.Last_i = ltd->go.Input;
}

/* ============================ 10. Transfer 包装（SguanFOC.c）=========== */
static void Transfer_PID_Loop(PID_STRUCT *pid, float Ref, float Fbk)
{
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop(pid);
}

static void Transfer_LPF_Loop(LPF_STRUCT *lpf, float input)
{
    lpf->filter.Input = input;
    LPF_Loop(lpf);
}

static void Transfer_LTD_Loop(LTD_STRUCT *ltd, float *output, float input)
{
    ltd->go.Input = input;
    LTD_Loop(ltd);
    *output = ltd->go.Output;
}

/* SMO 观测器 + PLL：对应 Transfer_SMO_Loop */
static void Transfer_SMO_Loop(PLL_STRUCT *pll)
{
    static float SMO_We = 0.0f;
    float Sine, Cosine;

    SMO.data.Input_We = SMO_We;

    SMO.alpha.Input_Ix = current.Real_Ialpha;
    SMO.alpha.Input_Ux = foc.Ualpha;
    SMO.beta.Input_Ix  = current.Real_Ibeta;
    SMO.beta.Input_Ux  = foc.Ubeta;

    SMO_Loop(&SMO);

    /* ★ OutRe已经是电角度，不要再乘Poles！*/
    fast_sin_cos(pll->go.OutRe, &Sine, &Cosine);
    pll->go.Error = -(SMO.alpha.Output_Ex * Cosine +
                      SMO.beta.Output_Ex  * Sine);
    PLL_Loop(pll);

    SMO_We = pll->go.OutWe * motor.Poles;
}

/* ============================ 11. Transfer_Init ========================= */
static void Transfer_Init(void)
{
    /* 1.电流环参数（取自 UserData_Parameter.h）*/
    Current_D.T = PMSM_RUN_T;
    Current_D.Kp = 0.152f;
    Current_D.Ki = 1.05f;
    Current_D.Kd = 0.0f;
    Current_D.Wc = 100.0f;
    Current_D.OutMax = 10.0f;
    Current_D.OutMin = -10.0f;
    Current_D.IntMax = 3.0f;
    Current_D.IntMin = -3.0f;
    PID_Init(&Current_D);

    Current_Q.T = PMSM_RUN_T;
    Current_Q.Kp = 0.152f;
    Current_Q.Ki = 1.05f;
    Current_Q.Kd = 0.0f;
    Current_Q.Wc = 100.0f;
    Current_Q.OutMax = 10.0f;
    Current_Q.OutMin = -10.0f;
    Current_Q.IntMax = 3.0f;
    Current_Q.IntMin = -3.0f;
    PID_Init(&Current_Q);

    /* 2.转速环参数（Response = 5）*/
    Velocity.T = PMSM_RUN_T * 5.0f;
    Velocity.Kp = 0.06f;
    Velocity.Ki = 0.0f;
    Velocity.Kd = 0.0f;
    Velocity.Wc = 100.0f;
    Velocity.OutMax = 10.f;
    Velocity.OutMin = -10.f;
    Velocity.IntMax = 15000.0f;
    Velocity.IntMin = -15000.0f;
    PID_Init(&Velocity);

    /* 5.滤波器参数 */
    LPF_D.T = PMSM_RUN_T;
    LPF_D.Wc = 31415.96f;
    LPF_Init(&LPF_D);

    LPF_Q.T = PMSM_RUN_T;
    LPF_Q.Wc = 31415.96f;
    LPF_Init(&LPF_Q);

    LPF_encoder.T = PMSM_RUN_T;
    LPF_encoder.Wc = 50.0f;
    LPF_Init(&LPF_encoder);

    /* 6.锁相环参数 */
    PLL_encoder.T  = PMSM_RUN_T;
    PLL_encoder.Kp = 150.0f;
    PLL_encoder.Ki = 20000.0f;
    PLL_Init(&PLL_encoder);

    /* 14.最速控制LTD（强拖速度平滑）*/
    LTD.T = PMSM_RUN_T;
    LTD.r = SL_LTD_R;
    LTD_Init(&LTD);

    /* 18.滑模观测器 */
    SMO.T  = PMSM_RUN_T;
    SMO.Rs = motor.Rs;
    SMO.Ld = motor.Ld;
    SMO.Lq = motor.Lq;
    SMO.Wc = 2000.0f;
    //SMO.h  = 3.0f;
		SMO.h  = 1.5f;
    SMO.IntMax = 20;
    SMO.IntMin = -20;
    SMO_Init(&SMO);

    /* 20.无感权重数据（对应 UserData_Parameter.h 的无感参数，单位 rad/s 机械）
     *   低于 Start  ：不跑观测器，纯开环
     *   AbsMin~AbsMax：权重 0→1 平滑过渡
     *   高于 AbsMax ：完全用观测角
     *   ★ AbsMax 必须小于强拖目标转速，否则永远不会切进闭环
     *     SL_OPEN_RPM=800rpm -> 83.8 rad/s，这里取 80 留 5% 余量  */
    value.Sensorless_Start  = 30.0f;    /* 约 573 rpm（EMF 0.66V，信噪比 2.3）开始观测 */
    value.Sensorless_AbsMin = 50.0f;    /* 约 764 rpm（EMF 0.88V）开始融合 */
    value.Sensorless_AbsMax = 100.0f;   /* 约 1050 rpm（EMF 1.21V）权重到 1 */

    foc.sine   = 0.0f;
    foc.cosine = 1.0f;
}

/* ============================ 12. Offset_Current_Init ===================
 * 原工程：主循环里采 24 点、每点延时 2ms（阻塞）
 * 本工程：10kHz 中断里每拍采一次，采满 1000 拍（100ms）后取平均
 * （名字与计算方式保持一致，只是不能在这里 Delay）
 * ========================================================================*/
static void Offset_Current_Init(void)
{
    s_OffSum0 += (float)ADC_InjectedValues[1];
    s_OffSum1 += (float)ADC_InjectedValues[2];

    if (++s_OffCnt >= 1000)
    {
        current.Current_offset0 = s_OffSum0 / (float)s_OffCnt;
        current.Current_offset1 = s_OffSum1 / (float)s_OffCnt;

        /* 计算电流最后增益（与 SguanFOC 完全一致的公式）*/
        current.Final_Gain = MCU_VOLTAGE /
            (ADC_PRECISION * AMPLIFIER * SAMPLING_RS);

        s_OffSum0 = 0.0f;
        s_OffSum1 = 0.0f;
        s_OffCnt  = 0;
        value.Status = MOTOR_STATUS_ALIGN;
        s_AlignCnt   = 0;
    }
}

/* ============================ 13. Current_ReadIabc / Current_Tick ======= */
static void Current_ReadIabc(void)
{
    float I0 = ((float)ADC_InjectedValues[1] - current.Current_offset0)
             * current.Final_Gain * motor.Current_Dir0;
    float I1 = ((float)ADC_InjectedValues[2] - current.Current_offset1)
             * current.Final_Gain * motor.Current_Dir1;
    float I2 = -(I0 + I1);

    if (motor.Current_Num == 0)                     /* AB 采样 */
    {
        current.Real_Ia = (motor.Motor_Dir == 1) ? I0 : I1;
        current.Real_Ib = (motor.Motor_Dir == 1) ? I1 : I0;
        current.Real_Ic = I2;
    }
    else if (motor.Current_Num == 1)                /* AC 采样（本板 JDR2=A JDR3=C）*/
    {
        current.Real_Ia = (motor.Motor_Dir == 1) ? I0 : I2;
        current.Real_Ib = (motor.Motor_Dir == 1) ? I2 : I0;
        current.Real_Ic = I1;
    }
    else                                            /* BC 采样 */
    {
        current.Real_Ia = (motor.Motor_Dir == 1) ? I2 : I0;
        current.Real_Ib = (motor.Motor_Dir == 1) ? I0 : I2;
        current.Real_Ic = I1;
    }
}

static void Current_Tick(void)
{
    float Id_Raw, Iq_Raw;

    Current_ReadIabc();

    clarke(&current.Real_Ialpha, &current.Real_Ibeta,
           current.Real_Ia, current.Real_Ib);

    park(&Id_Raw, &Iq_Raw,
         current.Real_Ialpha, current.Real_Ibeta,
         foc.sine, foc.cosine);

    Transfer_LPF_Loop(&LPF_D, Id_Raw);
    Transfer_LPF_Loop(&LPF_Q, Iq_Raw);
	
    current.Real_Id = LPF_D.filter.Output;
    current.Real_Iq = LPF_Q.filter.Output;
}

/* ============================ 14. Encoder_Virtual_Angle =================
 * 开环强拖：LTD 速度平滑 + 角度梯形积分（对应 SguanFOC.c 第 875 行）
 * ========================================================================*/
static void Encoder_Virtual_Angle(void)
{
    static float Last_Real_Speed = 0.0f;

//    /* 1.IF 开环：给固定 Iq，速度由 LTD 渐进到 Target_Speed */
//    foc.Target_Iq = foc.Target_IF_Iq;

    Transfer_LTD_Loop(&LTD, &foc.Speed_in, foc.Target_Speed);
    encoder.Real_Speed = foc.Speed_in;

    /* 2.角度梯形积分 */
    Value_Rad_Loop(&encoder.Real_Pos,
                   encoder.Real_Speed,
                   Last_Real_Speed,
                   PMSM_RUN_T);

    encoder.Real_We = encoder.Real_Speed * motor.Poles;
    encoder.Real_Re = Value_normalize(encoder.Real_Pos * motor.Poles);

    Last_Real_Speed = encoder.Real_Speed;   /* 原工程漏了这句，这里补上 */
}

/* ============================ 15. Encoder_Sensorless_Tall ===============
 * SMO + PLL + 低速域/高速域权重融合（与 SguanFOC.c 第 1007 行一致）
 *   低速域 Low_angle  : 开环积分角
 *   高速域 High_angle : SMO+PLL 观测角
 *   Sensorless_Gain   : 按转速在 0~1 之间平滑过渡
 * ========================================================================*/
static void Encoder_Sensorless_Tall(void)
{
    static float Last_Speed_in = 0.0f;
    static float Last_High_Wm  = 0.0f;
    float Speed_Abs;
    float Normalized_Gain;
    float Speed;
    static uint8_t s_SmoSync     = 0;      /* ★ 新增：是否已同步 */
    /* 1.速度设置处理*/
    Transfer_LTD_Loop(&LTD, &foc.Speed_in, foc.Target_Speed);

	/* 2.按当前反馈速度算权重：速度低强拖比重大，速度高观测比重高 */
    Speed_Abs = Value_Gain_Get(&value.Sensorless_Gain,
                               encoder.Real_Speed,
                               value.Sensorless_AbsMax,
                               value.Sensorless_AbsMin);
    Normalized_Gain = 1.0f - value.Sensorless_Gain;

    if (Speed_Abs >= value.Sensorless_Start)
    {
        if (!s_SmoSync)                    /* ★ 首次进入观测区 */
        {
            s_SmoSync = 1;

            /* ★ 只同步“角度”和“速度”，不动 PI 内部状态 */
            PLL_encoder.go.OutRe = value.Low_angle;      /* 角度对齐 */
            PLL_encoder.go.Re_i  = foc.Speed_in;         /* 上一拍速度 = 当前速度 */
            PLL_encoder.go.OutWe = foc.Speed_in;         /* 速度对齐 */
            PLL_encoder.go.We_i  = 0.0f;                 /* ★ 误差积分清零，不要设成速度 */

            /* SMO 电流观测器从实际电流起步 */
            SMO.alpha.Output_Ix = current.Real_Ialpha;
            SMO.beta.Output_Ix  = current.Real_Ibeta;
        }

        Transfer_SMO_Loop(&PLL_encoder);
    }
    else
    {
        s_SmoSync = 0;                    
    }
    /* 4.高速域数值更新（与源程序一致：High_Wm是机械角速度）*/
    value.High_Wm    = PLL_encoder.go.OutWe / (float)motor.Poles;  /* ★ 电角速度/极对数 = 机械角速度 */
    value.High_angle = PLL_encoder.go.OutRe;                       /* 电角度 */

		#if 0   /* ★ 排查用：atan2 直接算角度，绕过 PLL.OutRe */
    {
        /* Eα = -ωφ·sin(θe)，Eβ = ωφ·cos(θe)  →  θe = atan2(-Eα, Eβ) */
        float fThetaE = atan2f(-SMO.alpha.Output_Ex, SMO.beta.Output_Ex);

        value.High_angle = Value_normalize(fThetaE);
    }
		#else
				value.High_angle = PLL_encoder.go.OutRe;   /* 原来的 */
		#endif
		
		
		
    /* 5.速度域判断和角度切换（与源程序Encoder_Sensorless_Tall的IF模式一致）*/
    static uint8_t count_high = 0;
    float Re;  /* 最终电角度 */

    if (Speed_Abs <= value.Sensorless_AbsMin)
    {
        /* 低速域：用开环机械角速度积分 */
        Value_Rad_Loop(&value.Low_angle, foc.Speed_in, Last_Speed_in, PMSM_RUN_T);
        value.Low_Wm = foc.Speed_in;

        /* 电角度 = 机械角度 × 极对数 */
        Re = Value_normalize(value.Low_angle * (float)motor.Poles);

        /* 速度用机械速度 */
        Speed = foc.Speed_in;

        count_high = 0;
    }
    else
    {
        /* 高速域：用观测器角度 */
        Re = value.High_angle;  /* 已经是电角度 */
        Speed = value.High_Wm;  /* 已经是机械角速度 */

        /* 第一次进入高速域时，对齐积分项 */
        if (count_high == 0)
        {
            float error = Re - Value_normalize(value.Low_angle * (float)motor.Poles);
            float sine_error, cosine_error;
            fast_sin_cos(error, &sine_error, &cosine_error);

            /* 无扰接手：预置电流环积分项 */
            Current_D.run.Io = foc.Uq_in * sine_error + foc.Ud_in * cosine_error;
            Current_Q.run.Io = foc.Uq_in * cosine_error - foc.Ud_in * sine_error;

            count_high = 1;
        }

        /* 更新Low_angle，保持一致性（机械角度 = 电角度 / 极对数）*/
        value.Low_angle = Re / (float)motor.Poles;
        value.Low_Wm = value.High_Wm;
    }

    /* 6.速度低通滤波 */
    Transfer_LPF_Loop(&LPF_encoder, Speed);
    encoder.Real_Speed = LPF_encoder.filter.Output;

    /* 7.角度输出（直接用电角度Re，不做加权融合）*/
    encoder.Real_We = encoder.Real_Speed * (float)motor.Poles;
    encoder.Real_Re = Re;  /* 电角度 */

    /* 8.历史值 */
    Last_Speed_in = foc.Speed_in;
    Last_High_Wm  = value.High_Wm;
}

/* 角度解算函数表（对应原工程 Encoder_Tick[]，去掉多余的观测器）*/
static void (*const Encoder_Tick[])(void) =
{
    Encoder_Virtual_Angle,      // 0 = I/f 开环强拖
    Encoder_Sensorless_Tall     // 1 = SMO 无感
};

/* ============================ 16. Control_Tick ========================== */
static void Control_Tick(void)
{
    /* 1.转速环：输出 Iq 给定 */
    Transfer_PID_Loop(&Velocity, foc.Target_Speed, encoder.Real_Speed);
    foc.Target_Iq = Velocity.run.Output;

    /* 2.电流环：输出 Ud/Uq */
    Transfer_PID_Loop(&Current_D, foc.Target_Id, current.Real_Id);
    foc.Ud_in = Current_D.run.Output;

    Transfer_PID_Loop(&Current_Q, foc.Target_Iq, current.Real_Iq);
    foc.Uq_in = Current_Q.run.Output;
}

/* ============================ 17. PWM_Tick ============================= */
static void PWM_Tick(float U_alpha, float U_beta)
{
    uint32_t Duty_u, Duty_v, Duty_w;

#if CONFIG_PWM
    SPWM(U_alpha, U_beta, &foc.Du, &foc.Dv, &foc.Dw);
#else
    SVPWM(U_alpha, U_beta, &foc.Du, &foc.Dv, &foc.Dw);
#endif

    /* 三相归一化电压值限幅 */
    Value_Limit(&foc.Du, 1.0f, 0.0f);
    Value_Limit(&foc.Dv, 1.0f, 0.0f);
    Value_Limit(&foc.Dw, 1.0f, 0.0f);

    /* 电机占空比输出 */
    if (motor.PWM_Dir == 1)
    {
        Duty_u = (uint32_t)(foc.Du * (float)motor.Duty);
        Duty_v = (uint32_t)(foc.Dv * (float)motor.Duty);
        Duty_w = (uint32_t)(foc.Dw * (float)motor.Duty);
    }
    else
    {
        Duty_u = (uint32_t)((1.0f - foc.Du) * (float)motor.Duty);
        Duty_v = (uint32_t)((1.0f - foc.Dv) * (float)motor.Duty);
        Duty_w = (uint32_t)((1.0f - foc.Dw) * (float)motor.Duty);
    }

    if (motor.Motor_Dir == -1)              /* 判断电机方向并修改(AB相序交换) */
    {
        uint32_t duty_temp = Duty_u;
        Duty_u = Duty_v;
        Duty_v = duty_temp;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duty_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Duty_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duty_w);

}

/* ============================ 18. Status_Switch_Loop ====================
 * 状态机：只负责决定「用哪个角度解算函数」与「电流/速度给定」
 * ========================================================================*/
static void Status_Switch_Loop(void)
{
    switch (value.Status)
    {
    /*---------------- 0x01 START：零漂标定 ----------------*/
    case MOTOR_STATUS_START:
        Offset_Current_Init();
        foc.Target_Id = 0.0f;
        foc.Target_Iq = 0.0f;
        foc.Target_Speed = 0.0f;
        Encoder_Tick_Index = 1;             /* 与 SguanFOC 一致：全程 Encoder_Sensorless_Tall */

				break;

    /*---------------- 0x02 ALIGN：锁 θ=0，Id 励磁 ----------------*/
    case MOTOR_STATUS_ALIGN:
        foc.sine   = 0.0f;
        foc.cosine = 1.0f;
        foc.Target_Id    = SL_ALIGN_I;
        foc.Target_Iq    = 0.0f;
        foc.Target_Speed = 0.0f;
        Encoder_Tick_Index = 1;             /* 与 SguanFOC 一致 */

        LTD.go.Output  = 0.0f;
        LTD.go.Last_i  = 0.0f;

        /* 复位 Encoder_Sensorless_Tall 用到的角度/权重状态，避免上次残留 */
        value.Low_angle       = 0.0f;
        value.High_angle      = 0.0f;
        value.Low_Wm          = 0.0f;
        value.High_Wm         = 0.0f;
        value.Sensorless_Gain = 0.0f;

        encoder.Real_Speed = 0.0f;
        encoder.Real_Pos   = 0.0f;
        encoder.Real_We    = 0.0f;
        encoder.Real_Re    = 0.0f;

        if (++s_AlignCnt >= (uint16_t)((uint32_t)SL_ALIGN_MS * 20U))
        {
            s_AlignCnt = 0;
            PLL_Init(&PLL_encoder);
            SMO_Init(&SMO);

            value.Status = MOTOR_STATUS_OPEN;
        }
        break;

    /*---------------- 0x03 OPEN：I/f 强拖 ----------------*/
    case MOTOR_STATUS_OPEN:
        foc.Target_Id    = 0.0f;
        foc.Target_Iq    = SL_OPEN_IQ;      /* 显式给 Iq */
        Encoder_Tick_Index = 1;             /* 与 SguanFOC 一致：权重融合角 */

        /* 权重辨识结果：观测器已完全接管 -> 进闭环
         * （比"角度误差判据"更可靠：融合权重就是按转速平滑过渡的）*/
        if (value.Sensorless_Gain >= 0.95f)
        {
            Velocity.run.Io = SL_OPEN_IQ;   /* 无扰接手：积分预置成当前 Iq */
            value.Status = MOTOR_STATUS_IDLE;
        }
        break;

    /*---------------- 0x05 IDLE：无感闭环 ----------------*/
    case MOTOR_STATUS_IDLE:
    default:
        foc.Target_Id = 0.0f;
        Encoder_Tick_Index = 1;             /* 角度用观测角 */
        break;
    }
}

/* ============================ 19. Status_RUN_Loop ======================
 * 与 SguanFOC 保持一致：目标转速由电位器或固定值给出
 * ========================================================================*/
static void Status_RUN_Loop(void)
{
	/* 目标转速：电位器或固定值 */
	if (value.Status >= MOTOR_STATUS_OPEN)
	{
			foc.Target_Speed = (float)ADC_InjectedValues[3] * (2000.0f / 4095.0f)
											 * Value_2PI / 60.0f;
	}

}

/* ============================ 20. Sguan_Calculate_High_Loop ============ */
/* ==================== VOFA+ JustFloat 数据上传 ====================
 * 协议：8 个 float（小端）+ 4 字节帧尾 0x00 0x00 0x80 0x7F
 * VOFA+ 设置：协议 JustFloat，通道数 8，波特率 921600，8N1
 * ================================================================ */
#define VOFA_NUM   20

static uint8_t s_VofaBuf[(VOFA_NUM + 1) * 4];

static void VOFA_Send(void)
{
    float *pF = (float *)s_VofaBuf;

    /* 上一帧 DMA 还没发完就跳过，避免覆盖缓冲区 */
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

		pF[0] = SMO.alpha.Output_Ex;          /* Eα */
		pF[1] = SMO.beta.Output_Ex;           /* Eβ */
		pF[2] = PLL_encoder.go.Error;         /* 角度误差信号 */
		pF[3] = PLL_encoder.go.OutWe;         /* 观测电角速度 rad/s */
		pF[4] = PLL_encoder.go.OutRe * 57.3f; /* 观测电角度（PLL输出）*/
		pF[5] = value.Low_angle * 57.3f;      /* 开环机械角度 */
		pF[6] = value.Sensorless_Gain;        /* 权重 */
		pF[7] = encoder.Real_Speed * 9.55f;   /* 机械转速 rpm */
		pF[8] = value.High_Wm * 57.3f;        /* 高速域机械角速度 */
		pF[9] = value.High_angle * 57.3f;     /* 高速域电角度（atan2或PLL）*/
		pF[10] = value.Low_Wm * 57.3f;        /* 低速域机械角速度 */
		pF[11] = encoder.Real_Re * 57.3f;     /* ★ 最终电角度（实际用于FOC）*/
		pF[12] = encoder.Real_Speed;          /* 机械角速度 rad/s */
		pF[13] = encoder.Real_We;             /* 电角速度 rad/s */
		
		pF[15] = (float)value.Status;
    /* 帧尾（小端 0x7F800000）*/
    s_VofaBuf[VOFA_NUM * 4 + 0] = 0x00;
    s_VofaBuf[VOFA_NUM * 4 + 1] = 0x00;
    s_VofaBuf[VOFA_NUM * 4 + 2] = 0x80;
    s_VofaBuf[VOFA_NUM * 4 + 3] = 0x7F;

    HAL_UART_Transmit_DMA(&huart1, s_VofaBuf, (uint16_t)((VOFA_NUM + 1) * 4));
}

static void Sguan_Calculate_High_Loop(void)
{
    /* 1.母线电压更新 */
    foc.Real_VBUS = (float)ADC_InjectedValues[0] * ADC_VBUS_TO_VOLT;
    if ((foc.Real_VBUS < 1.0f) || (foc.Real_VBUS > 100.0f))
    {
        foc.Real_VBUS = motor.VBUS;
    }

    /* 2.状态机（决定角度源与给定）*/
    Status_Switch_Loop();
    Status_RUN_Loop();

    /* 3.电机相线和各轴电流计算（用上一拍角度做 Park）*/
    Current_Tick();

    /* 4.电机角度和角速度计算 */
    Encoder_Tick[Encoder_Tick_Index]();
    fast_sin_cos(encoder.Real_Re, &foc.sine, &foc.cosine);

    /* 5.控制器（对齐/强拖阶段给固定电流，闭环给速度环输出）*/
    if (value.Status == MOTOR_STATUS_IDLE)
    {
        Control_Tick();
    }
    else
    {
			
        /* 开环阶段：只用电流环跟踪固定给定 */
        Transfer_PID_Loop(&Current_D, foc.Target_Id, current.Real_Id);
        foc.Ud_in = Current_D.run.Output;
				//foc.Ud_in = 0;

        Transfer_PID_Loop(&Current_Q, foc.Target_Iq, current.Real_Iq);
        foc.Uq_in = Current_Q.run.Output;
				//foc.Uq_in = 2.0f;
    }


    /* 6.反 Park + 调制输出 */
    ipark(&foc.Ualpha, &foc.Ubeta, foc.Ud_in, foc.Uq_in, foc.sine, foc.cosine);
    PWM_Tick(foc.Ualpha / foc.Real_VBUS, foc.Ubeta / foc.Real_VBUS);

    /* 7.VOFA 上传：20kHz / 200 = 100Hz（921600 波特下 36 字节仅需 0.39ms）*/
    {
        static uint16_t s_VofaCnt = 0;

        if (++s_VofaCnt >= 20)
        {
            s_VofaCnt = 0;
            VOFA_Send();
        }
    }
}

/* ============================ 21. 初始化 =============================== */
void SguanFOC_Init(void)
{
    /* 电机与硬件参数 */
    motor.Poles       = MOTOR_POLES;
    motor.Rs          = MOTOR_RS;
    motor.Ld          = MOTOR_LD;
    motor.Lq          = MOTOR_LQ;
    motor.Flux        = MOTOR_FLUX;
    motor.VBUS        = MOTOR_VBUS;
    motor.Duty        = PMSM_PWM_ARR;
    motor.PWM_Dir     = -1;         /* TIM1 是 PWM2 模式 */
    motor.Motor_Dir   = 1;
    motor.Current_Num = 1;          /* AC 采样：JDR2=A, JDR3=C */
    motor.Current_Dir0 = 1.0f;
    motor.Current_Dir1 = 1.0f;

    /* 电流 */
    current.Current_offset0 = ADC_CUR_OFFSET;
    current.Current_offset1 = ADC_CUR_OFFSET;
    current.Final_Gain      = MCU_VOLTAGE /
        (ADC_PRECISION * AMPLIFIER * SAMPLING_RS);
    current.Real_Ia = current.Real_Ib = current.Real_Ic = 0.0f;
    current.Real_Ialpha = current.Real_Ibeta = 0.0f;
    current.Real_Id = current.Real_Iq = 0.0f;

    /* FOC */
    foc.Ud_in = foc.Uq_in = 0.0f;
    foc.Ualpha = foc.Ubeta = 0.0f;
    foc.Du = foc.Dv = foc.Dw = 0.5f;
    foc.Real_VBUS = motor.VBUS;
    foc.Target_Id = 0.0f;
    foc.Target_Iq = 0.0f;
    foc.Target_Speed = 0.0f;
    foc.Target_IF_Iq = SL_OPEN_IQ;
    foc.Target_VF_Uq = 0.0f;
    foc.Speed_in = 0.0f;

    /* 角度 */
    encoder.Real_Speed = 0.0f;
    encoder.Real_Pos   = 0.0f;
    encoder.Real_We    = 0.0f;
    encoder.Real_Re    = 0.0f;
    encoder.Real_offset = 0.0f;

    /* 状态机与权重融合数据 */
    value.Status   = MOTOR_STATUS_START;
    value.ThetaErr = 0.0f;

    value.Low_angle       = 0.0f;
    value.High_angle      = 0.0f;
    value.Low_Wm          = 0.0f;
    value.High_Wm         = 0.0f;
    value.Sensorless_Gain = 0.0f;

    s_AlignCnt = s_OffCnt = 0;
    s_OffSum0 = s_OffSum1 = 0.0f;

    Transfer_Init();

    s_InitDone = 1;
}

float g_TestThetaStep = 0.001f;
/* ============================ 22. 入口 ================================= */
void SguanFOC_High_Loop(void)
{
    if (!s_InitDone)
    {
        SguanFOC_Init();
    }
		

    Sguan_Calculate_High_Loop();
}
