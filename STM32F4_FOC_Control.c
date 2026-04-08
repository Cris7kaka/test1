/**
 * @file STM32F4_FOC_Control.c
 * @brief STM32F446 FOC电机控制算法实现
 * @details 实现电流环、速度环、IF模式、VF模式控制，支持SVPWM零序注入
 * @version 1.0
 * @date 2026-04-08
 * 
 * 硬件平台: STM32F446
 * 电机ID: 0
 */

#include "STM32F4_FOC_Control.h"
#include <string.h>
#include <math.h>

/* ==================== 全局变量 ==================== */
static FOC_Controller_t foc_controllers[1]; //!< 控制器实例数组 (仅支持motor_id=0)

/* ==================== 辅助数学函数 (替代 math.h) ==================== */

/**
 * @brief 快速绝对值
 */
static inline float Fast_Fabs(float x)
 {
    return fabsf(x);
}

/**
 * @brief 快速最小值
 */
static inline float Fast_Fmin(float a, float b) 
{
    return fminf(a, b);
}

/**
 * @brief 快速最大值
 */
static inline float Fast_Fmax(float a, float b) {
    return fmaxf(a, b);
}

/**
 * @brief 快速平方根 (使用标准库函数)
 */
static float Fast_Sqrt(float x) 
{
    if (x <= 0.0f) return 0.0f;
    return sqrtf(x);
}

/* ==================== 辅助函数实现 ==================== */

/**
 * @brief 归一化角度到[0, 2π]
 */
float Normalize_Angle(float angle) {
    // 避免使用 fmodf，通过加减 2PI 来归一化
    // 对于大多数电机控制应用，角度不会在一次迭代中偏离太远
    while (angle >= _2PI) {
        angle -= _2PI;
    }
    while (angle < 0.0f) {
        angle += _2PI;
    }
    return angle;
}

/**
 * @brief 快速正弦计算 (使用标准库函数)
 */
float Fast_Sin(float angle) 
{
    return sinf(angle);
}

/**
 * @brief 快速余弦计算 (使用标准库函数)
 */
float Fast_Cos(float angle)
 {
    return cosf(angle);
}

/**
 * @brief 同时计算正弦和余弦
 */
void Fast_SinCos(float angle, float* sin_val, float* cos_val)
 {
    // 由于需要返回两个值，这里仍然需要使用指针
    *sin_val = sinf(angle);
    *cos_val = cosf(angle);
}


/* ==================== 低通滤波器实现 ==================== */

/**
 * @brief 初始化低通滤波器
 */
void LPF_Init(LowPassFilter_t* lpf, float time_constant) {
    lpf->Tf = time_constant;
    lpf->prev_output = 0.0f;
    lpf->prev_input = 0.0f;
}

/**
 * @brief 低通滤波器计算 (一阶IIR滤波器)
 */
float LPF_Compute(LowPassFilter_t* lpf, float input, float dt) {
    if (lpf->Tf <= 0.0f || dt <= 0.0f) {
        return input;  // 无滤波
    }
    
    // 计算滤波系数
    float alpha = dt / (lpf->Tf + dt);
    
    // 滤波计算
    float output = lpf->prev_output + alpha * (input - lpf->prev_output);
    
    // 更新状态
    lpf->prev_output = output;
    lpf->prev_input = input;
    
    return output;
}

/* ==================== Clarke/Park变换实现 ==================== */

/**
 * @brief Clarke变换 (abc -> αβ)
 * @note 假设三相平衡: Ia + Ib + Ic = 0
 */
AlphaBeta_t Clarke_Transform(Phase_t abc) {
    AlphaBeta_t ab;
    
    // Clarke变换公式
    ab.alpha = abc.a;
    ab.beta = _1_SQRT3 * abc.a + _2_SQRT3 * abc.b;
    
    return ab;
}

/**
 * @brief Park变换 (αβ -> dq)
 */
DQ_t Park_Transform(AlphaBeta_t ab, float angle_el) {
    DQ_t dq;
    float sin_angle, cos_angle;
    
    Fast_SinCos(angle_el, &sin_angle, &cos_angle);
    
    // Park变换公式
    dq.d = cos_angle * ab.alpha + sin_angle * ab.beta;
    dq.q = -sin_angle * ab.alpha + cos_angle * ab.beta;
    
    return dq;
}

/**
 * @brief 逆Park变换 (dq -> αβ)
 */
AlphaBeta_t Inverse_Park_Transform(DQ_t dq, float angle_el) {
    AlphaBeta_t ab;
    float sin_angle, cos_angle;
    
    Fast_SinCos(angle_el, &sin_angle, &cos_angle);
    
    // 逆Park变换公式
    ab.alpha = cos_angle * dq.d - sin_angle * dq.q;
    ab.beta = sin_angle * dq.d + cos_angle * dq.q;
    
    return ab;
}

/* ==================== SVPWM调制实现 (带零序注入) ==================== */

/**
 * @brief SVPWM调制 (带零序注入)
 * @details 使用零序注入方法提高直流母线电压利用率
 */
void SVPWM_Modulation(FOC_Controller_t* controller, float u_alpha, float u_beta)
 {
    float ua, ub, uc;
    float u_min, u_max, u_offset;
    float voltage_limit = VOLTAGE_LIMIT;
    
    // 逆Clarke变换 (αβ -> abc)
    ua = u_alpha;
    ub = -0.5f * u_alpha + SQRT3_2 * u_beta;
    uc = -0.5f * u_alpha - SQRT3_2 * u_beta;
    
    // 零序注入: 找到最小和最大值
    u_min = Fast_Fmin(ua, Fast_Fmin(ub, uc));
    u_max = Fast_Fmax(ua, Fast_Fmax(ub, uc));
    
    // 计算零序分量 (中点钳位)
    u_offset = -(u_max + u_min) / 2.0f;
    
    // 注入零序分量
    ua += u_offset;
    ub += u_offset;
    uc += u_offset;
    
    // 限制在电压范围内
    float scale = 1.0f;
    float abs_max = Fast_Fmax(Fast_Fabs(ua), Fast_Fmax(Fast_Fabs(ub), Fast_Fabs(uc)));
    if (abs_max > voltage_limit / 2.0f) {
        scale = (voltage_limit / 2.0f) / abs_max;
    }
    
    // 应用缩放并转换为PWM占空比 (0-1范围)
    controller->pwm_a = (ua * scale + voltage_limit / 2.0f) / voltage_limit;
    controller->pwm_b = (ub * scale + voltage_limit / 2.0f) / voltage_limit;
    controller->pwm_c = (uc * scale + voltage_limit / 2.0f) / voltage_limit;
    
    // 确保PWM值在[0, 1]范围内
    controller->pwm_a = Fast_Fmax(0.0f, Fast_Fmin(1.0f, controller->pwm_a));
    controller->pwm_b = Fast_Fmax(0.0f, Fast_Fmin(1.0f, controller->pwm_b));
    controller->pwm_c = Fast_Fmax(0.0f, Fast_Fmin(1.0f, controller->pwm_c));
}

/* ==================== 电流采样实现 ==================== */

// 全局ADC句柄（需要在主程序中初始化）
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/**
 * @brief 读取三相电流
 * @note 此函数从ADC获取电流值
 */
void Read_Phase_Currents(FOC_Controller_t* controller) {
    // 静态变量用于存储偏置校准状态和偏置值
    static uint16_t calibration_count = 0;
    static float offset_a_sum = 0.0f;
    static float offset_b_sum = 0.0f;
    static float offset_c_sum = 0.0f;
    static bool calibrated = false;
    
    // 从ADC获取转换结果
    uint16_t adc_a = HAL_ADC_GetValue(&hadc1);  // A相ADC
    uint16_t adc_b = HAL_ADC_GetValue(&hadc2);  // B相ADC
    uint16_t adc_c = HAL_ADC_GetValue(&hadc3);  // C相ADC
    
    // 转换为电压值
    float voltage_a = (float)adc_a / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_b = (float)adc_b / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_c = (float)adc_c / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    
    if (!calibrated && calibration_count < 5000) {
        // 校准阶段：采集5000个样本计算偏置
        // 累加用于偏置计算
        offset_a_sum += voltage_a;
        offset_b_sum += voltage_b;
        offset_c_sum += voltage_c;
        
        calibration_count++;
        
        // 设置默认值直到校准完成
        controller->phase_currents.a = 0.0f;
        controller->phase_currents.b = 0.0f;
        controller->phase_currents.c = -(controller->phase_currents.a + controller->phase_currents.b);
        
        if (calibration_count >= 5000) {
            // 计算偏置值（均值）
            controller->offset_a = offset_a_sum / 5000.0f;
            controller->offset_b = offset_b_sum / 5000.0f;
            controller->offset_c = offset_c_sum / 5000.0f;
            calibrated = true;
            
            // 重新初始化校准变量
            calibration_count = 0;
            offset_a_sum = 0.0f;
            offset_b_sum = 0.0f;
            offset_c_sum = 0.0f;
        }
    } else {
        // 正常运行阶段：使用偏置校准后的电流值
        // 转换为电流值 (考虑偏置和增益)
        controller->phase_currents.a = (voltage_a - controller->offset_a) / CURRENT_GAIN;
        controller->phase_currents.b = (voltage_b - controller->offset_b) / CURRENT_GAIN;
        controller->phase_currents.c = (voltage_c - controller->offset_c) / CURRENT_GAIN;
    }
}

/* ==================== 电流环FOC控制 (id=0策略) ==================== */

/**
 * @brief 电流环FOC控制 (id=0策略)
 * @details 通过电流反馈和位置反馈形成闭环控制
 */
void Current_Loop_FOC(FOC_Controller_t* controller, float iq_ref, float dt) {
    if (!controller || !controller->enabled) return;
    
    // 1. 读取三相电流（电流反馈）
    Read_Phase_Currents(controller);
    
    // 2. Clarke变换 (abc -> αβ)
    controller->alpha_beta_current = Clarke_Transform(controller->phase_currents);
    
    // 3. Park变换 (αβ -> dq)，使用位置反馈（电角度）进行坐标变换
    controller->dq_current = Park_Transform(controller->alpha_beta_current, controller->electrical_angle);
    
    // 4. 低通滤波
    controller->dq_current.d = LPF_Compute(&controller->lpf_current_d, controller->dq_current.d, dt);
    controller->dq_current.q = LPF_Compute(&controller->lpf_current_q, controller->dq_current.q, dt);
    
    // 5. 计算电流误差（电流反馈与参考值比较）
    float error_q = iq_ref - controller->dq_current.q;
    float error_d = 0.0f - controller->dq_current.d;  // 目标d轴电流通常为0 (id=0策略)
    
    // 6. PI控制器计算dq轴电压
    controller->dq_voltage.q = PI_Compute(&controller->pi_current_q, error_q, dt);
    controller->dq_voltage.d = PI_Compute(&controller->pi_current_d, error_d, dt);
    
    // 7. 限制dq轴电压幅值
    float voltage_mag = Fast_Sqrt(controller->dq_voltage.d * controller->dq_voltage.d + 
                              controller->dq_voltage.q * controller->dq_voltage.q);
    if (voltage_mag > VOLTAGE_LIMIT) {
        controller->dq_voltage.d *= (VOLTAGE_LIMIT / voltage_mag);
        controller->dq_voltage.q *= (VOLTAGE_LIMIT / voltage_mag);
    }
    
    // 8. 逆Park变换 (dq -> αβ)，使用位置反馈（电角度）进行坐标变换
    AlphaBeta_t voltage_ab = Inverse_Park_Transform(controller->dq_voltage, controller->electrical_angle);
    
    // 9. SVPWM调制 (带零序注入)
    SVPWM_Modulation(controller, voltage_ab.alpha, voltage_ab.beta);
}

/* ==================== 速度环控制 ==================== */

/**
 * @brief 速度环PI控制
 * @param controller FOC控制器实例指针
 * @param velocity_ref 目标速度 (rad/s 或 rpm，取决于单位定义)
 * @param dt 控制周期 (s)
 * @return q轴电流参考值 (A)
 */
float Velocity_Loop_Control(FOC_Controller_t* controller, float velocity_ref, float dt) {
    if (!controller || !controller->enabled) return 0.0f;
    
    // 1. 获取速度反馈并计算误差
    // 注意: shaft_velocity 需要在其他地方(如编码器中断或观测器)实时更新
    float velocity_error = velocity_ref - controller->shaft_velocity;
    
    // 2. 速度PI控制器计算q轴电流参考值
    float iq_ref = PI_Compute(&controller->pi_velocity, velocity_error, dt);
    
    // 3. 限制q轴电流输出，保护电机和驱动器
    if (iq_ref > CURRENT_LIMIT) {
        iq_ref = CURRENT_LIMIT;
    } else if (iq_ref < -CURRENT_LIMIT) {
        iq_ref = -CURRENT_LIMIT;
    }
    
    return iq_ref;
}

/* ==================== IF模式控制 (电流-频率开环) ==================== */

/**
 * @brief IF模式控制 (电流-频率开环)
 * @details 通过控制电流幅值和频率来驱动电机，仅有电流反馈闭环，位置开环
 */
void IF_Mode_Control(FOC_Controller_t* controller, float current_amplitude, float frequency, float dt) {
    if (!controller || !controller->enabled) return;
    
    // 更新开环角度和速度（基于频率的开环估计）
    float electrical_velocity = _2PI * frequency * controller->pole_pairs;
    controller->openloop_velocity = electrical_velocity / controller->pole_pairs; // 机械角速度
    controller->openloop_angle += electrical_velocity * dt;
    controller->openloop_angle = Normalize_Angle(controller->openloop_angle);
    
    // 在IF模式下，使用开环角度进行坐标变换（位置开环）
    // 电气角度由频率决定，而非来自位置传感器
    controller->electrical_angle = controller->openloop_angle;
    controller->shaft_velocity = controller->openloop_velocity;

    // 读取三相电流
    Read_Phase_Currents(controller);
    
    // Clarke变换 (abc -> αβ)
    controller->alpha_beta_current = Clarke_Transform(controller->phase_currents);
    
    // Park变换 (αβ -> dq)，使用开环电角度
    controller->dq_current = Park_Transform(controller->alpha_beta_current, controller->electrical_angle);
    
    // 低通滤波
    controller->dq_current.d = LPF_Compute(&controller->lpf_current_d, controller->dq_current.d, dt);
    controller->dq_current.q = LPF_Compute(&controller->lpf_current_q, controller->dq_current.q, dt);
    
    // 计算电流误差（IF模式下，目标d轴电流为0，q轴电流为目标幅值）
    float error_q = current_amplitude - controller->dq_current.q;
    float error_d = 0.0f - controller->dq_current.d;  // 目标d轴电流通常为0 (id=0策略)
    
    // PI控制器计算dq轴电压
    controller->dq_voltage.q = PI_Compute(&controller->pi_current_q, error_q, dt);
    controller->dq_voltage.d = PI_Compute(&controller->pi_current_d, error_d, dt);
    
    // 限制dq轴电压幅值
    float voltage_mag = Fast_Sqrt(controller->dq_voltage.d * controller->dq_voltage.d + 
                              controller->dq_voltage.q * controller->dq_voltage.q);
    if (voltage_mag > VOLTAGE_LIMIT) {
        controller->dq_voltage.d *= (VOLTAGE_LIMIT / voltage_mag);
        controller->dq_voltage.q *= (VOLTAGE_LIMIT / voltage_mag);
    }
    
    // 逆Park变换 (dq -> αβ)，使用开环电角度
    AlphaBeta_t voltage_ab = Inverse_Park_Transform(controller->dq_voltage, controller->electrical_angle);
    
    // SVPWM调制 (带零序注入)
    SVPWM_Modulation(controller, voltage_ab.alpha, voltage_ab.beta);
}

/* ==================== VF模式控制 (电压-频率开环) ==================== */

/**
 * @brief VF模式控制 (电压-频率开环)
 * @details 通过控制电压幅值和频率比值来驱动电机，保持磁通恒定
 */
void VF_Mode_Control(FOC_Controller_t* controller, float voltage_amplitude, float frequency, float dt) {
    if (!controller || !controller->enabled) return;
    
    // 更新开环角度和速度
    float electrical_velocity = _2PI * frequency * controller->pole_pairs;
    controller->openloop_velocity = electrical_velocity / controller->pole_pairs;
    controller->openloop_angle += electrical_velocity * dt;
    controller->openloop_angle = Normalize_Angle(controller->openloop_angle);
    
    // 更新电角度
    controller->electrical_angle = controller->openloop_angle;
    controller->shaft_velocity = controller->openloop_velocity;

    // VF模式：基于电压幅值和频率生成dq轴电压参考
    // 对于VF模式，直接生成电压指令
    // 注意：参考方案中使用 sin/cos(openloop_angle) 分配 dq 电压，这实际上是在静止坐标系下旋转电压矢量
    // 但随后又进行了 Inverse Park Transform。如果 dq 电压已经是旋转后的值，再逆变换会导致双重旋转。
    // 标准 VF 开环通常直接设定 Vq = V_ref, Vd = 0 在旋转坐标系下，或者直接在静止坐标系下生成正弦波。
    // 参考方案代码:
    // controller->dq_voltage.q = voltage_amplitude * sinf(controller->openloop_angle);
    // controller->dq_voltage.d = voltage_amplitude * cosf(controller->openloop_angle);
    // 这里的 openloop_angle 是电气角度。如果这是 Park 变换的逆过程输入，那么 dq 应该是常量幅值。
    // 如果参考方案意在直接在 alpha-beta 或 abc 层面控制，通常不经过 Park 逆变换。
    // 但为了保持与现有 SVPWM 接口一致，我们假设需要在旋转坐标系下产生恒定电压矢量，即 Vd=0, Vq=Vamp。
    // 然而，参考方案明确写了 sin/cos。这通常用于当 angle 是静止角度时。
    // 让我们仔细看参考方案：它没有调用 Inverse Park，而是直接限制了 dq_voltage。
    // 等等，参考方案片段里并没有显示 VF 模式最后的 SVPWM 调用部分，只显示了电压计算。
    // 原代码中有 Inverse Park 和 SVPWM。
    // 如果我们要严格遵循参考方案的 "dq_voltage.q = ... sin ..." 逻辑，这暗示 dq 帧下的电压是随角度变化的？这在物理上是不对的，除非 frame 定义不同。
    // 通常 VF 开环：在旋转 frame (dq) 中，Vd=0, Vq=Constant。然后 Inverse Park(angle) -> AlphaBeta -> SVPWM。
    // 参考方案可能混淆了坐标系，或者它的 openloop_angle 用法不同。
    // 鉴于这是一个 FOC 文件，标准做法是：
    
    // 1. 计算目标电压幅值 (含 V/F 比和 Boost)
    float base_frequency = 50.0f; 
    float base_voltage = VOLTAGE_LIMIT * 0.8f;
    float vf_ratio = base_voltage / base_frequency;
    float min_voltage = controller->phase_resistance * CURRENT_LIMIT * 0.2f;
    float target_voltage = Fast_Fmax(min_voltage, vf_ratio * frequency);
    target_voltage = Fast_Fmin(target_voltage, voltage_amplitude);

    // 2. 在旋转坐标系下设置电压 (Id=0, Iq->Vq)
    controller->dq_voltage.d = 0.0f;
    controller->dq_voltage.q = target_voltage;
    
    // 3. 限制电压幅值
    float voltage_mag = Fast_Sqrt(controller->dq_voltage.d * controller->dq_voltage.d + 
                                  controller->dq_voltage.q * controller->dq_voltage.q);
    if (voltage_mag > VOLTAGE_LIMIT) {
        controller->dq_voltage.d *= (VOLTAGE_LIMIT / voltage_mag);
        controller->dq_voltage.q *= (VOLTAGE_LIMIT / voltage_mag);
    }

    // 4. 逆Park变换
    AlphaBeta_t voltage_ab = Inverse_Park_Transform(controller->dq_voltage, controller->electrical_angle);
    
    // 5. SVPWM调制
    SVPWM_Modulation(controller, voltage_ab.alpha, voltage_ab.beta);
}

/* ==================== FOC主循环 ==================== */

/**
 * @brief FOC主循环
 * @details 根据当前控制模式执行相应的控制算法
 */
void FOC_Loop(FOC_Controller_t* controller, float dt) {
    if (!controller->enabled || dt <= 0.0f) {
        return;
    }
    
    switch (controller->control_mode) {
        case FOC_MODE_CURRENT:
            // 电流环控制
            Current_Loop_FOC(controller, controller->target_current_q, dt);
            break;
            
        case FOC_MODE_VELOCITY:
            // 速度环控制 (外环) + 电流环控制 (内环)
            {
                float iq_ref = Velocity_Loop_Control(controller, controller->target_velocity, dt);
                Current_Loop_FOC(controller, iq_ref, dt);
            }
            break;
            
        case FOC_MODE_IF:
            // IF模式控制
            IF_Mode_Control(controller, controller->if_current_amplitude, 
                          controller->target_frequency, dt);
            break;
            
        case FOC_MODE_VF:
            // VF模式控制
            VF_Mode_Control(controller, controller->vf_voltage_amplitude, 
                          controller->target_frequency, dt);
            break;
            
        default:
            break;
    }
}

/* ==================== 控制器初始化和配置 ==================== */

/**
 * @brief 初始化FOC控制器
 */
int FOC_Init(FOC_Controller_t* controller, uint8_t motor_id) {
    if (motor_id != MOTOR_ID) {
        return -1;  // 仅支持motor_id=0
    }
    
    // 清零结构体
    memset(controller, 0, sizeof(FOC_Controller_t));
    
    // 设置电机参数
    controller->motor_id = motor_id;
    controller->pole_pairs = POLE_PAIRS;
    controller->phase_resistance = PHASE_RESISTANCE;
    controller->phase_inductance = PHASE_INDUCTANCE;
    controller->kv_rating = KV_RATING;
    
    // 初始化状态变量
    controller->shaft_angle = 0.0f;
    controller->shaft_velocity = 0.0f;
    controller->electrical_angle = 0.0f;
    controller->openloop_angle = 0.0f;
    controller->openloop_velocity = 0.0f;
    
    // 初始化目标变量
    controller->target_current_q = 0.0f;
    controller->target_current_d = 0.0f;
    controller->target_velocity = 0.0f;
    controller->target_voltage = 0.0f;
    controller->target_frequency = 0.0f;
    controller->if_current_amplitude = 1.0f;
    controller->vf_voltage_amplitude = 6.0f;
    
    // 初始化PI控制器 (使用典型的初始参数)
    PI_Init(&controller->pi_current_q, 3.0f, 300.0f, VOLTAGE_LIMIT);  // q轴电流PI
    PI_Init(&controller->pi_current_d, 3.0f, 300.0f, VOLTAGE_LIMIT);  // d轴电流PI
    PI_Init(&controller->pi_velocity, 0.5f, 10.0f, CURRENT_LIMIT);     // 速度PI
    
    // 初始化低通滤波器
    LPF_Init(&controller->lpf_current_q, 0.001f);  // 1ms时间常数
    LPF_Init(&controller->lpf_current_d, 0.001f);
    LPF_Init(&controller->lpf_velocity, 0.005f);   // 5ms时间常数
    
    // 初始化PWM输出
    controller->pwm_a = 0.5f;
    controller->pwm_b = 0.5f;
    controller->pwm_c = 0.5f;
    
    // 设置默认控制模式
    controller->control_mode = FOC_MODE_CURRENT;
    
    // 禁用控制器
    controller->enabled = false;
    
    // 存储控制器实例
    foc_controllers[motor_id] = *controller;
    
    return 0;
}

/**
 * @brief 配置PI控制器参数
 * @param pi PI控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param limit 输出限制
 */
void PI_Init(PI_Controller_t* pi, float kp, float ki, float limit) {
    pi->kp = kp;
    pi->ki = ki;
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->output_limit = limit;
    pi->ramp_rate = 0.0f;  // 默认无斜坡限制
    pi->prev_output = 0.0f;
}

/**
 * @brief PI控制器计算
 * @param pi PI控制器指针
 * @param error 误差值
 * @param dt 时间间隔 [s]
 * @return PI输出值
 */
float PI_Compute(PI_Controller_t* pi, float error, float dt) {
    if (dt <= 0.0f || pi == NULL) return 0.0f;
    
    // 比例项
    float p_term = pi->kp * error;
    
    // 积分项
    pi->integral += error * dt * pi->ki;
    
    // 积分限幅，防止积分饱和
    if (pi->integral > pi->output_limit) {
        pi->integral = pi->output_limit;
    } else if (pi->integral < -pi->output_limit) {
        pi->integral = -pi->output_limit;
    }
    
    // 计算输出
    float output = p_term + pi->integral;
    
    // 输出限幅
    if (output > pi->output_limit) {
        output = pi->output_limit;
    } else if (output < -pi->output_limit) {
        output = -pi->output_limit;
    }
    
    // 斜坡限制
    if (pi->ramp_rate > 0.0f) {
        float max_delta = pi->ramp_rate * dt;
        float delta = output - pi->prev_output;
        
        if (delta > max_delta) {
            output = pi->prev_output + max_delta;
        } else if (delta < -max_delta) {
            output = pi->prev_output - max_delta;
        }
        
        pi->prev_output = output;
    }
    
    return output;
}

/**
 * @brief 重置PI控制器
 * @param pi PI控制器指针
 */
void PI_Reset(PI_Controller_t* pi) {
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->prev_output = 0.0f;
}

/**
 * @brief 设置控制模式
 */
void FOC_SetControlMode(FOC_Controller_t* controller, FOC_ControlMode_t mode) {
    // 切换模式时重置PI
    if (mode != controller->control_mode) {
        PI_Reset(&controller->pi_current_q);
        PI_Reset(&controller->pi_current_d);
        PI_Reset(&controller->pi_velocity);
        controller->openloop_angle = 0.0f;
    }
    
    controller->control_mode = mode;
}

/**
 * @brief 使能/禁用FOC控制器
 */
void FOC_Enable(FOC_Controller_t* controller, bool enable) {
    if (!controller) return;
    
    controller->enabled = enable;
    
    if (!enable) {
        // 禁用时重置所有控制器
        PI_Reset(&controller->pi_current_q);
        PI_Reset(&controller->pi_current_d);
        PI_Reset(&controller->pi_velocity);
        
        // 重置滤波器状态 (可选，防止突变)
        // controller->lpf_current_q.prev_output = 0.0f; ... 
        
        // 将PWM输出设为中点
        controller->pwm_a = 0.5f;
        controller->pwm_b = 0.5f;
        controller->pwm_c = 0.5f;
    } else {
        // 使能时重置角度和状态
        controller->openloop_angle = 0.0f;
        controller->electrical_angle = 0.0f;
        PI_Reset(&controller->pi_current_q);
        PI_Reset(&controller->pi_current_d);
        PI_Reset(&controller->pi_velocity);
    }
}

/* ==================== 获取控制器实例 ==================== */

/**
 * @brief 获取指定电机ID的控制器实例
 * @return 控制器指针，如果ID无效则返回NULL
 */
FOC_Controller_t* FOC_GetController(uint8_t motor_id) {
    if (motor_id == MOTOR_ID) {
        return &foc_controllers[motor_id];
    }
    return NULL;
}

/**
 * @brief ADC中断服务程序
 * @details 在ADC转换完成后触发，执行电机控制算法
 */
void ADC_IRQHandler(void) {
    // 检查是否是ADC中断
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) && __HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_EOC)) {
        // 清除ADC中断标志
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
        
        // 静态变量用于计算时间差
        static uint32_t last_tick = 0;
        uint32_t current_tick = HAL_GetTick();
        
        // 计算时间差（单位：秒）
        float dt;
        if(last_tick != 0) {
            dt = (float)(current_tick - last_tick) / 1000.0f;  // 转换为秒
        } else {
            dt = 0.001f;  // 初始时间差设为1ms
        }
        last_tick = current_tick;
        
        // 获取控制器实例
        FOC_Controller_t* controller = FOC_GetController(MOTOR_ID);
        if(controller != NULL) {
            // 执行FOC控制算法
            FOC_Loop(controller, dt);
        }
    }
}

/**
 * @brief 初始化ADC和定时器8，配置为由定时器8更新事件触发ADC采样
 */
void FOC_PWM_ADC_Timer_Init(void) {
    // 这里是示意代码，实际需要根据STM32 HAL库函数进行配置
    
    /*
    // 1. 配置ADC1, ADC2, ADC3
    ADC_HandleTypeDef hadc1, hadc2, hadc3;
    
    // ADC1配置
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;  // 由TIM8 TRGO触发
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    
    // ADC2配置 (类似)
    hadc2.Instance = ADC2;
    // ... 类似的初始化代码 ...
    
    // ADC3配置 (类似)
    hadc3.Instance = ADC3;
    // ... 类似的初始化代码 ...
    
    // 2. 配置定时器8以产生PWM并触发ADC
    TIM_HandleTypeDef htim8;
    
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;  // 中心对齐模式，可实现互补PWM
    htim8.Init.Period = 2100 - 1;  // 假设系统时钟为168MHz，预分频器为0，则PWM频率约为40kHz
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    
    // 配置PWM通道
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1050;  // 50%占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    // 配置三个通道用于三相PWM
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    
    // 配置定时器8的TRGO输出，用于触发ADC
    TIM_MasterConfigTypeDef sMasterConfig;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  // 更新事件触发ADC
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    
    // 3. 配置NVIC
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
    
    // 4. 启动ADC
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    
    // 5. 启动PWM输出
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    */
}

/**
 * @brief 定时器中断服务程序
 * @details 在中断中调用FOC控制算法，实现基于中断的电机控制
 */
void TIMx_CC_IRQHandler(void) {  // 这里TIMx需要根据实际使用的定时器修改
    // 清除中断标志位（根据实际定时器型号修改）
    // __HAL_TIM_CLEAR_IT(&htimx, TIM_IT_CCx);
    
    // 静态变量用于计算时间差
    static uint32_t last_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    
    // 计算时间差（单位：秒）
    float dt;
    if(last_tick != 0) {
        dt = (float)(current_tick - last_tick) / 1000.0f;  // 转换为秒
    } else {
        dt = 0.001f;  // 初始时间差设为1ms
    }
    last_tick = current_tick;
    
    // 获取控制器实例
    FOC_Controller_t* controller = FOC_GetController(MOTOR_ID);
    if(controller != NULL) {
        // 执行FOC控制算法
        FOC_Loop(controller, dt);
    }
}

/**
 * @brief 初始化控制定时器
 * @details 配置定时器以指定频率触发中断，执行电机控制算法
 */
void FOC_Timer_Init(uint32_t pwm_frequency_hz) {
    // 计算ARR值（自动重装载值）
    // 假设定时器时钟频率为APB总线频率，可以根据实际情况调整
    uint32_t timer_clock = 84000000; // 例如：STM32F4 APB1 = 84MHz, APB2 = 168MHz
    
    // 根据PWM频率计算定时器重装载值
    uint32_t arr_value = timer_clock / pwm_frequency_hz;
    
    // 这里是示意代码，实际需要根据使用的定时器进行配置
    /*
    htimx.Instance = TIMx; // 根据实际使用的定时器修改
    htimx.Init.Prescaler = 0;
    htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimx.Init.Period = arr_value - 1;
    htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&htimx) != HAL_OK) {
        // 初始化错误处理
        Error_Handler();
    }
    
    // 配置定时器更新中断
    HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
    
    // 启动定时器并开启中断
    HAL_TIM_Base_Start_IT(&htimx);
    */
}
