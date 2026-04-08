/**
 * @file STM32F4_FOC_Example.c
 * @brief STM32F446 FOC控制使用示例
 * @details 演示如何使用FOC控制模块进行电流环、速度环、IF模式和VF模式控制
 * 
 * 硬件平台: STM32F446
 * 电机ID: 0
 */

#include "STM32F4_FOC_Control.h"
#include <stdio.h>

// 全局FOC控制器实例
static FOC_Controller_t motor_controller;

/**
 * @brief 系统初始化
 */
void System_Init(void) {
    // 初始化STM32外设 (时钟、GPIO、定时器、ADC等)
    // TODO: 添加实际的STM32 HAL初始化代码
    
    printf("System initialized\n");
}

/**
 * @brief FOC控制器初始化
 */
void FOC_Controller_Init(void) {
    int result;
    
    // 初始化FOC控制器 (motor_id = 0)
    result = FOC_Init(&motor_controller, 0);
    if (result != 0) {
        printf("FOC initialization failed!\n");
        return;
    }
    
    printf("FOC controller initialized for motor ID 0\n");
    
    // 可选: 自定义PID参数
    // PID_Init(&motor_controller.pid_current_q, 5.0f, 500.0f, 0.0f, VOLTAGE_LIMIT);
    // PID_Init(&motor_controller.pid_current_d, 5.0f, 500.0f, 0.0f, VOLTAGE_LIMIT);
    // PID_Init(&motor_controller.pid_velocity, 1.0f, 20.0f, 0.0f, CURRENT_LIMIT);
}

/**
 * @brief 示例1: 电流环控制模式
 * @details 直接控制q轴电流，实现转矩控制
 */
void Example_Current_Mode(void) {
    float dt = 0.0001f;  // 100us控制周期
    
    // 设置为电流环控制模式
    FOC_SetControlMode(&motor_controller, FOC_MODE_CURRENT);
    
    // 使能控制器
    FOC_Enable(&motor_controller, true);
    
    // 设置目标q轴电流 (2A)
    motor_controller.target_current_q = 2.0f;
    
    printf("Current mode: Target Iq = 2.0A\n");
    
    // 主循环 (在实际应用中应该在中断或RTOS任务中运行)
    for (int i = 0; i < 1000; i++) {
        FOC_Loop(&motor_controller, dt);
        // delay_us(100);  // 等待100us
    }
    
    // 禁用控制器
    FOC_Enable(&motor_controller, false);
}

/**
 * @brief 示例2: 速度环控制模式
 * @details 通过速度环自动控制电流，实现速度闭环控制
 */
void Example_Velocity_Mode(void) {
    float dt = 0.0001f;  // 100us控制周期
    
    // 设置为速度环控制模式
    FOC_SetControlMode(&motor_controller, FOC_MODE_VELOCITY);
    
    // 使能控制器
    FOC_Enable(&motor_controller, true);
    
    // 设置目标速度 (100 rad/s ≈ 955 RPM)
    motor_controller.target_velocity = 100.0f;
    
    printf("Velocity mode: Target velocity = 100 rad/s\n");
    
    // 主循环
    for (int i = 0; i < 5000; i++) {
        FOC_Loop(&motor_controller, dt);
        // delay_us(100);
        
        // 打印状态 (每1000次循环)
        if (i % 1000 == 0) {
            printf("Velocity: %.2f rad/s, Current q: %.2f A\n", 
                   motor_controller.shaft_velocity,
                   motor_controller.dq_current.q);
        }
    }
    
    // 禁用控制器
    FOC_Enable(&motor_controller, false);
}

/**
 * @brief 示例3: IF模式控制 (电流-频率开环)
 * @details 无需位置传感器，通过控制电流和频率驱动电机
 */
void Example_IF_Mode(void) {
    float dt = 0.0001f;  // 100us控制周期
    float frequency = 0.0f;
    float ramp_rate = 1.0f;  // 频率斜坡 1Hz/s
    
    // 设置为IF模式
    FOC_SetControlMode(&motor_controller, FOC_MODE_IF);
    
    // 使能控制器
    FOC_Enable(&motor_controller, true);
    
    // 设置IF模式电流幅值
    motor_controller.if_current_amplitude = 2.0f;
    
    printf("IF mode: Starting frequency ramp\n");
    
    // 频率斜坡启动 (从0到30Hz)
    for (int i = 0; i < 30000; i++) {
        frequency += ramp_rate * dt;
        motor_controller.target_frequency = frequency;
        
        FOC_Loop(&motor_controller, dt);
        // delay_us(100);
        
        // 打印状态
        if (i % 5000 == 0) {
            printf("Frequency: %.1f Hz, Angle: %.2f rad\n", 
                   frequency,
                   motor_controller.openloop_angle);
        }
    }
    
    // 禁用控制器
    FOC_Enable(&motor_controller, false);
}

/**
 * @brief 示例4: VF模式控制 (电压-频率开环)
 * @details 经典的V/F控制，保持磁通恒定
 */
void Example_VF_Mode(void) {
    float dt = 0.0001f;  // 100us控制周期
    float frequency = 0.0f;
    float ramp_rate = 2.0f;  // 频率斜坡 2Hz/s
    
    // 设置为VF模式
    FOC_SetControlMode(&motor_controller, FOC_MODE_VF);
    
    // 使能控制器
    FOC_Enable(&motor_controller, true);
    
    // 设置VF模式电压幅值
    motor_controller.vf_voltage_amplitude = 8.0f;
    
    printf("VF mode: Starting frequency ramp\n");
    
    // 频率斜坡启动 (从0到40Hz)
    for (int i = 0; i < 20000; i++) {
        frequency += ramp_rate * dt;
        motor_controller.target_frequency = frequency;
        
        FOC_Loop(&motor_controller, dt);
        // delay_us(100);
        
        // 打印状态
        if (i % 5000 == 0) {
            printf("Frequency: %.1f Hz, Voltage: %.1f V\n", 
                   frequency,
                   motor_controller.dq_voltage.q);
        }
    }
    
    // 禁用控制器
    FOC_Enable(&motor_controller, false);
}

/**
 * @brief 动态切换控制模式示例
 */
void Example_Mode_Switching(void) {
    float dt = 0.0001f;
    
    printf("=== Mode Switching Example ===\n");
    
    // 阶段1: VF模式启动 (开环)
    printf("Phase 1: VF mode startup\n");
    FOC_SetControlMode(&motor_controller, FOC_MODE_VF);
    FOC_Enable(&motor_controller, true);
    motor_controller.target_frequency = 10.0f;
    
    for (int i = 0; i < 2000; i++) {
        FOC_Loop(&motor_controller, dt);
    }
    
    // 阶段2: 切换到IF模式
    printf("Phase 2: Switch to IF mode\n");
    FOC_SetControlMode(&motor_controller, FOC_MODE_IF);
    motor_controller.if_current_amplitude = 1.5f;
    motor_controller.target_frequency = 15.0f;
    
    for (int i = 0; i < 2000; i++) {
        FOC_Loop(&motor_controller, dt);
    }
    
    // 阶段3: 切换到速度环闭环控制
    printf("Phase 3: Switch to velocity closed-loop\n");
    FOC_SetControlMode(&motor_controller, FOC_MODE_VELOCITY);
    motor_controller.target_velocity = 50.0f;
    
    for (int i = 0; i < 5000; i++) {
        FOC_Loop(&motor_controller, dt);
    }
    
    // 阶段4: 切换到电流环直接控制
    printf("Phase 4: Switch to current control\n");
    FOC_SetControlMode(&motor_controller, FOC_MODE_CURRENT);
    motor_controller.target_current_q = 1.0f;
    
    for (int i = 0; i < 2000; i++) {
        FOC_Loop(&motor_controller, dt);
    }
    
    // 禁用
    FOC_Enable(&motor_controller, false);
    printf("=== Example Complete ===\n");
}

/**
 * @brief 获取控制器状态并打印
 */
void Print_Controller_Status(FOC_Controller_t* controller) {
    printf("\n--- Controller Status ---\n");
    printf("Motor ID: %d\n", controller->motor_id);
    printf("Enabled: %s\n", controller->enabled ? "Yes" : "No");
    printf("Control Mode: %d\n", controller->control_mode);
    printf("Electrical Angle: %.3f rad\n", controller->electrical_angle);
    printf("Shaft Velocity: %.2f rad/s\n", controller->shaft_velocity);
    printf("DQ Current - d: %.3f A, q: %.3f A\n", 
           controller->dq_current.d, controller->dq_current.q);
    printf("DQ Voltage - d: %.3f V, q: %.3f V\n", 
           controller->dq_voltage.d, controller->dq_voltage.q);
    printf("PWM - A: %.2f%%, B: %.2f%%, C: %.2f%%\n",
           controller->pwm_a * 100.0f,
           controller->pwm_b * 100.0f,
           controller->pwm_c * 100.0f);
    printf("------------------------\n\n");
}

/**
 * @brief 主函数示例
 */
int main(void) {
    // 系统初始化
    System_Init();
    
    // FOC控制器初始化
    FOC_Controller_Init();
    
    // 运行各种示例
    printf("\n=== Running Examples ===\n\n");
    
    // 示例1: 电流环控制
    Example_Current_Mode();
    
    // 示例2: 速度环控制
    Example_Velocity_Mode();
    
    // 示例3: IF模式
    Example_IF_Mode();
    
    // 示例4: VF模式
    Example_VF_Mode();
    
    // 示例5: 模式切换
    Example_Mode_Switching();
    
    printf("\nAll examples completed!\n");
    
    while (1) {
        // 主循环
    }
}
