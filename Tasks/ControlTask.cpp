/*
 * @Author: milkcase choose68@163.com
 * @Date: 2026-10-15 19:25:33
 * @LastEditors: milkcase choose68@163.com
 * @LastEditTime: 2026-10-15 21:49:37
 * @FilePath: \homework_2\Tasks\ControlTask.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ControlTask.hpp"
#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"
#include "Gm6020.h"
#include "Pid.h"
#include "HW_can.hpp"
#include <cmath>

volatile uint32_t tick = 0, motor_rx_count = 0;
volatile uint32_t can_tx_enqueue_count = 0, can_tx_error_count = 0;
volatile uint32_t can_error = 0, control_ready = 0;
volatile float target_vel = 0.0f, feedback_vel = 0.0f, current_ref = 0.0f;
volatile float target_angle = 0.0f, feedback_angle = 0.0f; // Degrees relative to start.
static volatile float motor_angle_deg = 0.0f;
static volatile uint32_t last_feedback_ms = 0;
static uint8_t can1_tx_data[8] = {}, can1_rx_data[8];
static CAN_RxHeaderTypeDef rx_header1;
static float phase = 0.0f;
static float angle_center_deg = 0.0f;
static bool position_started = false;
constexpr float kRadToDeg = 57.295779513f;
constexpr float kAngleAmplitudeDeg = 180.0f;
constexpr float kAngleOmega = 0.5f; // rad/s of sine phase; period about 12.57 s.
Gm6020 Motor1(1); // CAN1, motor ID 1, current control mode (enable current loop in RoboMaster Assistant).
// Keep the tuned inner speed loop: rpm -> raw current, limit +/-1500.
Pid vel_pid(20.0f, 50.0f, 0.01f, 1500.0f, -1500.0f);
// Outer position loop: degree error -> rpm. Initial Kp, speed limit +/-60 rpm.
Pid angle_pid(5.0f, 1.0f, 0.0f, 200.0f, -200.0f);

void ControlTaskInit(void) {
    CanFilter_Init(&hcan1);
    Can_EnableAutoRecovery(&hcan1);
    if (HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

void MainTask(void) {
    HAL_IWDG_Refresh(&hiwdg);
    tick = tick + 1;
    // Snapshot the CAN feedback together; CAN RX can preempt this timer IRQ.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t rx_count = motor_rx_count;
    const uint32_t feedback_ms = last_feedback_ms;
    const float angle_deg = motor_angle_deg;
    const float speed_rpm = feedback_vel;
    __set_PRIMASK(primask);
    control_ready = (rx_count != 0U &&
                     uint32_t(HAL_GetTick() - feedback_ms) <= 100U);
    if (control_ready) {
        if (!position_started) {
            // Start around the present shaft angle, rather than encoder zero.
            angle_center_deg = angle_deg;
            phase = 0.0f;
            angle_pid.reset();
            vel_pid.reset();
            position_started = true;
        }
        feedback_angle = angle_deg - angle_center_deg;
        target_angle = kAngleAmplitudeDeg * std::sin(phase);
        target_vel = angle_pid.calc(target_angle, feedback_angle);
        target_vel = 200.0f * std::sin(phase);
        current_ref = vel_pid.calc(target_vel, speed_rpm);
        phase += kAngleOmega * 0.001f;
        if (phase >= 6.283185307f) phase -= 6.283185307f;
    } else {
        position_started = false;
        phase = 0.0f;
        target_angle = 0.0f;
        feedback_angle = 0.0f;
        target_vel = 0.0f;
        current_ref = 0.0f;
        vel_pid.reset();
        angle_pid.reset();
    }
    Motor1.setCurrent(static_cast<int16_t>(current_ref));
    Motor1.encode(can1_tx_data);
    if (CAN_Send_Msg(&hcan1, can1_tx_data, Motor1.txId(), 8) == HAL_OK) {
        can_tx_enqueue_count = can_tx_enqueue_count + 1;
    } else {
        can_tx_error_count = can_tx_error_count + 1;
    }
    can_error = HAL_CAN_GetError(&hcan1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) MainTask();
}

void CanFeedbackCallback(uint32_t id, const uint8_t *data) {
    if (id == Motor1.rxId()) {
        Motor1.decode(data);
        motor_angle_deg = Motor1.angle() * kRadToDeg;
        feedback_vel = Motor1.velRpm();
        last_feedback_ms = HAL_GetTick();
        motor_rx_count = motor_rx_count + 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan != &hcan1) return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) != HAL_OK) break;
        if (rx_header1.IDE == CAN_ID_STD && rx_header1.RTR == CAN_RTR_DATA && rx_header1.DLC == 8U) {
            CanFeedbackCallback(rx_header1.StdId, can1_rx_data);
        }
    }
}
