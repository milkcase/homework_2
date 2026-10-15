#ifndef CONTROLTASK_H
#define CONTROLTASK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern volatile uint32_t tick, motor_rx_count, can_tx_enqueue_count, can_tx_error_count;
extern volatile uint32_t can_error, control_ready;
extern volatile float target_vel, feedback_vel, current_ref;
extern volatile float target_angle, feedback_angle; // Relative angle, degrees.
void ControlTaskInit(void);
void MainTask(void);
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);
#ifdef __cplusplus
}
#endif
#endif
