//
// Created by kcd on 2024/1/9.
//

#ifndef BLDC_V6_02_ANTICOGGING_H
#define BLDC_V6_02_ANTICOGGING_H

void anticogging_init(void);
float anticogging_get_feedforward(void);
float anticogging_get_feedforward_diff_mode(void);

// do anticogging calibration, results are printed out or(and) sent to vesc tool.
// should only be called after encoder and position loop configured
//
// parameters:
//     print:             print results or not. default: true
//     attempt_count:     allowed maximum number of failures waiting for position loop to the commanded position. default: 100
//     err_abs_threshold: absolute tolerance of position, in degrees. default: 0.5
//     err_threshold:     allowed maximum mean deviation of position, in degrees. default: 0.2
//     sample_callback:   called on sample success or failures. default: NULL
// returns:
//     true: after two full revolutions are sampled, both clockwise and counterclockwise
//     false: encoder is not configured, or cancelled by the user manually from stop button
bool anticogging_calibration(bool print, int attempt_count, int sample_per_point, float err_abs_threshold, float err_threshold, void(* sample_callback)(bool finish, bool success, bool forward, int pos_index, float iq));
bool anticogging_block_write_start(void);
bool anticogging_block_write(uint8_t* ptr, uint32_t len, uint32_t offset);
bool anticogging_block_write_end(void);
void anticogging_block_read(uint8_t* ptr, uint32_t len, uint32_t offset);
bool anticogging_current_block_valid(void);
enum ANTICOGGING_BLOCK_TRANSMISSION_STATE {
    AC_BLOCK_START,
    AC_BLOCK_ONGOING,
    AC_BLOCK_END,
    AC_BLOCK_ACK
};

#endif //BLDC_V6_02_ANTICOGGING_H
