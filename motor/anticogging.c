//
// Created by kcd on 2024/1/9.
//

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "anticogging.h"
#include "commands.h"
#include "terminal.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "encoder.h"
#include "errno.h"
#include "mcpwm_foc.h"
//#include "../../anticogging/anticogging_data.h"
#include "flash_helper.h"
#include <crc.h>
#include <buffer.h>

//extern float anticogging_data[3600];
bool anticogging_calibration_done;
#define ADDR_FLASH_SECTOR_8     				((uint32_t)0x08080000)
#define ADDR_FLASH_SECTOR_9 				    ((uint32_t)0x080A0000)
#define BLOCK_FLAG_WRITTEN                      ((uint32_t)0x8E2E5BB0)
#define BLOCK_FLAG_UNWRITTEN                    ((uint32_t)0xFFFFFFFF)
#define BLOCK_FLAG_INVALID                      ((uint32_t)0x00000000)

typedef struct {
    float iq_common_mode[3600];
    float iq_differential_mode[3600];
    uint32_t flag;
    uint32_t crc;
} anticogging_data_block;

anticogging_data_block* current_block = NULL;
bool current_block_valid = false; // false denotes that current block is not valid but writeable (all 0xff)

void anticogging_block_get(void);

bool check_user_abort(void) {
    if(mc_interface_get_state() == MC_STATE_OFF) {
        // when stop button is clicked in VESC Tool
        commands_printf("warning: abort by user\n");
        return true;
    }
    return false;
}

float strtof_default(const char* str, float def) {
    float ret;
    char* str_end;
    ret = strtof(str, &str_end);
    if(errno == ERANGE) {
        return def;
    }
    return ret;
}

float endian_swap(float input) {
#define BIG_LITTLE_SWAP32(x)        ( (((*(uint32_t *)&x) & 0xff000000) >> 24) | \
                                      (((*(uint32_t *)&x) & 0x00ff0000) >> 8) | \
                                      (((*(uint32_t *)&x) & 0x0000ff00) << 8) | \
                                      (((*(uint32_t *)&x) & 0x000000ff) << 24) )
    uint8_t *p = (uint8_t*)&input;
    uint8_t ret_array[4] = {p[3],p[2],p[1],p[0]};
    float retf = *(float*)ret_array;
    return retf;
}

float angle_diff(float x, float y) {
    float diff = x - y;
    if(diff > 180.f) {
        // cross zero clockwise
        // e.g. last = 1 deg, input = 359 deg
        diff -= 360.f;
    }
    else if(diff < -180.f) {
        // cross zero counterclockwise
        // e.g. last = 359 deg, input = 1 deg
        diff += 360.f;
    }
    return diff;
}

void util_angle_multi_turn_handler(float abs_input, float *last_input, float *out) {
    float diff = abs_input - *last_input;
    if(diff > 180.f) {
        // cross zero clockwise
        // e.g. last = 1 deg, input = 359 deg
        diff -= 360.f;
    }
    else if(diff < -180.f) {
        // cross zero counterclockwise
        // e.g. last = 359 deg, input = 1 deg
        diff += 360.f;
    }
    *out += diff;
    *last_input = abs_input;
}

float util_read_angle_filter(int delay_ms) {
    float pos_sum = 0.f;
    int delay_500us = delay_ms * 2;
    for(int i = 0; i < delay_500us; ++i) {
        pos_sum += encoder_read_deg();
        chThdSleepMicroseconds(500);
    }
    return pos_sum / delay_500us;
}

void terminal_anticogging_calibration(int argc, const char **argv) {
    if(argc == 1) {
        commands_printf("usage: anticogging_calibration [mode] <param>\n");
    }
    else {
        // argc >= 2
        char* dummy;
        int mode = strtol(argv[1], &dummy, 10);
        if (mode == 0) {
            // release
            mc_interface_release_motor();
        }
        else if(mode == 1 && argc == 3) {
            // position loop
            float pos = strtof(argv[2],&dummy);
            timeout_reset();
            mc_interface_set_pid_pos(pos);
        }
        else if(mode == 2 && argc == 2) {
            // print current position
            encoder_type_t enc = encoder_is_configured();
            if(enc == ENCODER_TYPE_NONE) {
                commands_printf("error: encoder not configured\n");
                return;
            }
            float pos = util_read_angle_filter(20);
            commands_printf("current pos: %f\n", (double)pos);
        }
        else if(mode == 3 && argc == 2) {
            // do calibration - position loop mode
            anticogging_calibration(true, 100, 10, 0.5f, 0.2f, NULL);
        }
        else if(mode == 4 && argc == 2) {
            // do calibration : current mode
            encoder_type_t enc = encoder_is_configured();
            if(enc == ENCODER_TYPE_NONE) {
                commands_printf("error: encoder not configured\n");
                return;
            }
            // configurations
            const float slots = 36.f;
            const float current_delta = 0.05f;
            const float pos_step = 360.f / slots * 0.7f;
            float start_pos = util_read_angle_filter(20);
            float current_pos = start_pos;
            float last_pos = start_pos;
            commands_printf("pos, iq");
            while(current_pos < start_pos + 360.f) {
                // measure one revolution
                float current_out = 0.f; // current amps
                float current_max = 0.f;
                float end_pos = current_pos + pos_step;
                float mid_pos = current_pos + pos_step / 4.f; // when to slow down sample rate
                float half_pos = current_pos + pos_step * 3.f / 5.f; // restore using pos loop
                while(true) {
                    // measure one slot_step
                    // next current step
                    current_out += current_delta;
                    timeout_reset();
                    mc_interface_set_current(current_out);
                    if(current_pos < mid_pos) {
                        // delay 200 ms and avoid continuous turning
                        for (int i = 0; i < 200; ++i) {
                            util_angle_multi_turn_handler(encoder_read_deg(), &last_pos, &current_pos);
                            if (current_pos >= end_pos) {
                                break;
                            }
                            if (check_user_abort()) {
                                return;
                            }
                            chThdSleepMilliseconds(1);
                        }
                    }
                    else {
                        // delay 400 ms and avoid continuous turning
                        for (int i = 0; i < 400; ++i) {
                            util_angle_multi_turn_handler(encoder_read_deg(), &last_pos, &current_pos);
                            if (current_pos >= end_pos) {
                                break;
                            }
                            if (check_user_abort()) {
                                return;
                            }
                            chThdSleepMilliseconds(1);
                        }
                    }
                    if (current_pos >= end_pos) {
                        // commands_printf("# end smpl");
                        timeout_reset();
                        mc_interface_set_pid_pos(end_pos);
                        chThdSleepMilliseconds(300);
                        // back
                        timeout_reset();
                        mc_interface_set_pid_pos(half_pos);
                        chThdSleepMilliseconds(300);
                        mc_interface_release_motor();
                        break;
                    }
                    else {
                        util_angle_multi_turn_handler(util_read_angle_filter(20), &last_pos, &current_pos);
                        float angle_out = current_pos;
                        current_max = MAX(current_max, current_out);
                        utils_norm_angle(&angle_out);
                        commands_printf("%f, %f", (double)angle_out, (double)current_out);
                    }
                }
            }
            commands_printf("sample success\n");
        }
        else if(mode == 5 && argc == 4) {
            // read specified address and print
            // anticogging_calibration 5 <addr:hex> <len:dec>
            char* ptr, *end;
            ptr = (char*)strtoul(argv[2], &end, 16) + ADDR_FLASH_SECTOR_8;
            if(end == argv[2]) {
                commands_printf("sscanf failed.");
                return;
            }
            uint32_t len;
            len = strtoul(argv[3], &end, 10);
            if(end == argv[3]) {
                commands_printf("sscanf failed.");
                return;
            }
            commands_printf("reading %u bytes from %p", len, ptr);
            for(uint32_t i = 0; i < len; i += 16) {
                char buf[4 * 16 + 1];
                char* buf_ptr = buf;
                for(uint32_t j = 0; j < 16 && (i + j) < len; ++j) {
                    sprintf(buf_ptr, "%02x ", ptr[i + j]);
                    buf_ptr += strlen(buf_ptr);
                }
                buf[4 * 16] = '\0';
                commands_printf("%p: %s", ptr + i, buf);
            }
            commands_printf("complete\n");
        }
        else if(mode == 6 && argc == 4) {
            // write an uint32_t to specified address
            // anticogging_calibration 6 <addr:hex> <data:hex>
            char* end;
            uint32_t address;
            address = strtoul(argv[2], &end, 16);
            if(end == argv[2]) {
                commands_printf("sscanf failed.");
                return;
            }
            uint32_t data_to_write;
            data_to_write = strtoul(argv[3], &end, 16);
            if(end == argv[3]) {
                commands_printf("sscanf failed.");
                return;
            }
            uint32_t start = chVTGetSystemTime();
            if(flash_helper_write_nvm((uint8_t*)&data_to_write, 4, address)) {
                uint32_t complete = chVTGetSystemTime();

                commands_printf("complete after %u00 microseconds", complete - start);
            }
            else {
                uint32_t complete = chVTGetSystemTime();

                commands_printf("failed after %u00 microseconds", complete - start);
            }

        }
        else if(mode == 7) {
            anticogging_block_get();
        }
        else if(mode == 8 && argc == 3) {
            // read feedforward
            char* end;
            uint32_t index;
            index = strtoul(argv[2], &end, 0);
            if(end == argv[2]) {
                commands_printf("strtoul failed");
                return;
            }
            if(!current_block_valid) {
                commands_printf("current block not valid");
                return;
            }
            commands_printf(
                    "read: [%u] -> common mode: %f, diff mode: %f",
                    index, (double)current_block->iq_common_mode[index], (double)current_block->iq_differential_mode[index]
                    );

        }
        else {
            commands_printf("invalid arguments");
            commands_printf("usage: anticogging_calibration [mode] <param>\n");
        }
    }
}

bool anticogging_calibration(
        bool print, int attempt_count, int sample_per_point, float err_abs_threshold, float err_threshold,
        void(* sample_callback)(bool finish, bool success, bool forward, int pos_index, float iq)) {
    // do calibration - position loop mode
    encoder_type_t enc = encoder_is_configured();
    if(enc == ENCODER_TYPE_NONE) {
        if(print) {
            commands_printf("error: encoder not configured\n");
        }
        goto finish_error;
    }
    float start_pos = encoder_read_deg();
    //float end_pos = start_pos + 360.f;
    float delta = 360.f;
    if(print) {
        commands_printf(
                "starting anticogging calibration with attempt=%d, abs_threshold=%f, err_threshold=%f",
                attempt_count, (double)err_abs_threshold, (double)err_threshold
        );
        commands_printf("forward");
        commands_printf("pos, iq, err_abs, err_avg, attempt");
    }
    for(int sample_index = 0; sample_index < 3700; ++sample_index) {
        float pos = (float)sample_index * delta / 3600.f + start_pos;
        utils_norm_angle(&pos);
        timeout_reset();
        mc_interface_set_pid_pos(pos);
        if(check_user_abort()) {
            goto finish_error;
        }
        float pos_err_abs_sum = 0.f;
        float pos_err_sum = 0.f;
        float iq_sample = 0.f;
        float iq_filter_sample = 0.f;
        int sample_attempt;
        for(sample_attempt = 0; sample_attempt < attempt_count; ++sample_attempt) {
            // attempt 100 times
            pos_err_abs_sum = 0.f;
            pos_err_sum = 0.f;
            float iq_sum = 0.f;
            float iq_filter_sum = 0.f;
            if(check_user_abort()) {
                goto finish_error;
            }
            timeout_reset();
            chThdSleepMilliseconds(30);
            for(int i = 0; i < sample_per_point; ++i) {
                // sample 10 times
                chThdSleepMilliseconds(5);
                if(check_user_abort()) {
                    goto finish_error;
                }
                iq_sum += mcpwm_foc_get_iq();
                iq_filter_sum += mcpwm_foc_get_iq_filter();
                float pos_err = angle_diff(encoder_read_deg(), pos);
                pos_err_abs_sum += fabsf(pos_err);
                pos_err_sum += pos_err;
            }
            if(pos_err_abs_sum / sample_per_point < err_abs_threshold && fabsf(pos_err_sum / sample_per_point) < err_threshold) {
                // average pos err threshold
                iq_sample = iq_sum / sample_per_point;
                iq_filter_sample = iq_filter_sum / sample_per_point;
                break;
            }
        }
        if(sample_attempt >= attempt_count) {
            // failed
            if(sample_callback) {
                sample_callback(false, false, true, (int)roundf((pos * 10.f)), iq_filter_sample);
            }
            if(print) {
                commands_printf(
                        "sample failed at pos=%f. with err_abs=%f, err_avg=%f. make sure your position loop works",
                        (double) pos, (double) (pos_err_abs_sum / sample_per_point), (double) (pos_err_sum / sample_per_point));
            }
        }
        else {
            if(sample_callback) {
                sample_callback(false, true, true, (int)roundf((pos * 10.f)), iq_filter_sample);
            }
            if(print) {
                commands_printf("%f, %f, %f, %f, %f, %d",
                                (double) pos, (double) iq_sample, (double) iq_filter_sample, (double) (pos_err_abs_sum / sample_per_point),
                                (double) (pos_err_sum / sample_per_point), sample_attempt);
            }
        }
    }


    if(print) {
        commands_printf("\nreverse");
        commands_printf("pos, iq, err_abs, err_avg, attempt");
    }
    start_pos = encoder_read_deg();
    for(int sample_index = 0; sample_index < 3700; ++sample_index) {
        float pos = -(float)sample_index * delta / 3600.f + start_pos;
        utils_norm_angle(&pos);
        timeout_reset();
        mc_interface_set_pid_pos(pos);
        if(check_user_abort()) {
            goto finish_error;
        }
        float pos_err_abs_sum = 0.f;
        float pos_err_sum = 0.f;
        float iq_sample = 0.f;
        float iq_filter_sample = 0.f;
        int sample_attempt;
        for(sample_attempt = 0; sample_attempt < attempt_count; ++sample_attempt) {
            // attempt 100 times
            pos_err_abs_sum = 0.f;
            pos_err_sum = 0.f;
            float iq_sum = 0.f;
            float iq_filter_sum = 0.f;
            if(check_user_abort()) {
                goto finish_error;
            }
            timeout_reset();
            chThdSleepMilliseconds(30);
            for(int i = 0; i < sample_per_point; ++i) {
                // sample 10 times
                chThdSleepMilliseconds(5);
                if(check_user_abort()) {
                    goto finish_error;
                }
                iq_sum += mcpwm_foc_get_iq();
                iq_filter_sum += mcpwm_foc_get_iq_filter();
                float pos_err = angle_diff(encoder_read_deg(), pos);
                pos_err_abs_sum += fabsf(pos_err);
                pos_err_sum += pos_err;
            }
            if(pos_err_abs_sum / sample_per_point < err_abs_threshold && fabsf(pos_err_sum / sample_per_point) < err_threshold) {
                // average pos err threshold
                iq_sample = iq_sum / sample_per_point;
                iq_filter_sample = iq_filter_sum / sample_per_point;
                break;
            }
        }
        if(sample_attempt >= attempt_count) {
            // failed
            if(sample_callback) {
                sample_callback(false, false, false, 0, 0.f);
            }
            if(print) {
                commands_printf(
                        "sample failed at pos=%f. with err_abs=%f, err_avg=%f. make sure your position loop works",
                        (double) pos, (double) (pos_err_abs_sum / sample_per_point), (double) (pos_err_sum / sample_per_point));
            }
        }
        else {
            if(sample_callback) {
                sample_callback(false, true, false, (int)roundf((pos * 10.f)), iq_filter_sample);
            }
            if(print) {
                commands_printf("%f, %f, %f, %f, %f, %d",
                                (double) pos, (double) iq_sample, (double) iq_filter_sample, (double) (pos_err_abs_sum / sample_per_point),
                                (double) (pos_err_sum / sample_per_point), sample_attempt);
            }
        }
    }

    if(print) {
        commands_printf("sample success\n");
    }
    if(sample_callback) {
        sample_callback(true, true, false, 0, 0.f);
    }
    return true;
    finish_error:
    mc_interface_release_motor();
    if(sample_callback) {
        sample_callback(true, false, false, 0, 0.f);
    }
    if(print) {
        commands_printf("sample failed\n");
    }
    return false;
}

void anticogging_init(void) {
    terminal_register_command_callback(
            "anticogging_calibration",
            "do anticogging calibration",
            "[mode] <param>",
            terminal_anticogging_calibration);

}

float anticogging_get_feedforward(void) {
    if(!current_block_valid) {
        commands_printf("get feedforward: current block invalid.");
        return 0.f;
    }
    float pos = encoder_read_deg();
    int index = (int)(pos * 10.f);
    return current_block->iq_common_mode[index];
}

float anticogging_get_feedforward_diff_mode(void) {
    if(!current_block_valid) {
        commands_printf("get feedforward: current block invalid.");
        return 0.f;
    }
    float pos = encoder_read_deg();
    int index = (int)(pos * 10.f);
    return current_block->iq_differential_mode[index];
}

bool anticogging_block_verify(anticogging_data_block* data) {
    // a valid block should have:
    // 1. flag == BLOCK_FLAG_WRITTEN
    // 2. crc == 0
    if(data->flag != BLOCK_FLAG_WRITTEN) {
        return false;
    }

    // make sure RCC_AHB1Periph_CRC is enabled
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
    crc32_reset();
    uint32_t crc = crc32((uint32_t*)data, sizeof(anticogging_data_block) / 4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, DISABLE);
    return crc == 0;
}

bool anticogging_block_unwritten(anticogging_data_block* data) {
    // an unwritten block should all be made up of 0xff bytes
    uint32_t* ptr = (uint32_t*)data;
    uint32_t* end = (uint32_t*)(data + 1);
    while(ptr < end) {
        if(*ptr != BLOCK_FLAG_UNWRITTEN) {
            return false;
        }
        ++ptr;
    }
    return true;
}

void anticogging_block_get(void) {
    // look for a valid block
    anticogging_data_block* ptr = (anticogging_data_block*)ADDR_FLASH_SECTOR_8;
    while(ptr < (anticogging_data_block*)ADDR_FLASH_SECTOR_9) {
        if(anticogging_block_verify(ptr)) {
            // found
            current_block = ptr;
            current_block_valid = true;
            commands_printf("anticogging: block_get: found valid block at %p", ptr);
            return;
        }
        ++ptr;
    }
    // not found: search from start for next unwritten block
    ptr = (anticogging_data_block*)ADDR_FLASH_SECTOR_8;
    while(ptr < (anticogging_data_block*)ADDR_FLASH_SECTOR_9) {
        if(anticogging_block_unwritten(ptr)) {
            // found
            current_block = ptr;
            current_block_valid = false;
            commands_printf("anticogging: block_get: found unwritten block at %p", ptr);
            return;
        }
        ++ptr;
    }
    // not found: erase current sector (8) and return first block
    bool ret = flash_helper_wipe_nvm();
    if(!ret) {
        commands_printf("anticogging: block_get: flash erasure failed");
        return;
    }
    current_block = (anticogging_data_block*)ADDR_FLASH_SECTOR_8;
    current_block_valid = false;
    commands_printf("anticogging: block_get: sector erased. unwritten block at %p", current_block);
    if(!anticogging_block_unwritten(current_block)) {
        // should be impossible
        commands_printf("anticogging: block_get: sector erased. however block is still not unwritten");
    }
}

bool anticogging_block_write_start(void) {
    // prepare for flashing a new block
    if(current_block_valid) {
        // invalidate current block
        commands_printf("anticogging: write_start: invalidate current block");
        uint32_t invalid_flag = BLOCK_FLAG_INVALID;
        bool ret = flash_helper_write_nvm(
                (uint8_t*)&invalid_flag, 4,
                (uint32_t)((uint8_t*)current_block - ADDR_FLASH_SECTOR_8 + offsetof(anticogging_data_block, crc))
                );
        if(!ret) {
            commands_printf("anticogging: write_start: failed to invalidate current block");
        }
    }
    // search from start for next unwritten block
    anticogging_data_block* ptr;
    ptr = (anticogging_data_block*)ADDR_FLASH_SECTOR_8;
    while(ptr < (anticogging_data_block*)ADDR_FLASH_SECTOR_9) {
        if(anticogging_block_unwritten(ptr)) {
            // found
            current_block = ptr;
            current_block_valid = false;
            commands_printf("anticogging: write_start: found unwritten block at %p", ptr);
            return true;
        }
        ++ptr;
    }
    // not found: erase current sector (8) and return first block
    bool ret = flash_helper_wipe_nvm();
    if(!ret) {
        commands_printf("anticogging: write_start: flash erasure failed");
        return false;
    }
    current_block = (anticogging_data_block*)ADDR_FLASH_SECTOR_8;
    current_block_valid = false;
    commands_printf("anticogging: write_start: sector erased. unwritten block at %p", current_block);
    if(!anticogging_block_unwritten(current_block)) {
        // should be impossible
        commands_printf("anticogging: write_start: sector erased. however block is still not unwritten");
        return false;
    }
    return true;
}

bool anticogging_block_write(uint8_t* ptr, uint32_t len, uint32_t offset) {
    if(offset + len > offsetof(anticogging_data_block, flag)) {
        commands_printf("anticogging: block_write: offset + len = %u + %u out of range.", offset, len);
        return false;
    }
    // convert
    float * pval = (float*)ptr;
    while(pval < (float*)(ptr + len)) {
        *pval = endian_swap(*pval);
        ++pval;
    }
    // write
    bool ret = flash_helper_write_nvm(
            ptr, len,
            (uint32_t)((uint8_t*)current_block - ADDR_FLASH_SECTOR_8 + offset)
            );
    if(!ret) {
        commands_printf("anticogging: block_write: failed to write flash");
    }
    commands_printf("anticogging: block_write: write offset = %u, len = %u", offset, len);
    return true;
}

bool anticogging_block_write_end(void) {
    // finishing block write
    // set flag to BLOCK_FLAG_WRITTEN
    uint32_t written_flag = BLOCK_FLAG_WRITTEN;
    bool ret = flash_helper_write_nvm(
            (uint8_t*)&written_flag, 4,
            (uint32_t)((uint8_t*)current_block - ADDR_FLASH_SECTOR_8 + offsetof(anticogging_data_block, flag))
    );
    // set crc
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
    crc32_reset();
    uint32_t crc = crc32((uint32_t*)current_block, offsetof(anticogging_data_block, crc) / 4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, DISABLE);
    ret &= flash_helper_write_nvm(
            (uint8_t*)&crc, 4,
            (uint32_t)((uint8_t*)current_block - ADDR_FLASH_SECTOR_8 + offsetof(anticogging_data_block, crc))
    );
    if(!ret) {
        commands_printf("anticogging: write_end: failed to write flash");
        return false;
    }
    if(!anticogging_block_verify(current_block)) {
        commands_printf("anticogging: write_end: invalid after write");
        anticogging_block_get();
        return false;
    }
    current_block_valid = true;
    commands_printf("anticogging: write_end: complete");
    return true;
}

void anticogging_block_read(uint8_t* ptr, uint32_t len, uint32_t offset) {
    if(offset + len > offsetof(anticogging_data_block, flag)) {
        commands_printf("anticogging: block_read: offset + len = %u + %u out of range.", offset, len);
        return;
    }
    memcpy(ptr, (uint8_t*)current_block + offset, len);
    // convert
    float * pval = (float*)ptr;
    while(pval < (float*)(ptr + len)) {
        *pval = endian_swap(*pval);
        ++pval;
    }
    commands_printf("anticogging: block_read: read offset, len = %u, %u", offset, len);
}

bool anticogging_current_block_valid(void) {
    return current_block_valid;
}