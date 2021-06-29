/*

  fans.c - plugin for for handling fans

  Part of grblHAL

  Copyright (c) 2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if FANS_ENABLE

#include <string.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"

static const char *fans[] = {
    "Fan 0",
    "Fan 1",
    "Fan 2",
    "Fan 3"
};

static uint8_t base_port;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static driver_reset_ptr driver_reset;

static user_mcode_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == Fan_On || mcode == Fan_Off
            ? mcode
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case Fan_On:
        case Fan_Off:
            if(gc_block->words.p) {
                if(isnan(gc_block->values.p))
                    state = Status_BadNumberFormat;
                else if(!isintf(gc_block->values.p) || gc_block->values.p < 0.0f  || gc_block->values.p >= (float)FANS_ENABLE)
                    state = Status_GcodeValueOutOfRange;
                else
                    gc_block->words.p = Off;
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

        case Fan_On:
            hal.port.digital_out(base_port + gc_block->words.p ? (uint8_t)gc_block->values.p : 0, true);
            break;

        case Fan_Off:
            hal.port.digital_out(base_port + gc_block->words.p ? (uint8_t)gc_block->values.p : 0, false);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void driverReset (void)
{
    driver_reset();

    uint32_t idx = FANS_ENABLE;
    do {
        hal.port.digital_out(base_port + --idx, false);
    } while(idx);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    // Add setting(s) for delayed off?

    uint32_t idx = FANS_ENABLE;
    do {
        hal.port.digital_out(base_port + --idx, false);
    } while(idx);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:Fans v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Fans plugin failed to initialize!", Message_Warning);
}

void fans_init (void)
{
    if(hal.port.num_digital_out >= FANS_ENABLE) {

        hal.port.num_digital_out -= FANS_ENABLE;
        base_port = hal.port.num_digital_out;

        if(hal.port.set_pin_description) {
            uint32_t idx = min(FANS_ENABLE, 4);
            do {
                idx--;
                hal.port.set_pin_description(true, true, base_port + idx, fans[idx]);
            } while(idx);
        }

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = userMCodeCheck;
        hal.user_mcode.validate = userMCodeValidate;
        hal.user_mcode.execute = userMCodeExecute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = driverReset;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_program_completed = grbl.on_program_completed;
        grbl.on_program_completed = onProgramCompleted;

    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
