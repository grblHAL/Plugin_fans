/*

  fans.c - plugin for for handling fans

  Part of grblHAL

  Copyright (c) 2021-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "driver.h"

#if FANS_ENABLE

#if FANS_ENABLE > 4
#undef FANS_ENABLE
#define FANS_ENABLE 4
#warning Max number of allowed fans is 4!
#endif

#include <string.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/spindle_control.h"

typedef struct {
    uint8_t port[4];
    uint8_t spindle_link;
    float fan0_off_delay;
} fan_settings_t;

static const char *fan_names[] = {
    "Fan 0",
    "Fan 1",
    "Fan 2",
    "Fan 3"
};

static uint32_t n_fans = 0, fans_on = 0, fans_linked = 0;
static user_mcode_ptrs_t user_mcode;
static fan_settings_t fan_setting, fans;
static io_port_cfg_t d_out;
static nvs_address_t nvs_address;

static on_spindle_select_ptr on_spindle_select;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static on_program_completed_ptr on_program_completed;
static spindle_set_state_ptr on_spindle_set_state, fan_spindle_set_state = NULL;
static on_unknown_accessory_override_ptr on_unknown_accessory_override;
static driver_reset_ptr driver_reset;

bool fan_get_state (uint8_t fan);
void fan_set_state (uint8_t fan, bool on);

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == Fan_On || mcode == Fan_Off
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case Fan_On:    // M106
        case Fan_Off:   // M107
            if(gc_block->words.p) {
                if(!isintf(gc_block->values.p) || gc_block->values.p < 0.0f  || fans.port[(uint_fast8_t)gc_block->values.p] == 0xFF)
                    state = Status_GcodeValueOutOfRange;
                gc_block->words.p = Off;
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void fan_off (void *data)
{
    fan_set_state(0, Off);
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;
    uint_fast8_t fan = gc_block->words.p ? (uint8_t)gc_block->values.p : 0;

    if (state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

        case Fan_On:
            fan_set_state(fan, On);
            break;

        case Fan_Off:
            if(fan == 0)
                task_delete(fan_off, NULL);
            fan_set_state(fan, Off);
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
        fan_set_state(--idx, Off);
    } while(idx);
}

static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    uint_fast8_t idx = FANS_ENABLE;
    do {
        if(bit_istrue(fans.spindle_link, bit(--idx))) {

            if(!state.on && bit_isfalse(fans_linked, bit(idx)))
                continue;

            if(state.on && !fan_get_state(idx))
                bit_true(fans_linked, bit(idx));

            if(idx == 0 && !state.on && fan_setting.fan0_off_delay > 0.0f)
                task_add_delayed(fan_off, NULL, (uint32_t)(fan_setting.fan0_off_delay * 60.0f * 1000.0f));
            else
                fan_set_state(idx, state.on);
        }
    } while(idx);

    on_spindle_set_state(spindle, state, rpm);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    uint_fast8_t idx = FANS_ENABLE;
    do {
        if(--idx == 0 && fans.port[0] != 0xFF && fan_setting.fan0_off_delay > 0.0f)
            task_add_delayed(fan_off, NULL, (uint32_t)(fan_setting.fan0_off_delay * 60.0f * 1000.0f));
        else
            fan_set_state(idx, Off);
    } while(idx);

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(report.fan) {
        stream_write("|Fan:");
        stream_write(uitoa(fans_on));
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onAccessoryOverride (uint8_t cmd)
{
    if(cmd == CMD_OVERRIDE_FAN0_TOGGLE && fans.port[0] != 0xFF)
        fan_set_state(0, !fan_get_state(0));
    else if(on_unknown_accessory_override)
        on_unknown_accessory_override(cmd);
}

bool fan_get_state (uint8_t fan)
{
    return fans.port[fan] != 0xFF && !!(fans_on & (1 << fan));
}

void fan_set_state (uint8_t fan, bool on)
{
    if(fans.port[fan] != 0xFF) {

        if(on)
            bit_true(fans_on, bit(fan));
        else {
            bit_false(fans_on, bit(fan));
            bit_false(fans_linked, bit(fan));
        }

        if(fan == 0)
            task_delete(fan_off, NULL);

        report_add_realtime(Report_Fan);

        if(fan == 0 && fan_spindle_set_state) {
            spindle_state_t state = {0};
            state.on = on;
            fan_spindle_set_state(NULL, state, 0.0f);
        } else
            ioport_digital_out(fans.port[fan], on);
    }
}

static bool spindle_enumerate (spindle_info_t *spindle, void *data)
{
// TODO: needs evaluation - may be a dangerous approach to using the driver spindle...
//    if(!spindle->is_current && (spindle->hal->type == SpindleType_Basic || spindle->hal->type == SpindleType_PWM))
//        fan_spindle_set_state = spindle->hal->set_state;
    return true;
}

static void fan_setup (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    driver_reset = hal.driver_reset;
    hal.driver_reset = driverReset;

    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = onRealtimeReport;

    on_unknown_accessory_override = grbl.on_unknown_accessory_override;
    grbl.on_unknown_accessory_override = onAccessoryOverride;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    return d_out.n_ports >= setting->id - Setting_FanPort0;
}

static status_code_t set_float (setting_id_t setting, float value)
{
    return d_out.set_value(&d_out, &fan_setting.port[setting - Setting_FanPort0], (pin_cap_t){}, value);
}

static float get_float (setting_id_t setting)
{
    return d_out.get_value(&d_out, fan_setting.port[setting - Setting_FanPort0]);
}

static status_code_t set_spindle_link (setting_id_t setting, uint32_t value)
{
    uint_fast8_t idx = FANS_ENABLE;

    fans.spindle_link = 0;
    fan_setting.spindle_link = value;

    do {
        if(bit_istrue(fan_setting.spindle_link, bit(--idx)) && fans.port[idx] != IOPORT_UNASSIGNED)
            bit_true(fans.spindle_link, bit(idx));
    } while(idx);

    return Status_OK;
}

static uint32_t get_spindle_link (setting_id_t setting)
{
    return fan_setting.spindle_link;
}

static const setting_detail_t fan_settings[] = {
    { Setting_Fan0OffDelay, Group_Coolant, "Fan 0 off delay", "minutes", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &fan_setting.fan0_off_delay, NULL, NULL },
    { Setting_FanPort0, Group_AuxPorts, "Fan 0 port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
#if FANS_ENABLE == 1
    { Setting_FanToSpindleLink, Group_Spindle, "Fan to spindle enable link", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCoreFn, set_spindle_link, get_spindle_link, NULL },
#endif
#if FANS_ENABLE > 1
    { Setting_FanPort1, Group_AuxPorts, "Fan 1 port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
#endif
#if FANS_ENABLE == 2
    { Setting_FanToSpindleLink, Group_Spindle, "Fan to spindle enable link", NULL, Format_Bitfield, "Fan 0,Fan 1", NULL, NULL, Setting_NonCoreFn, set_spindle_link, get_spindle_link, NULL },
#endif
#if FANS_ENABLE > 2
    { Setting_FanPort2, Group_AuxPorts, "Fan 2 port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
#endif
#if FANS_ENABLE == 3
    { Setting_FanToSpindleLink, Group_Spindle, "Fan to spindle enable link", NULL, Format_Bitfield, "Fan 0,Fan 1,Fan 2", NULL, NULL, Setting_NonCoreFn, set_spindle_link, get_spindle_link, NULL },
#endif
#if FANS_ENABLE > 3
    { Setting_FanPort3, Group_AuxPorts, "Fan 3 port", NULL, Format_Decimal, "-#0", "-1", d_out.port_maxs, Setting_NonCoreFn, set_float, get_float, is_setting_available, { .reboot_required = On } },
#endif
#if FANS_ENABLE == 4
    { Setting_FanToSpindleLink, Group_Spindle, "Fan to spindle enable link", NULL, Format_Bitfield, "Fan 0,Fan 1,Fan 2,Fan 3", NULL, NULL, Setting_NonCoreFn, set_spindle_link, get_spindle_link, NULL },
#endif
};

static const setting_descr_t fan_settings_descr[] = {
    { Setting_Fan0OffDelay, "Delay before turning fan 0 off after program end." },
    { Setting_FanPort0, "Aux output port number to use for fan 0 control. Set to -1 to disable." },
    { Setting_FanToSpindleLink, "Link fan enable signal to spindle enable, fan 0 with optional off delay." },
#if FANS_ENABLE > 1
    { Setting_FanPort1, "Aux output port number to use for fan 1 control. Set to -1 to disable." },
#endif
#if FANS_ENABLE > 2
    { Setting_FanPort2, "Aux output port number to use for fan 2 control. Set to -1 to disable." },
#endif
#if FANS_ENABLE > 3
    { Setting_FanPort3, "Aux output port number to use for fan 3 control. Set to -1 to disable." },
#endif
};

// Write settings to non volatile storage (NVS).
static void fan_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&fan_setting, sizeof(fan_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
// Default is highest numbered free port.
static void fan_settings_restore (void)
{
    uint32_t idx = FANS_ENABLE;

    fan_setting.spindle_link = 0;
    fan_setting.fan0_off_delay = 0.0f;

    do {
        idx--;
        fan_setting.port[idx] = d_out.get_next(&d_out, idx == FANS_ENABLE - 1 ? IOPORT_UNASSIGNED : fan_setting.port[idx + 1], fan_names[idx], (pin_cap_t){});
    } while(idx);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&fan_setting, sizeof(fan_settings_t), true);
}

static void fan_settings_load (void)
{
    uint_fast8_t failed = 0;
    uint_fast8_t idx = FANS_ENABLE;

    spindle_enumerate_spindles(spindle_enumerate, NULL);

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&fan_setting, nvs_address, sizeof(fan_settings_t), true) != NVS_TransferResult_OK)
        fan_settings_restore();

    fans.spindle_link = fan_setting.spindle_link;

    do {
        if(--idx != 0 || fan_spindle_set_state == NULL) {

            if((fans.port[idx] = fan_setting.port[idx]) != IOPORT_UNASSIGNED && d_out.claim(&d_out, &fans.port[idx], fan_names[idx], (pin_cap_t){}))
                n_fans++;
            else {
                failed++;
                fans.port[idx] = IOPORT_UNASSIGNED;
                fans.spindle_link &= ~(1 << idx);
            }
        }
    } while(idx);

    if(n_fans)
        fan_setup();

    if(failed)
        task_run_on_startup(report_warning, "Fans plugin: configured port number(s) not available");
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        report_plugin("Fans", "0.21");
        hal.stream.write("[FANS:");
        hal.stream.write(uitoa(n_fans));
        hal.stream.write("]" ASCII_EOL);
    }
}

void fans_init (void)
{
    static setting_details_t setting_details = {
        .settings = fan_settings,
        .n_settings = sizeof(fan_settings) / sizeof(setting_detail_t),
        .descriptions = fan_settings_descr,
        .n_descriptions = sizeof(fan_settings_descr) / sizeof(setting_descr_t),
        .save = fan_settings_save,
        .load = fan_settings_load,
        .restore = fan_settings_restore
    };

    if(ioports_cfg(&d_out, Port_Digital, Port_Output)->n_ports && (nvs_address = nvs_alloc(sizeof(fan_settings_t)))) {

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

    } else
        task_run_on_startup(report_warning, "Fans plugin failed to initialize!");
}

#endif // FANS_ENABLE
