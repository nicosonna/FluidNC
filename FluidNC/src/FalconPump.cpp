// Creality Falcon2 air pump drive.
//
// The pump is a diaphragm pump on an H-bridge fed by two pins.  The stock
// firmware (gas_drive.c) runs both from one 58 Hz, 13-bit LEDC timer:
//
//   GPIO43 duty = 8192 / 2 / 100 * level         high during the first part
//   GPIO44 duty = 8192 - that, output inverted   high during the last part
//
// so the coil gets a pulse one way at the start of each period and a pulse
// the other way at its end: a full alternating square wave at level 100,
// narrower pulses below.  At rest BOTH pins are low.  The inversion of the
// second output matters: without it GPIO44 rests high while GPIO43 rests low,
// and the bridge feeds the coil with direct current - the pump stands still
// and overheats.  The two
// outputs must share a timer to stay in phase, which FluidNC's own PWM pins
// (one timer each) cannot do, hence this module.
//
// config.yaml:
//   falcon_pump:
//     m8_level: 100      pump level while M8 (flood) is on
//     m7_level: 50       pump level while M7 (mist) is on
//     laser_level: 100   pump level while the laser is on (M3 / M4); 0 = do not follow the laser
//     run_on_ms: 5000    the pump keeps running this long after the laser is switched off
// The pump therefore runs during every engraving, whether or not the job asks
// for air with M8, as it does under the stock firmware.  M9 stops the pump
// when the laser is off.  $Falcon/Pump=60 forces a level for tests, and
// $Falcon/Pump=auto goes back to following M7/M8/M9.
//
// Only built for the Falcon2 firmware (-DFALCON2_BOARD).

#ifdef FALCON2_BOARD

#    include "Config.h"
#    include "Module.h"
#    include "Channel.h"
#    include "GCode.h"
#    include "Settings.h"
#    include "Protocol.h"

#    include "Driver/sdspi.h"

#    include <driver/ledc.h>
#    include <esp_wifi.h>
#    include <freertos/FreeRTOS.h>
#    include <freertos/task.h>
#    include <cstdlib>
#    include <cstring>

namespace {
    const gpio_num_t     PUMP_A_GPIO = GPIO_NUM_43;
    const gpio_num_t     PUMP_B_GPIO = GPIO_NUM_44;
    const ledc_mode_t    MODE        = LEDC_LOW_SPEED_MODE;
    // FluidNC hands out channels 0, 2, 4, 6 with timers 0..3 to its own PWM
    // pins; the laser and up to two analog outputs leave timer 3 free.
    const ledc_timer_t   TIMER       = LEDC_TIMER_3;
    const ledc_channel_t CHANNEL_A   = LEDC_CHANNEL_6;
    const ledc_channel_t CHANNEL_B   = LEDC_CHANNEL_7;
    const uint32_t       FULL_SCALE  = 8192;  // 13 bits
    const uint32_t       FREQ_HZ     = 58;
}

class FalconPump;
static FalconPump* pump = nullptr;

class FalconPump : public ConfigurableModule {
    int32_t _m8_level = 100;
    int32_t _m7_level    = 50;
    int32_t _laser_level = 100;
    int32_t _run_on_ms   = 5000;

    uint32_t _laser_seen_ms = 0;      // last time the laser was seen on
    bool     _laser_seen    = false;  // ... and whether it ever was

    int32_t _forced  = -1;  // -1: follow M7 / M8 / M9
    int32_t _current = -1;
    bool    _ready   = false;

    static void task(void* arg) {
        auto self = static_cast<FalconPump*>(arg);
        for (;;) {
            self->update();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    void apply(int32_t level) {
        uint32_t d = uint32_t(FULL_SCALE / 2 / 100.0f * level);
        ledc_set_duty(MODE, CHANNEL_A, d);
        ledc_update_duty(MODE, CHANNEL_A);
        ledc_set_duty(MODE, CHANNEL_B, FULL_SCALE - d);
        ledc_update_duty(MODE, CHANNEL_B);
        _current = level;
    }

    void update() {
        int32_t level = _forced;
        if (level < 0) {
            // The pump never runs in an alarm state, whatever the modal state says.
            if (state_is(State::Alarm) || state_is(State::ConfigAlarm)) {
                level       = 0;
                _laser_seen = false;
            } else {
                level = 0;
                if (gc_state.modal.coolant.Flood) {
                    level = _m8_level;
                } else if (gc_state.modal.coolant.Mist) {
                    level = _m7_level;
                }
                // Laser on (M3 or M4), or switched off less than run_on_ms ago.
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (gc_state.modal.spindle != SpindleState::Disable) {
                    _laser_seen    = true;
                    _laser_seen_ms = now;
                } else if (_laser_seen && now - _laser_seen_ms >= uint32_t(_run_on_ms)) {
                    _laser_seen = false;
                }
                if (_laser_seen && _laser_level > level) {
                    level = _laser_level;
                }
            }
        }
        if (level != _current) {
            apply(level);
        }
    }

public:
    FalconPump(const char* name) : ConfigurableModule(name) {}

    void init() override {
        ledc_timer_config_t timer = {};
        timer.speed_mode          = MODE;
        timer.duty_resolution     = LEDC_TIMER_13_BIT;
        timer.timer_num           = TIMER;
        timer.freq_hz             = FREQ_HZ;
        // All low-speed LEDC timers share one clock source on the ESP32-S3.
        // FluidNC's own PWM pins use the crystal clock: anything else here makes
        // the laser's timer setup fail.
        timer.clk_cfg             = LEDC_USE_XTAL_CLK;
        if (ledc_timer_config(&timer) != ESP_OK) {
            log_error("falcon_pump: timer setup failed");
            return;
        }
        ledc_channel_config_t a = {};
        a.gpio_num              = PUMP_A_GPIO;
        a.speed_mode            = MODE;
        a.channel               = CHANNEL_A;
        a.timer_sel             = TIMER;
        a.duty                  = 0;  // low: pump off
        ledc_channel_config_t b = a;
        b.gpio_num              = PUMP_B_GPIO;
        b.channel               = CHANNEL_B;
        b.duty                  = FULL_SCALE;
        b.flags.output_invert   = 1;  // as the stock firmware: the pin is low at rest
        if (ledc_channel_config(&a) != ESP_OK || ledc_channel_config(&b) != ESP_OK) {
            log_error("falcon_pump: channel setup failed");
            return;
        }
        _current = 0;
        _ready   = true;
        pump     = this;
        xTaskCreatePinnedToCore(task, "falcon_pump", 3072, this, 1, nullptr, SUPPORT_TASK_CORE);
        log_info("falcon_pump: gpio.43 / gpio.44 at " << FREQ_HZ << "Hz, M8 level " << _m8_level << ", M7 level " << _m7_level);
    }

    Error command(const char* value, Channel& out) {
        if (!_ready) {
            log_string(out, "falcon_pump: not running");
            return Error::Ok;
        }
        if (value && *value) {
            if (!strcmp(value, "auto")) {
                _forced = -1;
            } else {
                char* end;
                long  v = strtol(value, &end, 10);
                if (*end || v < 0 || v > 100) {
                    return Error::NumberRange;
                }
                _forced = int32_t(v);
            }
            update();
        }
        log_stream(out, "[FALCON:pump level=" << _current << (_forced < 0 ? " mode=auto" : " mode=forced") << " m8=" << _m8_level << " m7=" << _m7_level << " laser=" << _laser_level);
        return Error::Ok;
    }

    // Configuration handlers:
    void validate() override {}
    void afterParse() override {}

    void group(Configuration::HandlerBase& handler) override {
        // @config m8_level
        // @default 100
        // Pump level, 0 to 100, while M8 (flood coolant) is on.  100 is the stock firmware's maximum.
        handler.item("m8_level", _m8_level, 0, 100);

        // @config m7_level
        // @default 50
        // Pump level, 0 to 100, while M7 (mist coolant) is on.
        handler.item("m7_level", _m7_level, 0, 100);

        // @config laser_level
        // @default 100
        // Pump level, 0 to 100, while the laser is on (M3 or M4).  0: the pump only follows M7 / M8.
        handler.item("laser_level", _laser_level, 0, 100);

        // @config run_on_ms
        // @default 5000
        // Time the pump keeps running after the laser is switched off, in milliseconds.
        handler.item("run_on_ms", _run_on_ms, 0, 60000);
    }
};

namespace {
    Error falcon_pump_command(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (!pump) {
            log_string(out, "falcon_pump: not configured");
            return Error::Ok;
        }
        return pump->command(value, out);
    }

    // $Falcon/SdClock=1000 sets the SD card clock, in kHz, for the mounted card.
    // A test aid: it lasts until the next restart; config.yaml's frequency_hz is
    // what applies at start-up.
    Error falcon_sd_clock_command(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (!value || !*value) {
            return Error::InvalidValue;
        }
        char* end;
        long  khz = strtol(value, &end, 10);
        if (*end || khz < 100 || khz > 20000) {
            return Error::NumberRange;
        }
        bool ok = sd_set_clock_khz(uint32_t(khz));
        log_stream(out, "[FALCON:sd clock " << khz << " kHz " << (ok ? "set" : "refused"));
        return Error::Ok;
    }

    // $Falcon/WifiPower=8 limits the Wi-Fi transmit power, in dBm (2 to 20).
    // A test aid, to see whether the current drawn by Wi-Fi transmissions is
    // what upsets the SD card.  Lasts until the next restart.
    Error falcon_wifi_power_command(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (value && *value) {
            char* end;
            long  dbm = strtol(value, &end, 10);
            if (*end || dbm < 2 || dbm > 20) {
                return Error::NumberRange;
            }
            if (esp_wifi_set_max_tx_power(int8_t(dbm * 4)) != ESP_OK) {
                return Error::InvalidValue;
            }
        }
        int8_t quarter_dbm = 0;
        esp_wifi_get_max_tx_power(&quarter_dbm);
        log_stream(out, "[FALCON:wifi max tx power " << (quarter_dbm / 4) << " dBm");
        return Error::Ok;
    }

    ConfigurableModuleFactory::InstanceBuilder<FalconPump> registration("falcon_pump");

    // Registered once the command tables exist: make_user_commands() runs
    // before the modules are initialised, so this is done from a module.
    class FalconPumpCommand : public Module {
    public:
        FalconPumpCommand(const char* name) : Module(name) {}
        void init() override {
            new ReportCommand("FP", "Falcon/Pump", falcon_pump_command);
            new ReportCommand("FSC", "Falcon/SdClock", falcon_sd_clock_command);
            new ReportCommand("FWP", "Falcon/WifiPower", falcon_wifi_power_command);
        }
    };
    ModuleFactory::InstanceBuilder<FalconPumpCommand> __attribute__((init_priority(112))) command_registration("falcon_pump_cmd", true);
}

#endif
