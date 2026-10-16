// Creality Falcon2: the play key starts the newest file on the SD card.
//
// Without a network there is no way to choose a file, so the machine would be
// useless when its Wi-Fi is not there.  In that degraded mode - FluidNC not
// connected to a Wi-Fi network as a station, typically because it fell back
// to its own access point - a press on the play key (control: cycle_start_pin)
// while the machine is idle starts the most recent G-code file found at the
// root of the SD card.  In every other case the key keeps its normal meaning:
// resume a paused job.
//
// "Most recent" is the latest modification date.  The machine has no clock, so
// the files it receives over Wi-Fi all carry the same date; among files of
// the same date, the last one in the card's directory wins, which is normally
// the last one written.  A card filled from a PC carries real dates.
//
// config.yaml:
//   falcon_play:
//     always: false      true: also when the Wi-Fi is connected
//
// $Falcon/Play tells which file the key would start, without starting it.
//
// Only built for the Falcon2 firmware (-DFALCON2_BOARD).

#ifdef FALCON2_BOARD

#    include "Config.h"
#    include "Module.h"
#    include "Channel.h"
#    include "Settings.h"
#    include "Protocol.h"
#    include "Planner.h"
#    include "Job.h"
#    include "Macro.h"
#    include "FluidPath.h"
#    include "Driver/watchdog.h"

#    include <WiFi.h>
#    include <algorithm>
#    include <cctype>
#    include <filesystem>
#    include <string>

namespace {
    bool is_gcode_name(const std::string& name) {
        if (name.empty() || name[0] == '.' || name.find('&') != std::string::npos) {
            // '&' separates commands in the macro used to start the file
            return false;
        }
        auto dot = name.rfind('.');
        if (dot == std::string::npos) {
            return false;
        }
        std::string ext = name.substr(dot + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
        return ext == "gcode" || ext == "nc" || ext == "gc" || ext == "g" || ext == "ngc" || ext == "tap";
    }

    // Name of the newest G-code file at the root of the SD card, or "" if none.
    std::string newest_file(std::string& why) {
        std::error_code ec;
        FluidPath       root { "", SD, ec };
        if (ec) {
            why = "no SD card";
            return "";
        }
        auto iter = std::filesystem::directory_iterator { root, ec };
        if (ec) {
            why = "cannot read the SD card";
            return "";
        }
        std::string                     best;
        std::filesystem::file_time_type best_time {};
        for (auto const& entry : iter) {
            feed_watchdog();
            std::error_code e2;
            if (entry.is_directory(e2) || e2) {
                continue;
            }
            std::string name = entry.path().filename().string();
            if (!is_gcode_name(name)) {
                continue;
            }
            auto t = std::filesystem::last_write_time(entry.path(), e2);
            if (e2) {
                continue;
            }
            // >= : among files of the same date, the last one listed wins
            if (best.empty() || t >= best_time) {
                best      = name;
                best_time = t;
            }
        }
        if (best.empty()) {
            why = "no G-code file on the SD card";
        }
        return best;
    }

    bool wifi_connected() {
        return WiFi.status() == WL_CONNECTED;
    }
}

class FalconPlay;
static FalconPlay* play = nullptr;

class FalconPlay : public ConfigurableModule {
    bool  _always = false;
    Macro _macro { "falcon_play" };

public:
    FalconPlay(const char* name) : ConfigurableModule(name) {}

    void init() override {
        play = this;
        log_info("falcon_play: play key starts the newest SD file " << (_always ? "always" : "when the Wi-Fi is not connected"));
    }

    bool armed() { return _always || !wifi_connected(); }

    // True if a file was started; false if the key must keep its normal meaning.
    bool try_start() {
        if (!armed() || !state_is(State::Idle) || Job::active() || plan_get_current_block() != nullptr) {
            return false;
        }
        std::string why;
        std::string name = newest_file(why);
        if (name.empty()) {
            log_warn("falcon_play: " << why);
            return false;
        }
        log_info("falcon_play: starting /sd/" << name);
        _macro.set("$SD/Run=/" + name);
        return _macro.run(nullptr);
    }

    Error command(Channel& out) {
        std::string why;
        std::string name = newest_file(why);
        log_stream(out,
                   "[FALCON:play " << (armed() ? "armed" : "not armed") << " wifi=" << (wifi_connected() ? "connected" : "not connected")
                                   << " always=" << (_always ? "true" : "false") << " file=" << (name.empty() ? why : name));
        return Error::Ok;
    }

    // Configuration handlers:
    void validate() override {}
    void afterParse() override {}

    void group(Configuration::HandlerBase& handler) override {
        // @config always
        // @default false
        // false: the play key starts the newest SD file only when the Wi-Fi is not connected.
        // true: also when it is.
        handler.item("always", _always);
    }
};

// The event behind control: cycle_start_pin on this board (see Control.cpp).
static void falcon_play_key() {
    if (play && play->try_start()) {
        return;
    }
    cycleStartEvent.run(nullptr);
}
extern const NoArgEvent falconPlayEvent;
const NoArgEvent        falconPlayEvent { falcon_play_key };

namespace {
    Error falcon_play_command(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (!play) {
            log_string(out, "falcon_play: not configured");
            return Error::Ok;
        }
        return play->command(out);
    }

    ConfigurableModuleFactory::InstanceBuilder<FalconPlay> registration("falcon_play");

    class FalconPlayCommand : public Module {
    public:
        FalconPlayCommand(const char* name) : Module(name) {}
        void init() override { new ReportCommand("FPL", "Falcon/Play", falcon_play_command); }
    };
    ModuleFactory::InstanceBuilder<FalconPlayCommand> __attribute__((init_priority(114))) command_registration("falcon_play_cmd", true);
}

#endif
