// Creality Falcon2 laser head serial link.  See FalconHead.h.

#include "FalconHead.h"
#include "Machine/MachineConfig.h"
#include "Settings.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#ifdef ESP_PLATFORM
#    include <esp_system.h>
#endif

namespace {
    FalconHead* instance = nullptr;

    uint32_t now_ms() {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    const char* reset_reason() {
#ifdef ESP_PLATFORM
        switch (esp_reset_reason()) {
            case ESP_RST_POWERON:
                return "power-on";
            case ESP_RST_SW:
                return "software";
            case ESP_RST_PANIC:
                return "panic";
            case ESP_RST_INT_WDT:
                return "interrupt-watchdog";
            case ESP_RST_TASK_WDT:
                return "task-watchdog";
            case ESP_RST_WDT:
                return "other-watchdog";
            case ESP_RST_BROWNOUT:
                return "brownout";
            case ESP_RST_EXT:
                return "external-pin";
            case ESP_RST_DEEPSLEEP:
                return "deep-sleep";
            default:
                return "unknown";
        }
#else
        return "n/a";
#endif
    }

    Error falcon_status(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (!instance) {
            log_string(out, "falcon_head: not configured");
            return Error::Ok;
        }
        instance->report(out);
        return Error::Ok;
    }
}

void FalconHead::init() {
    if (_uart_num < 0) {
        return;
    }
    if (_uart_num >= MAX_N_UARTS || !config->_uarts[_uart_num]) {
        log_error("falcon_head: missing uart" << _uart_num << " section");
        return;
    }
    _uart = config->_uarts[_uart_num];
    if (!_uart->configured()) {
        log_error("falcon_head: uart" << _uart_num << " failed configuration");
        _uart = nullptr;
        return;
    }
    _protocol = new FalconProtocol(sendToHead, this, _handshake);
    instance  = this;
    new ReportCommand("FH", "Falcon/Status", falcon_status);

    xTaskCreatePinnedToCore(task, "falcon_head", 4096, this, 1, nullptr, SUPPORT_TASK_CORE);
    log_info("falcon_head: uart" << _uart_num << (_handshake ? " handshake on" : " listen only"));
}

void FalconHead::task(void* arg) {
    static_cast<FalconHead*>(arg)->run();
}

void FalconHead::sendToHead(void* ctx, const uint8_t* data, size_t len) {
    static_cast<FalconHead*>(ctx)->_uart->write(data, len);
}

void FalconHead::run() {
    uint8_t buf[64];
    for (;;) {
        size_t   n   = _uart->timedReadBytes(buf, sizeof(buf), pdMS_TO_TICKS(10));
        uint32_t now = now_ms();
        for (size_t i = 0; i < n; i++) {
            _protocol->feed(buf[i], now);
        }
        _protocol->tick(now);
    }
}

void FalconHead::report(Channel& out) {
    const auto& s     = _protocol->stats();
    auto        st    = _protocol->state();
    const char* state = st == FalconProtocol::State::Running ? "running" : st == FalconProtocol::State::Booting ? "booting" : "waiting for head";
    log_stream(out,
               "[FALCON:state=" << state << " handshake=" << (_handshake ? "on" : "off") << " bytes=" << s.bytes << " frames=" << s.frames
                                << " junk=" << s.junk << " bad_end=" << s.bad_end << " sent=" << s.sent << " resets=" << s.resets
                                << " last_rx_ms_ago=" << (s.frames ? now_ms() - _protocol->lastRx() : 0) << " model=" << s.model << "]");
    for (size_t t = 0; t < FalconProtocol::NTypes; t++) {
        if (!s.type_count[t]) {
            continue;
        }
        std::string hex;
        char        b[4];
        for (size_t i = 0; i < s.type_last_len[t]; i++) {
            snprintf(b, sizeof(b), "%02X ", s.type_last[t][i]);
            hex += b;
        }
        log_stream(out, "[FALCON:type=" << t << " count=" << s.type_count[t] << " last=" << hex << "]");
    }
}

// Always present, even with no falcon_head section: adds a line to $I so a
// build carrying this module can be told apart from the stock release.
class FalconBuildInfo : public Module {
public:
    FalconBuildInfo(const char* name) : Module(name) {}

    void build_info(Channel& out) override {
        log_msg_to(out, "Falcon2: falcon_head module included");
        // Why the processor last restarted, and for how long it has been up:
        // tells a brown-out or a watchdog from a requested restart.
        log_msg_to(out, "Falcon2: reset=" << reset_reason() << " uptime_s=" << (xTaskGetTickCount() * portTICK_PERIOD_MS / 1000));
    }
};

// Configuration registration
namespace {
    ConfigurableModuleFactory::InstanceBuilder<FalconHead> registration("falcon_head");
    ModuleFactory::InstanceBuilder<FalconBuildInfo> __attribute__((init_priority(111))) falcon_info_module("falcon_info", true);
}
