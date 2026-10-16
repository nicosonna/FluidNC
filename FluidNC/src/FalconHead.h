// Creality Falcon2 laser head serial link.
// The head has its own MCU and expects a controller to answer on a UART:
// a fixed boot sequence, then an echo of every frame it sends.  Without that
// dialogue the laser still fires, but the head's sensors (flame, air flow,
// temperature) stay in a fault display and report nothing.
// The protocol itself lives in FalconProtocol.h.
#pragma once

#include "Config.h"
#include "Module.h"
#include "Channel.h"
#include "Uart.h"
#include "FalconProtocol.h"

class FalconHead : public ConfigurableModule {
    int32_t _uart_num  = -1;
    bool    _handshake = false;  // false = listen only, never transmit

    Uart*           _uart     = nullptr;
    FalconProtocol* _protocol = nullptr;

    static void task(void* arg);
    static void sendToHead(void* ctx, const uint8_t* data, size_t len);
    void        run();

public:
    FalconHead(const char* name) : ConfigurableModule(name) {}

    FalconHead(const FalconHead&)            = delete;
    FalconHead(FalconHead&&)                 = delete;
    FalconHead& operator=(const FalconHead&) = delete;
    FalconHead& operator=(FalconHead&&)      = delete;

    virtual ~FalconHead() = default;

    void init() override;
    void report(Channel& out);

    // Configuration handlers:
    void validate() override {}
    void afterParse() override {}

    void group(Configuration::HandlerBase& handler) override {
        // @config uart_num
        // @default -1
        // Number of the uartN section wired to the laser head (9600 8N1 on the stock board).
        handler.item("uart_num", _uart_num);

        // @config handshake
        // @default false
        // false only listens to the head. true also answers it: boot sequence, then echo.
        handler.item("handshake", _handshake);
    }
};
