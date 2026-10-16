// Creality Falcon2 laser head serial protocol: framing and controller dialogue.
// Pure C++ with no FluidNC dependency, so it is unit-tested on the PC
// (falcon2/tests/cpp).  FalconHead.cpp wires it to a UART and a task.
// Protocol reference: github.com/drewgreenwell/Falcon2Universal (MIT).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

class FalconProtocol {
public:
    static const size_t MaxFrame = 272;  // 4 magic + 5 header + 255 data + LF, rounded up
    static const size_t KeptLen  = 40;   // bytes of each frame kept for reporting
    static const size_t NTypes   = 16;

    static const size_t Header    = 9;  // magic, length, index, type, ctrl1, ctrl2
    static const size_t LenPos    = 4;
    static const size_t IdxPos    = 5;
    static const size_t TypePos   = 6;
    static const size_t LongFrame = 100;  // the model-name frame is not echoed

    static const uint32_t BootDelayMs    = 1185 + 180;  // first frame seen -> first boot frame
    static const uint32_t BootFirstGapMs = 180;
    static const uint32_t BootGapMs      = 212;
    static const uint32_t EchoDelayMs    = 30;
    static const uint32_t IdleTimeoutMs  = 5000;

    enum class State : uint8_t { WaitFirstFrame, Booting, Running };

    typedef void (*SendFn)(void* ctx, const uint8_t* data, size_t len);

    struct Stats {
        uint32_t bytes   = 0;
        uint32_t junk    = 0;  // bytes dropped while hunting for the frame magic
        uint32_t bad_end = 0;  // frames whose last byte was not LF
        uint32_t frames  = 0;
        uint32_t sent    = 0;
        uint32_t resets  = 0;
        uint32_t type_count[NTypes] = { 0 };
        uint8_t  type_last[NTypes][KeptLen];
        uint8_t  type_last_len[NTypes] = { 0 };
        char     model[24]             = { 0 };
    };

private:
    SendFn _send;
    void*  _ctx;
    bool   _handshake;  // false = listen only, never transmit

    uint8_t _rx[MaxFrame];
    size_t  _rx_len = 0;

    // Previous frame, echoed in place of the long model-name frame
    uint8_t _prev[MaxFrame];
    size_t  _prev_len = 0;

    // Echo waiting for its delay
    uint8_t  _echo[MaxFrame];
    size_t   _echo_len = 0;
    uint32_t _echo_due = 0;

    State    _state     = State::WaitFirstFrame;
    uint32_t _next_boot = 0;
    uint8_t  _boot_step = 0;
    uint8_t  _tx_index  = 0;
    uint32_t _last_rx   = 0;

    Stats _stats;

    static const uint8_t* magic() {
        static const uint8_t m[4] = { 0x49, 0x4C, 0x6D, 0x70 };  // "ILmp"
        return m;
    }

    void send(const uint8_t* frame, size_t len) {
        uint8_t out[MaxFrame];
        memcpy(out, frame, len);
        out[IdxPos] = _tx_index++;
        if (_handshake) {
            _send(_ctx, out, len);
            _stats.sent++;
        }
    }

    void flushEcho() {
        if (_echo_len) {
            send(_echo, _echo_len);
            _echo_len = 0;
        }
    }

    void onFrame(uint32_t now) {
        _last_rx = now;
        _stats.frames++;

        uint8_t type = _rx[TypePos] < NTypes ? _rx[TypePos] : NTypes - 1;
        _stats.type_count[type]++;
        _stats.type_last_len[type] = _rx_len < KeptLen ? _rx_len : KeptLen;
        memcpy(_stats.type_last[type], _rx, _stats.type_last_len[type]);

        bool is_long = _rx_len >= LongFrame;
        if (is_long) {
            size_t n = sizeof(_stats.model) - 1;
            memcpy(_stats.model, _rx + 8, n);
            _stats.model[n] = 0;
        }

        switch (_state) {
            case State::WaitFirstFrame:
                _state     = State::Booting;
                _boot_step = 0;
                _tx_index  = 0;
                _prev_len  = 0;
                _echo_len  = 0;
                _next_boot = now + BootDelayMs;
                break;
            case State::Booting:
                // The stock controller sends nothing here but still counts the frame.
                _tx_index++;
                break;
            case State::Running:
                flushEcho();  // never hold more than one echo back
                if (is_long) {
                    memcpy(_echo, _prev, _prev_len);
                    _echo_len = _prev_len;
                } else {
                    memcpy(_prev, _rx, _rx_len);
                    _prev_len = _rx_len;
                    memcpy(_echo, _rx, _rx_len);
                    _echo_len = _rx_len;
                }
                _echo_due = now + EchoDelayMs;
                break;
        }
    }

    void bootTick(uint32_t now) {
        static const uint8_t boot1[] = { 0x49, 0x4C, 0x6D, 0x70, 0x08, 0x00, 0x07, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A };
        static const uint8_t boot2[] = { 0x49, 0x4C, 0x6D, 0x70, 0x04, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0A };
        static const uint8_t boot3[] = { 0x49, 0x4C, 0x6D, 0x70, 0x00, 0x0A, 0x01, 0x00, 0x00, 0x0A };
        static const uint8_t order[] = { 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 3 };

        if (int32_t(now - _next_boot) < 0) {
            return;
        }
        if (_boot_step >= sizeof(order)) {
            _state = State::Running;
            return;
        }
        switch (order[_boot_step]) {
            case 1:
                send(boot1, sizeof(boot1));
                break;
            case 2:
                send(boot2, sizeof(boot2));
                break;
            default:
                send(boot3, sizeof(boot3));
                break;
        }
        _next_boot = now + (_boot_step == 0 ? BootFirstGapMs : BootGapMs);
        _boot_step++;
    }

public:
    FalconProtocol(SendFn send, void* ctx, bool handshake) : _send(send), _ctx(ctx), _handshake(handshake) {}

    State        state() const { return _state; }
    const Stats& stats() const { return _stats; }
    bool         handshake() const { return _handshake; }
    uint32_t     lastRx() const { return _last_rx; }

    // One received byte.  Frames are length-prefixed binary, so they are cut
    // by length and not on LF, which can also appear inside the data.
    void feed(uint8_t c, uint32_t now) {
        _stats.bytes++;
        if (_rx_len < 4) {
            if (c != magic()[_rx_len]) {
                _stats.junk += _rx_len + 1;
                _rx_len = 0;
                if (c != magic()[0]) {
                    return;
                }
                _stats.junk--;
            }
            _rx[_rx_len++] = c;
            return;
        }
        _rx[_rx_len++] = c;
        if (_rx_len < Header) {
            return;
        }
        if (_rx_len < Header + _rx[LenPos] + 1) {
            return;
        }
        if (c != 0x0A) {
            _stats.bad_end++;
        }
        onFrame(now);
        _rx_len = 0;
    }

    // Call regularly (every 10 ms or so) with the current time in milliseconds.
    void tick(uint32_t now) {
        if (_state == State::Booting) {
            bootTick(now);
        } else if (_state == State::Running) {
            if (_echo_len && int32_t(now - _echo_due) >= 0) {
                flushEcho();
            }
            if (now - _last_rx > IdleTimeoutMs) {
                // Head switched off or unplugged: start over on its next frame.
                _state    = State::WaitFirstFrame;
                _echo_len = 0;
                _stats.resets++;
            }
        }
    }
};
