// Creality Falcon2 board: hold the laser off from the very first moment the
// application runs.
//
// On this board the laser fires when GPIO45 (head power) and GPIO46 (power
// PWM) are both high, and both nets are high-impedance: a weak pull-up on
// each is enough.  FluidNC only takes control of them when it loads
// config.yaml, about a second after reset, and the laser was seen to pulse
// during start-up.  This constructor runs before C++ static initialisation
// and long before the configuration is read: it drives both pins low as real
// outputs, so nothing that happens in between can fire the laser.  FluidNC
// reconfigures them normally afterwards (enable pin and LEDC PWM).
//
// Only built for the Falcon2 firmware (-DFALCON2_BOARD).

#ifdef FALCON2_BOARD

#    include "driver/gpio.h"

#    define FALCON2_LASER_ENABLE_GPIO GPIO_NUM_45
#    define FALCON2_LASER_PWM_GPIO GPIO_NUM_46

static void hold_low(gpio_num_t pin) {
    // Level first, then direction: the pin must never be an output at 1.
    gpio_set_level(pin, 0);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

// Priority 101 is the first one available to applications.
__attribute__((constructor(101))) static void falcon2_laser_off_at_boot(void) {
    hold_low(FALCON2_LASER_PWM_GPIO);
    hold_low(FALCON2_LASER_ENABLE_GPIO);
}

// GPIO43 and GPIO44 are the two inputs of the air pump's H-bridge on this board
// (the stock firmware's gas_drive.c), but they are also the ESP32-S3
// default UART0 pins, where FluidNC puts its console.  Left alone, every log
// line is sent into the pump driver as a burst of edges, and the idle level of
// a UART transmit line is high.  Nothing is wired to that console here, so
// take the pins back and put both low, as the stock firmware leaves them:
// no current in the pump.  One high and one low is NOT a rest state - it
// feeds the coil with direct current, which heats it without moving it.
// Called from setup() once the configuration is loaded and before any
// pin is initialised, so that config.yaml can then claim gpio.43 for the pump.
void falcon2_release_console_pins(void) {
    gpio_reset_pin(GPIO_NUM_43);
    gpio_set_level(GPIO_NUM_43, 0);
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_NUM_44);
    gpio_set_level(GPIO_NUM_44, 0);
    gpio_set_direction(GPIO_NUM_44, GPIO_MODE_OUTPUT);
}

// SD card recovery after a restart.
//
// When the processor restarts while the card is in the middle of a transfer
// (a crash, a firmware update, a watchdog), the card is left waiting in that
// state.  It then ignores the normal initialisation and stays mute until its
// power is cut - which on this machine means switching the whole machine off.
// Before the SPI driver takes the pins, talk to the card by hand: end a
// pending multi-block write (stop token), end a pending read (CMD12), then
// clock until the card releases its data line.  On an idle card all of this
// is ignored.  Pins as wired on the Falcon2 board.

#    include "esp_rom_sys.h"

#    define SD_SCK GPIO_NUM_5
#    define SD_MISO GPIO_NUM_4
#    define SD_MOSI GPIO_NUM_6
#    define SD_CS GPIO_NUM_7

static uint8_t sd_xfer(uint8_t out) {
    uint8_t in = 0;
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(SD_MOSI, (out >> bit) & 1);
        esp_rom_delay_us(4);
        gpio_set_level(SD_SCK, 1);
        in = (uint8_t)((in << 1) | gpio_get_level(SD_MISO));
        esp_rom_delay_us(4);
        gpio_set_level(SD_SCK, 0);
    }
    return in;
}

// Clocks the card until its data line has stayed idle (0xFF) for 16 bytes in
// a row, or `limit` bytes have gone by.  Returns the number of bytes clocked.
static int sd_clock_until_idle(int limit) {
    int idle = 0, n = 0;
    while (n < limit && idle < 16) {
        idle = sd_xfer(0xFF) == 0xFF ? idle + 1 : 0;
        n++;
    }
    return n;
}

// Sends one 6-byte command with the card selected, returns its R1 answer
// (0xFF if none) and clocks `extra` more answer bytes (R3 / R7).
static uint8_t sd_command(const uint8_t* cmd, int extra) {
    uint8_t r1 = 0xFF;
    gpio_set_level(SD_CS, 0);
    for (int i = 0; i < 6; i++) {
        sd_xfer(cmd[i]);
    }
    for (int i = 0; i < 16; i++) {
        uint8_t r = sd_xfer(0xFF);
        if (!(r & 0x80)) {
            r1 = r;
            break;
        }
    }
    for (int i = 0; i < extra; i++) {
        sd_xfer(0xFF);
    }
    gpio_set_level(SD_CS, 1);
    sd_xfer(0xFF);
    return r1;
}

// Diagnostic values read by setup(): time the card took to accept ACMD41
// (-1: not within 0.6 s) and its last answer to it.
int falcon2_sd_ready_ms  = -1;
int falcon2_sd_acmd41_r1 = 0xFF;

// Answer of the card to the last CMD0 sent by falcon2_sd_recover(): 0x01 is
// "idle", 0xFF is no answer at all.
int falcon2_sd_last_r1 = 0xFF;

// Returns how many bytes had to be clocked before the card went idle: a few
// dozen for a card that was idle already, more for one that was stuck.
int falcon2_sd_recover(void) {
    static const uint8_t cmd12[] = { 0x4C, 0x00, 0x00, 0x00, 0x00, 0x61 };
    int                  clocked = 0;

    gpio_reset_pin(SD_CS);
    gpio_reset_pin(SD_SCK);
    gpio_reset_pin(SD_MOSI);
    gpio_reset_pin(SD_MISO);
    gpio_set_level(SD_CS, 1);
    gpio_set_level(SD_SCK, 0);
    gpio_set_level(SD_MOSI, 1);
    gpio_set_direction(SD_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(SD_SCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(SD_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(SD_MISO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SD_MISO, GPIO_PULLUP_ONLY);

    for (int i = 0; i < 10; i++) {
        sd_xfer(0xFF);  // 80 clocks with the card deselected
    }

    gpio_set_level(SD_CS, 0);
    sd_xfer(0xFD);  // stop token: ends a multi-block write
    clocked += sd_clock_until_idle(4000);
    for (unsigned i = 0; i < sizeof(cmd12); i++) {
        sd_xfer(cmd12[i]);  // CMD12: ends a multi-block read
    }
    clocked += sd_clock_until_idle(4000);
    gpio_set_level(SD_CS, 1);
    sd_xfer(0xFF);
    sd_xfer(0xFF);

    // Put the card back in the idle state ourselves.  A card that is still
    // powered and was already initialised by the previous run did not answer
    // the driver's own CMD0 after a restart; repeating it by hand until the
    // card reports "idle" (R1 = 0x01) leaves it as it is after power-on.
    falcon2_sd_last_r1 = 0xFF;
    for (int attempt = 0; attempt < 10 && falcon2_sd_last_r1 != 0x01; attempt++) {
        static const uint8_t cmd0[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 };
        gpio_set_level(SD_CS, 0);
        for (unsigned i = 0; i < sizeof(cmd0); i++) {
            sd_xfer(cmd0[i]);
        }
        for (int i = 0; i < 16; i++) {
            uint8_t r = sd_xfer(0xFF);
            if (!(r & 0x80)) {
                falcon2_sd_last_r1 = r;
                break;
            }
        }
        gpio_set_level(SD_CS, 1);
        sd_xfer(0xFF);
        clocked += 24;
    }

    // Diagnostic.  A card that is still powered after a restart of the processor
    // answers CMD0 but never finishes its initialisation: ACMD41 keeps returning
    // "idle", for 8 s and more, at any clock speed, until the card loses power.
    // A short poll here records which case this start-up is in, so the log can
    // say "power-cycle the card" instead of a bare timeout.
    falcon2_sd_ready_ms = -1;
    if (falcon2_sd_last_r1 == 0x01) {
        static const uint8_t cmd8[]   = { 0x48, 0x00, 0x00, 0x01, 0xAA, 0x87 };
        static const uint8_t cmd55[]  = { 0x77, 0x00, 0x00, 0x00, 0x00, 0x65 };
        static const uint8_t acmd41[] = { 0x69, 0x40, 0x00, 0x00, 0x00, 0x77 };
        sd_command(cmd8, 4);
        for (int ms = 0; ms < 600; ms += 20) {
            sd_command(cmd55, 0);
            falcon2_sd_acmd41_r1 = sd_command(acmd41, 0);
            if (falcon2_sd_acmd41_r1 == 0x00) {
                falcon2_sd_ready_ms = ms;
                break;
            }
            esp_rom_delay_us(20000);
        }
    }

    // Hand the pins back untouched for the SPI driver.
    gpio_reset_pin(SD_SCK);
    gpio_reset_pin(SD_MOSI);
    gpio_reset_pin(SD_MISO);
    gpio_reset_pin(SD_CS);
    return clocked;
}

#endif
