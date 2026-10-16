#pragma once

#include "Driver/fluidnc_gpio.h"
#include <system_error>

bool sd_init_slot(uint32_t freq_hz, pinnum_t cs_pin, pinnum_t cd_pin = INVALID_PINNUM, pinnum_t wp_pin = INVALID_PINNUM);
void sd_unmount();
// Falcon2: true if the mounted card still answers (CMD13), without re-initialising it.
bool sd_alive();
// Falcon2: changes the card clock while the card is mounted, to find by experiment
// the highest speed at which block reads are reliable.  Returns false on error.
bool sd_set_clock_khz(uint32_t khz);
void sd_deinit_slot();

// Falcon2: 5 open files at once, like the stock firmware, instead of 2.  With 2, one
// download abandoned by its client plus a running job left none for an upload.
std::error_code sd_mount(uint32_t max_files = 5);
