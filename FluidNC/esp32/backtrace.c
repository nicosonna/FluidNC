// Copyright 2026 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Driver/backtrace.h"

#ifdef FALCON2_SAVE_BACKTRACE
// Falcon2: the board has no reachable serial console, so a panic leaves no
// trace at all.  The panic handler is wrapped (-Wl,--wrap=esp_panic_handler)
// to store the faulting PC and the call chain in RTC memory, which survives
// the restart; $Backtrace/Show then prints them.

#    include <string.h>
#    include "esp_attr.h"
#    include "esp_debug_helpers.h"
#    include "esp_cpu_utils.h"
#    include "esp_private/panic_internal.h"
#    include "xtensa_context.h"

#    define BACKTRACE_MAGIC 0xB7ACE001

static RTC_NOINIT_ATTR backtrace_t saved;
static RTC_NOINIT_ATTR uint32_t    saved_magic;

void __real_esp_panic_handler(panic_info_t* info);

void __wrap_esp_panic_handler(panic_info_t* info) {
    const XtExcFrame* f = (const XtExcFrame*)info->frame;
    if (f) {
        esp_backtrace_frame_t frame = { .pc = f->pc, .sp = f->a1, .next_pc = f->a0, .exc_frame = f };
        size_t                n     = 0;

        saved.pc       = f->pc;
        saved.excvaddr = f->excvaddr;
        saved.exccause = f->exccause;

        saved.addresses[n++] = esp_cpu_process_stack_pc(frame.pc);
        while (n < BACKTRACE_MAX_ADDRESSES && frame.next_pc != 0 && esp_backtrace_get_next_frame(&frame)) {
            saved.addresses[n++] = esp_cpu_process_stack_pc(frame.pc);
        }
        saved.num_addresses = n;
        saved_magic         = BACKTRACE_MAGIC;
    }
    __real_esp_panic_handler(info);
}

bool backtrace_available(void) {
    return saved_magic == BACKTRACE_MAGIC && saved.num_addresses <= BACKTRACE_MAX_ADDRESSES;
}

bool backtrace_get(backtrace_t* bt) {
    if (!backtrace_available()) {
        return false;
    }
    memcpy(bt, &saved, sizeof(saved));
    return true;
}

void backtrace_clear(void) {
    saved_magic = 0;
}

#else

bool backtrace_available(void) {
    return false;
}

bool backtrace_get(backtrace_t* bt) {
    (void)bt;
    return false;
}

void backtrace_clear(void) {}

#endif
