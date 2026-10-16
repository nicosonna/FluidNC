// Creality Falcon2: SD card read test that does not go through the network.
//
//   $Falcon/SdTest=lecture.gcode      starts reading the file 10 times over
//   $Falcon/SdTest=lecture.gcode,3    3 times
//   $Falcon/SdTest                    progress, then the result
//
// The reads run in their own task, straight from the card, with no web
// request involved.  Comparing this with a download of the same file tells
// whether read failures come from the card and its driver, or from serving
// the file over Wi-Fi at the same time.
//
// Only built for the Falcon2 firmware (-DFALCON2_BOARD).

#ifdef FALCON2_BOARD

#    include "Config.h"
#    include "Module.h"
#    include "Channel.h"
#    include "Settings.h"
#    include "FileStream.h"
#    include "FluidPath.h"

#    include <freertos/FreeRTOS.h>
#    include <freertos/task.h>
#    include <cstdlib>
#    include <cstring>
#    include <string>

namespace {
    struct {
        volatile bool running = false;
        std::string   name;
        int           passes     = 0;
        volatile int  pass       = 0;       // pass being read, 1-based
        volatile uint32_t bytes  = 0;       // bytes read in the current pass
        volatile uint32_t total  = 0;       // bytes read in all passes
        volatile uint32_t ms     = 0;       // elapsed
        uint32_t      first_sum  = 0;
        volatile int  mismatches = 0;       // passes whose checksum differs from the first
        volatile int  short_reads = 0;      // passes that ended before the file size
        std::string   error;
    } test;

    uint32_t now_ms() {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    void task(void*) {
        static char buf[4096];
        uint32_t    start = now_ms();
        for (int pass = 1; pass <= test.passes && test.error.empty(); pass++) {
            test.pass  = pass;
            test.bytes = 0;
            uint32_t sum = 0;
            try {
                FileStream file(test.name.c_str(), "r", SD);
                size_t     size = file.size();
                for (;;) {
                    size_t n = file.read(buf, sizeof(buf));
                    if (n == 0) {
                        break;
                    }
                    if (n > sizeof(buf)) {
                        // read() reports a failure as -1, which arrives here as a
                        // huge count: it must never be used as a length.
                        test.error = "read error at byte " + std::to_string(test.bytes) + " of pass " + std::to_string(pass);
                        break;
                    }
                    for (size_t i = 0; i < n; i++) {
                        sum = sum * 31 + uint8_t(buf[i]);
                    }
                    test.bytes = test.bytes + n;
                    test.total = test.total + n;
                    test.ms    = now_ms() - start;
                    vTaskDelay(1);  // let the other tasks run
                }
                if (test.bytes != size) {
                    test.short_reads = test.short_reads + 1;
                }
            } catch (...) {
                test.error = "cannot open or read the file";
            }
            if (pass == 1) {
                test.first_sum = sum;
            } else if (sum != test.first_sum) {
                test.mismatches = test.mismatches + 1;
            }
        }
        test.ms      = now_ms() - start;
        test.running = false;
        vTaskDelete(nullptr);
    }

    Error command(const char* value, AuthenticationLevel auth_level, Channel& out) {
        if (value && *value) {
            if (test.running) {
                log_string(out, "[FALCON:sdtest already running");
                return Error::Ok;
            }
            std::string arg(value);
            int         passes = 10;
            auto        comma  = arg.find(',');
            if (comma != std::string::npos) {
                passes = atoi(arg.c_str() + comma + 1);
                arg.resize(comma);
            }
            if (passes < 1 || passes > 1000) {
                return Error::NumberRange;
            }
            test.name        = arg;
            test.passes      = passes;
            test.pass        = 0;
            test.bytes       = 0;
            test.total       = 0;
            test.ms          = 0;
            test.mismatches  = 0;
            test.short_reads = 0;
            test.error.clear();
            test.running = true;
            xTaskCreatePinnedToCore(task, "falcon_sdtest", 6144, nullptr, 1, nullptr, SUPPORT_TASK_CORE);
        }
        log_stream(out,
                   "[FALCON:sdtest " << (test.running ? "running" : "idle") << " file=" << test.name << " pass=" << test.pass << "/" << test.passes
                                     << " bytes_this_pass=" << test.bytes << " total=" << test.total << " ms=" << test.ms
                                     << " mismatches=" << test.mismatches << " short_reads=" << test.short_reads
                                     << (test.error.empty() ? "" : " error=") << test.error);
        return Error::Ok;
    }

    class FalconSdTestCommand : public Module {
    public:
        FalconSdTestCommand(const char* name) : Module(name) {}
        void init() override { new ReportCommand("FST", "Falcon/SdTest", command); }
    };
    ModuleFactory::InstanceBuilder<FalconSdTestCommand> __attribute__((init_priority(113))) registration("falcon_sdtest_cmd", true);
}

#endif
