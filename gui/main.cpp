#include "loader/ldr.h"
#include "cpu/cpu.h"
#include "cpu/dma.h"
#include "cpu/twi.h"
#include "cpu/sport.h"
#include "peripheral/MT29F4G08.h"
#include "utils/log.h"
#include "glfw_display.h"
#include "headless.h"
#include "boot_keys.h"
#include "audio_output_miniaudio.h"
#include "usbipd.h"
#include "test_audio_input.h"
#include "build_info.h"
#ifdef ENABLE_PROFILING
#include "profiling/profiler.h"
#endif
#include <algorithm>
#include <vector>
#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <string>
#include <optional>
#include <stdexcept>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <climits>
#include <cerrno>
#include <cstdio>
#include <new>
#include <sys/stat.h>
#include <sys/wait.h>

std::atomic<bool> cpuShouldStop(false);

// Headless runs are stopped with SIGTERM/SIGINT. Default termination bypasses
// destructors and loses buffered output, so request a clean shutdown instead.
// The handler only stores to a lock-free atomic on the supported host.
static_assert(std::atomic<bool>::is_always_lock_free);
// fork-run: the CPU thread parks between Run() calls while cpuPause is set (cpuPaused acknowledges it).
static std::atomic<bool> cpuPause{false}, cpuPaused{false};
static bool forkChild = false;

static void ParkIfPaused() {
    if (!cpuPause.load()) return;
    cpuPaused.store(true);
    while (cpuPause.load() && !cpuShouldStop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    cpuPaused.store(false);
}

static void StopSigHandler(int) {
    cpuShouldStop.store(true);
}

void LdrExecutionThread(BlackFinCpu& cpu, const LDRParser& parser) {
    const auto& dxes = parser.getDXEs();
    for (const auto& dxe : dxes) {
        for (const auto& block : dxe.blocks) {
            if (!block.data.empty()) {
                cpu.GetEmulator().MemoryWrite(
                    block.header.target_address,
                    block.data.data(),
                    block.data.size()
                );
            }
            if (block.IsFirstBlock()) {
                cpu.SetRegister(RegIndex::RETS, 0x8000000);
                cpu.SetPC(block.header.target_address);
            }
        }
        LogInfo("Start executing DXE");

        while (!cpuShouldStop.load()) {
            cpu.Run();
            if (cpu.Failed()) cpuShouldStop.store(true);

            if (cpu.PC() == 0x8000000) {
                LogInfo("Finished executing DXE");
                break;
            }
            if (cpu.PC() == 0xffa06e54) {
                LogInfo("Hit delay(%d)", cpu.GetRegister(RegIndex::R0));
            }
            if (cpu.PC() == 0xffa06eec) {
                LogInfo("Hit delay end");
            }
        }

        if (cpuShouldStop.load()) {
            break;
        }
    }
    cpuShouldStop.store(true);
    LogInfo("CPU thread exiting");
}

void BootExcutionThread(BlackFinCpu& cpu, bool resume) {
    if (!resume) cpu.SetPC(0xEF000000); // Boot entry point (a fork-run child resumes where the parent paused)
    while (!cpuShouldStop.load()) {
        ParkIfPaused();
        cpu.Run();
        if (cpu.Failed()) cpuShouldStop.store(true);
        if (cpu.ResetRequested()) { LogInfo("CPU thread exiting for a system reset"); return; }
    }
    cpuShouldStop.store(true);
    LogInfo("CPU thread exiting");
}

static int RunEmulator(int argc, char* argv[]) {
    // Separate flags from positional args so flag order relative to
    // <nand_flash_file> [ldr_file] doesn't matter.
    std::vector<std::string> positional;
    // Persist guest writes by default, matching real hardware. --nand-snapshot
    // opens the image copy-on-write (backing file untouched) for disposable
    // runs or deterministic tests; --nand-rw is accepted as an explicit
    // persistent-mode selector.
    bool nandSnapshot = false;
    bool headless = false;
    bool teBoot = false;
    bool deterministic = false;
    bool realtimeFlag = false;
    std::optional<long long> rtcEpoch;
    unsigned twiSpeedup = TWI::kDefaultSpeedup;
    unsigned mdmaSpeedup = DMA::kDefaultMdmaSpeedup;
    unsigned audioBufferMs = MiniaudioOutput::kDefaultBufferMs;
    bool help = false;
    bool options = true;
    std::string inputScript;
    // Profiling options (profiling builds only; see docs/profiling.md).
    std::string profileMarks, profileTrace, profileCensus, perfFifo, perfWindow, profileUntil;
    std::string profileChecksum, profileChecksumEvery, profileChecksumFrom, profileDumpIr, profileDumpDir;
    std::string profileTraceEvents, profileEndCycles;
    bool perfJitdump = false;
    bool profileCounters = false;
    bool noChain = false;
    bool noLoopBlocks = false;
    bool noIdleWait = false;
    bool noDmaSettle = false;
    int steadySkip = 2;
    long long tierThreshold = BlackFinCpu::kTierThreshold;
    long long tierSizeRef = BlackFinCpu::kTierSizeRef;
    bool lightTier1 = true;
    bool tierBackground = true;
    bool copyPatch = true;
    bool copyPatchHints = true;
    long long tierPublishAfter = 0;
    long long jitCpu = -1;
    bool chainDry = false;
    bool markDigest = false;
    bool profileProbes = false;
    const std::pair<const char*, std::string*> valueOptions[] = {
        {"--profile-marks", &profileMarks}, {"--profile-trace", &profileTrace},
        {"--profile-census", &profileCensus}, {"--perf-ctl-fifo", &perfFifo},
        {"--perf-window", &perfWindow}, {"--profile-until", &profileUntil},
        {"--profile-checksum", &profileChecksum}, {"--profile-checksum-every", &profileChecksumEvery},
        {"--profile-checksum-from", &profileChecksumFrom},
        {"--profile-dump-ir", &profileDumpIr}, {"--profile-dump-dir", &profileDumpDir},
        {"--profile-trace-events", &profileTraceEvents}, {"--profile-end-cycles", &profileEndCycles},
    };
    bool anyProfiling = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (options && arg == "--") options = false;
        else if (options && arg == "--nand-rw") nandSnapshot = false;
        else if (options && arg == "--nand-snapshot") nandSnapshot = true;
        else if (options && arg == "--headless") headless = true;
        else if (options && arg == "--te-boot") teBoot = true;
        else if (options && arg == "--deterministic") deterministic = true;
        // Older scripts' flags: guest time is paced to the host's clock by default now (--realtime was
        // that), and blocks compile cheaply first instead of coming from a persistent cache.
        else if (options && arg == "--realtime") realtimeFlag = true;
        else if (options && arg == "--no-jit-cache") {}
        else if (options && arg == "--jit-cache") {
            if (++i == argc || !*argv[i]) throw std::runtime_error("--jit-cache requires a directory");
        }
        else if (options && arg == "--no-chain") noChain = true;
        else if (options && arg == "--no-loop-blocks") noLoopBlocks = true;
        else if (options && arg == "--no-idle-wait") noIdleWait = true;
        else if (options && arg == "--no-dma-settle") noDmaSettle = true;
        else if (options && arg == "--steady-skip") {
            if (++i == argc) throw std::runtime_error("--steady-skip requires off, dry or on");
            const std::string mode = argv[i];
            if (mode == "off") steadySkip = 0;
            else if (mode == "dry") steadySkip = 1;
            else if (mode == "on") steadySkip = 2;
            else throw std::runtime_error("Invalid --steady-skip (off, dry, on): " + mode);
        }
        else if (options && (arg == "--tier-threshold" || arg == "--tier-size-ref")) {
            if (++i == argc) throw std::runtime_error(arg + " requires a count (0 for none)");
            size_t used = 0;
            long long value = -1;
            try { value = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || value < 0 || value > UINT32_MAX)
                throw std::runtime_error("Invalid " + arg + ": " + argv[i]);
            (arg == "--tier-threshold" ? tierThreshold : tierSizeRef) = value;
        }
        else if (options && arg == "--tier1-pipeline") {
            if (++i == argc) throw std::runtime_error("--tier1-pipeline requires light or o1");
            const std::string pipeline = argv[i];
            if (pipeline == "light") lightTier1 = true;
            else if (pipeline == "o1") lightTier1 = false;
            else throw std::runtime_error("Invalid --tier1-pipeline (light, o1): " + pipeline);
        }
        else if (options && (arg == "--tier-publish-after" || arg == "--jit-cpu")) {
            if (++i == argc) throw std::runtime_error(arg + " requires a number");
            size_t used = 0;
            long long value = -1;
            try { value = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || value < 0)
                throw std::runtime_error("Invalid " + arg + ": " + argv[i]);
            (arg == "--jit-cpu" ? jitCpu : tierPublishAfter) = value;
        }
        else if (options && arg == "--no-tier-background") tierBackground = false;
        else if (options && arg == "--no-copy-patch") copyPatch = false;
        else if (options && arg == "--no-cp-hints") copyPatchHints = false;
        else if (options && arg == "--profile-chain-dry") chainDry = anyProfiling = true;
        else if (options && arg == "--profile-mark-digest") markDigest = anyProfiling = true;
        else if (options && arg == "--rtc-epoch") {
            if (++i == argc) throw std::runtime_error("--rtc-epoch requires seconds since 1970");
            size_t used = 0;
            long long seconds = -1;
            try { seconds = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || seconds < 0)
                throw std::runtime_error(std::string("Invalid --rtc-epoch: ") + argv[i]);
            if (seconds > TimeSource::kMaxEpochSeconds)
                throw std::runtime_error(std::string("--rtc-epoch past the RTC's range (2059-09-18, ") +
                                         std::to_string(TimeSource::kMaxEpochSeconds) + "): " + argv[i]);
            rtcEpoch = seconds;
        }
        else if (options && arg == "--twi-speedup") {
            if (++i == argc) throw std::runtime_error("--twi-speedup requires a factor");
            size_t used = 0;
            long long factor = -1;
            try { factor = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || factor < 1 || factor > 100000)
                throw std::runtime_error(std::string("Invalid --twi-speedup (1..100000): ") + argv[i]);
            twiSpeedup = static_cast<unsigned>(factor);
        }
        else if (options && arg == "--mdma-speedup") {
            if (++i == argc) throw std::runtime_error("--mdma-speedup requires a factor");
            size_t used = 0;
            long long factor = -1;
            try { factor = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || factor < 1 || factor > 100000)
                throw std::runtime_error(std::string("Invalid --mdma-speedup (1..100000): ") + argv[i]);
            mdmaSpeedup = static_cast<unsigned>(factor);
        }
        else if (options && arg == "--audio-buffer") {
            if (++i == argc) throw std::runtime_error("--audio-buffer requires milliseconds");
            size_t used = 0;
            long long ms = -1;
            try { ms = std::stoll(argv[i], &used); } catch (const std::exception&) {}
            if (used == 0 || argv[i][used] != '\0' || ms < MiniaudioOutput::kMinBufferMs ||
                ms > MiniaudioOutput::kMaxBufferMs)
                throw std::runtime_error("Invalid --audio-buffer (" + std::to_string(MiniaudioOutput::kMinBufferMs) +
                                         ".." + std::to_string(MiniaudioOutput::kMaxBufferMs) + "): " + argv[i]);
            audioBufferMs = static_cast<unsigned>(ms);
        }
        else if (options && arg == "--perf-jitdump") perfJitdump = anyProfiling = true;
        else if (options && arg == "--profile-counters") profileCounters = anyProfiling = true;
        else if (options && arg == "--profile-probes") profileProbes = anyProfiling = true;
        else if (options && std::any_of(std::begin(valueOptions), std::end(valueOptions),
                                        [&](const auto& option) { return arg == option.first; })) {
            if (++i == argc || !*argv[i]) throw std::runtime_error(arg + " requires a nonempty value");
            for (const auto& [name, value] : valueOptions)
                if (arg == name) *value = argv[i];
            anyProfiling = true;
        }
        else if (options && (arg == "--help" || arg == "-h")) help = true;
        else if (options && arg == "--build-info") {
            std::cout << BuildInfoJson() << std::endl;
            return 0;
        }
        else if (options && arg == "--input-script") {
            if (++i == argc) throw std::runtime_error("--input-script requires a path");
            inputScript = argv[i];
            if (inputScript.empty()) throw std::runtime_error("--input-script requires a nonempty path");
        } else if (options && arg.rfind("-", 0) == 0) {
            throw std::runtime_error("Unknown option: " + arg);
        } else positional.push_back(arg);
    }

    if (help || positional.empty()) {
        std::cout << "Usage: " << argv[0]
                  << " <nand_flash_file> [ldr_file] [--nand-rw|--nand-snapshot]\n"
                     "  --headless            No GLFW window or host audio device\n"
                     "  --te-boot             Hold COM at startup to enter the te-boot menu\n"
                     "  --input-script PATH   Read headless commands from PATH instead of stdin\n"
                     "  --deterministic       Run as fast as the host allows, not at the device's\n"
                     "                        pace: repeatable runs, for measurement and tests\n"
                     "  --realtime, --jit-cache DIR, --no-jit-cache  Accepted for older scripts;\n"
                     "                        no effect (pacing is the default, no JIT cache)\n"
                     "  --rtc-epoch SECONDS   RTC time at boot (default: the host's; 2024-01-01\n"
                     "                        with --deterministic)\n"
                     "  --no-chain            Return to the host after every guest block\n"
                     "  --no-idle-wait        Run an IDLE as a NOP instead of\n"
                     "                        letting guest time pass to the next wakeup\n"
                     "  --steady-skip MODE    on (default), dry or off: a loop whose iteration changed\n"
                     "                        nothing but its counters goes to its last iteration\n"
                     "                        (dry: only counts the iterations it would skip)\n"
                     "  --no-dma-settle       Move each memory DMA element at its own time instead\n"
                     "                        of when something could see it (same transfers, slower)\n"
                     "  --tier-threshold N    Compile a block with the whole pipeline once it\n"
                     "                        started N times, cheaply before (default 30000;\n"
                     "                        0: every block once, with the whole pipeline)\n"
                     "  --tier-size-ref N     A block of more than N packets waits as many times\n"
                     "                        longer as it has packets more (default 16; 0: the\n"
                     "                        same threshold for every block)\n"
                     "  --tier1-pipeline P    light (default) or o1: the IR pipeline of those second\n"
                     "                        compilations, but for loop blocks, which get O1\n"
                     "  --no-tier-background  Compile those blocks again on the CPU thread, not on\n"
                     "                        a thread of their own\n"
                     "  --no-copy-patch       Make the cheap compilations with LLVM, not from\n"
                     "                        stencils where the instructions allow\n"
                     "  --no-cp-hints         Make those from stencils that rely on nothing the\n"
                     "                        loop registers held when they were made\n"
                     "  --tier-publish-after N  With --deterministic, use such a block's second\n"
                     "                        compilation N packets after it was asked for\n"
                     "                        (default 8388608; the CPU thread waits for it)\n"
                     "  --jit-cpu N           Run the second compilations on CPU N\n"
                     "  --no-loop-blocks      Keep a guest loop's registers in memory across it\n"
                     "                        (bcore loop blocks off; same results, slower)\n"
                     "  --twi-speedup N       Run the I2C bus N times faster than programmed\n"
                     "                        (default 10; 1 is the hardware's rate)\n"
                     "  --mdma-speedup N      Run memory DMA N times faster than the hardware\n"
                     "                        (default 10; 1 is the hardware's rate)\n"
                     "  --audio-buffer MS     Audio latency, from the emulator to the sound card's\n"
                     "                        output (default 26, 9..200; more: later sound,\n"
                     "                        fewer gaps on a busy host)\n"
                     "  --build-info          Print what this binary was built from (JSON) and exit\n"
                     "Profiling builds (-DENABLE_PROFILING=ON, see docs/profiling.md):\n"
                     "  --profile-marks FILE  Phase marks (firmware PC ranges, verified frame)\n"
                     "  --profile-trace FILE  Chrome/Perfetto trace of translations and marks\n"
                     "  --profile-census PFX  Block, edge and MMIO census at every mark (diagnostic)\n"
                     "  --profile-until MARK  Stop cleanly when MARK fires\n"
                     "  --profile-end-cycles N  Mark 'end' at the first block boundary at or past N\n"
                     "                        guest cycles (a phase end that keystrokes cannot name)\n"
                     "  --perf-ctl-fifo FIFO  perf record --control=fifo:FIFO\n"
                     "  --perf-window A:B     Enable perf at mark A, disable at mark B\n"
                     "  --perf-jitdump        Write LLVM jitdump for perf inject --jit\n"
                     "  --profile-counters    CPU thread's user-space instructions and cycles at every mark\n"
                     "  --profile-checksum F  Guest-state checksum every N blocks (diagnostic)\n"
                     "  --profile-probes      Capture the marks file's probes (diagnostic)\n"
                     "  --profile-mark-digest Hashes of guest registers, CEC and RAM at every mark\n"
                     "  --profile-chain-dry   Blocks only report that they would chain; check that\n"
                     "                        the host then has nothing to do ([chain] violations)\n"
                     "  --profile-dump-ir PCS Write the LLVM IR of blocks at these PCs (comma-separated hex)\n"
                     "  --profile-dump-dir D  Directory for those files (default ir)\n"
                     "  --profile-checksum-every N  Blocks between checksums (default 65536)\n"
                     "  --profile-checksum-from N   First block to checksum (default 0)\n"
                     "  --profile-trace-events N    Trace capacity in events (default 262144)\n"
                     "Headless commands (one per line):\n"
                     "  keys | press KEY | release KEY | tap KEY [MS] | wait MS | quit\n"
                     "  at MS: later press/release/tap are for guest time MS (before the script's\n"
                     "         first wait: queued before the CPU starts)\n"
                     "  accel X Y Z | accel random | volume 0..255\n"
                     "  wait-frames N [TIMEOUT_MS] | screenshot PATH.ppm\n"
                     "  turn ENC cw|ccw [N [FRAMES]] | wheel ENC NOTCHES | hold KEY FRAMES\n"
                     "  audio-pos PATH | audio-anchor | wait-audio N|@N [MS] | barrier PATH\n"
                     "  poke ADDR VALUE | dump-mem ADDR LEN PATH | snapshot-mem ADDR LEN PATH [...] (hex)\n"
                     "  fork-run SCRIPT OUTDIR (run SCRIPT in a forked copy of the machine as it is now, in\n"
                     "  OUTDIR; the parent goes on from the same state afterwards)\n"
                     "With --headless --deterministic the CPU stops at the frame a wait-frames, hold or turn\n"
                     "waits for until the commands after it are queued (lockstep).\n"
                     "Keys are button names from gui/ui.json. Tap defaults to 100ms.\n"
                     "Wait/tap use wall time. EOF leaves the emulator running; quit or Ctrl-C stops it.\n";
        return help ? 0 : 1;
    }
    if (positional.size() > 2) throw std::runtime_error("Expected NAND and optional LDR paths");
    if (realtimeFlag && deterministic)
        throw std::runtime_error("--realtime (paced, the default) and --deterministic (unpaced) contradict");
    if (!inputScript.empty() && !headless)
        throw std::runtime_error("--input-script requires --headless");
#ifndef ENABLE_PROFILING
    if (anyProfiling)
        throw std::runtime_error("Profiling options need a build with -DENABLE_PROFILING=ON");
#endif

    std::shared_ptr<GLFWDisplay> window;
    std::shared_ptr<HeadlessFrontend> console;
    std::shared_ptr<Display> display;
    std::shared_ptr<Keyboard> keyboard;
    if (headless) {
        int fd = inputScript.empty() ? STDIN_FILENO : open(inputScript.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) throw std::runtime_error("Failed to open input script: " + inputScript);
        try {
            console = std::make_shared<HeadlessFrontend>("gui/ui.json", fd);
        } catch (...) {
            if (!inputScript.empty()) close(fd);
            throw;
        }
        if (!inputScript.empty()) close(fd);
        display = console;
        keyboard = console;
        LogInfo("Headless mode: commands from %s", inputScript.empty() ? "stdin" : inputScript.c_str());
    } else {
        window = std::make_shared<GLFWDisplay>();
        display = window;
        keyboard = window;
    }

    std::signal(SIGTERM, StopSigHandler);
    std::signal(SIGINT, StopSigHandler);

    // Create BlackFin CPU. A watchdog reset destroys it and makes a new one (makeCpu again), as a real reset
    // starts over from the boot ROM; the frontends, the NAND and the USB/IP server stay.
    std::unique_ptr<BlackFinCpu> cpuOwner;
    BlackFinCpu* cpu = nullptr;
    auto makeCpu = [&]() {
        cpuOwner = std::make_unique<BlackFinCpu>(deterministic);
        cpu = cpuOwner.get();
        cpu->SetTwiSpeedup(twiSpeedup);
        cpu->SetMdmaSpeedup(mdmaSpeedup);
        // An LDR runs with the host looking at the PC after every block
        // (LdrExecutionThread).
        if (chainDry && noChain) throw std::runtime_error("--profile-chain-dry checks chaining: not with --no-chain");
        cpu->SetChaining(!noChain && positional.size() < 2, chainDry);
        if (noLoopBlocks) cpu->SetLoopBlocks(false);
        if (noIdleWait) cpu->SetIdleWaits(false);
        cpu->SetSteadySkip(steadySkip);
        cpu->SetTiering(static_cast<uint32_t>(tierThreshold), static_cast<uint32_t>(tierSizeRef));
        cpu->SetLightTier1(lightTier1);
        cpu->SetTierBackground(tierBackground, static_cast<uint64_t>(tierPublishAfter), static_cast<int>(jitCpu));
        cpu->SetCopyPatch(copyPatch);
        cpu->SetCopyPatchHints(copyPatchHints);
        if (noDmaSettle) cpu->SetDmaSettling(false);
        if (rtcEpoch) cpu->SetRtcEpoch(std::chrono::system_clock::time_point(std::chrono::seconds(*rtcEpoch)));
    };
    makeCpu();
#ifdef ENABLE_PROFILING
    std::unique_ptr<Profiler> profiler;
    if (anyProfiling) {
        ProfilerOptions profile;
        profile.nandPath = positional[0];
        profile.buildInfo = BuildInfoJson();
        profile.marksPath = profileMarks;
        profile.tracePath = profileTrace;
        profile.censusPrefix = profileCensus;
        profile.perfFifo = perfFifo;
        profile.perfWindow = perfWindow;
        profile.until = profileUntil;
        profile.jitdump = perfJitdump;
        profile.counters = profileCounters;
        profile.probes = profileProbes;
        profile.markDigest = markDigest;
        if (!profileDumpDir.empty()) profile.dumpIrDir = profileDumpDir;
        for (size_t at = 0; !profileDumpIr.empty() && at <= profileDumpIr.size();) {
            const size_t comma = std::min(profileDumpIr.find(',', at), profileDumpIr.size());
            const std::string text = profileDumpIr.substr(at, comma - at);
            size_t used = 0;
            unsigned long long pc = 0;
            try { pc = std::stoull(text, &used, 16); } catch (const std::exception&) { used = 0; }
            if (used == 0 || used != text.size() || pc > 0xFFFFFFFFull)
                throw std::runtime_error("Invalid --profile-dump-ir PC: " + text);
            profile.dumpIrPcs.push_back(static_cast<uint32_t>(pc));
            at = comma + 1;
        }
        profile.checksumPath = profileChecksum;
        auto count = [](const std::string& text, const char* option, bool zero) {
            size_t used = 0;
            unsigned long long value = 0;
            bool parsed = text.find('-') == std::string::npos;
            try { if (parsed) value = std::stoull(text, &used); } catch (const std::exception&) { parsed = false; }
            if (!parsed || used != text.size() || (!zero && value == 0))
                throw std::runtime_error(std::string("Invalid ") + option + ": " + text);
            return static_cast<uint64_t>(value);
        };
        if (!profileChecksumEvery.empty())
            profile.checksumEvery = count(profileChecksumEvery, "--profile-checksum-every", false);
        if (!profileChecksumFrom.empty())
            profile.checksumFrom = count(profileChecksumFrom, "--profile-checksum-from", true);
        if (!profileEndCycles.empty())
            profile.endCycles = count(profileEndCycles, "--profile-end-cycles", false);
        if (!profileTraceEvents.empty()) {
            if (profileTrace.empty()) throw std::runtime_error("--profile-trace-events sizes --profile-trace: add it");
            profile.traceEvents = count(profileTraceEvents, "--profile-trace-events", false);
        }
        profiler = std::make_unique<Profiler>(profile, *cpu, [] { cpuShouldStop.store(true); });
        cpu->SetProfiler(profiler.get());
        // The verified-frame mark sees exactly the pixels the frontend gets.
        if (profiler->WantsFrames()) display = std::make_shared<FrameTap>(display, *profiler);
    }
#endif
    // SPORT still drains DMA and advances its sample accounting without a
    // host sink. Avoid opening ALSA/PulseAudio devices on headless hosts.
    std::shared_ptr<MiniaudioOutput> audioOutput;
    if (!headless) audioOutput = std::make_shared<MiniaudioOutput>(audioBufferMs);
    // OP1EMU_TEST_INPUT_HZ: a square wave at a quarter of full scale on SPORT0's input (sampler tests), instead
    // of silence.
    std::shared_ptr<TestAudioInput> audioInput;
    if (const char* setting = std::getenv("OP1EMU_TEST_INPUT_HZ")) {
        const unsigned kRate = SPORT::DefaultRateHz();
        char* end = nullptr;
        const long hz = std::strtol(setting, &end, 10);
        if (end == setting || *end || hz < 1 || hz > kRate / 2)
            throw std::invalid_argument("OP1EMU_TEST_INPUT_HZ must be 1..half the SPORT frame rate");
        audioInput = std::make_shared<TestAudioInput>(static_cast<unsigned>(hz), kRate);
        LogInfo("Diagnostic SPORT0 input: %ld Hz square wave, quarter scale", hz);
    }
    auto attachFrontends = [&]() {
        if (audioInput) cpu->AttachAudioInput(audioInput);
        cpu->AttachDisplay(display);
        cpu->AttachKeyboard(keyboard);
        if (audioOutput) cpu->AttachAudioOutput(audioOutput);
        cpu->SetBootMode(0x0D); // Set BMODE to 0b1101, boot from NAND flash with port H
    };
    attachFrontends();
    if (teBoot) BootKeys::HoldComForBoot(*cpu);

    auto loop = uvw::loop::get_default();
    // Its handles are made before the loop runs them, on its thread.
    USBIPServer usbipd(*loop, cpu->GetUSB());
    // OP1EMU_USBIP_PORT: another port than 3240, so that regression runs can each have their own.
    unsigned usbipPort = USBIPServer::kDefaultPort;
    if (const char* value = std::getenv("OP1EMU_USBIP_PORT")) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (!*value || *end || parsed < 1 || parsed > 65535)
            throw std::runtime_error("OP1EMU_USBIP_PORT must be 1..65535");
        usbipPort = static_cast<unsigned>(parsed);
    }
    usbipd.Start(usbipPort);
    std::thread uvloop([loop]() {
        while (true) {
            loop->run();
        }
    });
    // Detached: this thread intentionally runs until process exit. A joinable
    // std::thread destroyed at return would call std::terminate, masking the
    // exit code of a clean quit/SIGTERM shutdown.
    uvloop.detach();

    // Load LDR file
    LDRParser parser;
    if (positional.size() > 1 && !parser.loadFile(positional[1])) {
        std::cerr << "Failed to load LDR file: " << positional[1] << std::endl;
        return 1;
    }

    // Load NAND Flash underlying storage
    auto nandFlash = std::make_shared<MT29F4G08>(*cpu, positional[0], nandSnapshot);
    if (nandSnapshot)
        LogInfo("NAND flash image opened read-only (snapshot mode; guest writes will be discarded)");
    cpu->AttachNandFlash(nandFlash);

    uint64_t sensorVersion = UINT64_MAX; // forces the first headless push
    bool randomAccelSent = false;
    auto pushSensors = [&]() {
        const bool fixedAccel = headless && console->GetSensors().accelFixed;
        const bool sensorsChanged = headless && console->SensorVersion() != sensorVersion;
        // The random feed sends a sample, and so an ADXL interrupt, on every
        // host poll: how many, and at which guest instruction, depends on host
        // load. --deterministic sends one when the feed starts and on each
        // script change, so the guest sees the same samples on every run.
        const bool randomDue = !deterministic || !randomAccelSent || sensorsChanged;
        if (!fixedAccel && randomDue) {
            randomAccelSent = true;
            int16_t ax = static_cast<int16_t>((std::rand() % (540 - 50 + 1)) + 50); // ax in [50, 540]
            int16_t ay = static_cast<int16_t>((std::rand() % (-50 - (-540) + 1)) + (-540)); // ay in [-540, -50]
            int16_t az = static_cast<int16_t>((std::rand() % (874 - 75 + 1)) + 75); // az in [75, 874]
            cpu->SetAcceleration(ax, ay, az); // Placeholder for random accelerometer data
        }
        if (headless) {
            // Script-set sensors are pushed once per change; the CPU keeps the value.
            if (sensorsChanged) {
                const auto& sensors = console->GetSensors();
                if (sensors.accelFixed) cpu->SetAcceleration(sensors.ax, sensors.ay, sensors.az);
                cpu->SetPotentiometerValue(0xFF - sensors.volume);
                sensorVersion = console->SensorVersion();
            }
        } else {
            cpu->SetPotentiometerValue(0xFF - window->GetVolumeValue()); // Update potentiometer (volume) value
        }
    };
    // A script's leading commands (up to its first wait) take effect before
    // the CPU starts, so sensor values reach the guest at the same point of
    // every run instead of whenever the first host poll happens to land.
    if (headless) console->PollEvents();
    pushSensors();

    // Start CPU execution thread
    std::thread cpuThread;
    if (positional.size() > 1) {
        cpuThread = std::thread(LdrExecutionThread, std::ref(*cpu), std::ref(parser));
    } else {
        // If no LDR file is provided, just run the CPU without loading any code
        cpuThread = std::thread(BootExcutionThread, std::ref(*cpu), false);
    }

    // Headless commands that need the machine (see the usage text):
    //   fork-run SCRIPT OUTDIR  pause the CPU between Run() calls and fork(); the child gets the whole machine as it
    //                           is (RAM, devices, queued events, JIT, NAND overlay, copy-on-write), starts a fresh CPU
    //                           thread, runs SCRIPT's commands in OUTDIR (screenshots, audio dump, break dumps land
    //                           there) and exits at its quit; the parent waits, then goes on from the same state, so
    //                           every flow starts from the identical post-boot machine. USB/IP is not available in
    //                           the child (its server thread is not copied by fork). Use --deterministic.
    //   snapshot-mem ADDR LEN PATH [...] reads up to 16 RAM ranges under one CPU pause.
    //   dump-mem ADDR LEN PATH  write guest memory (hex ADDR, LEN) to PATH, with the CPU paused.
    if (headless && deterministic) console->SetLockstep(true);
    if (headless) SPORT::txHook = [&](uint64_t delivered) { console->OnAudioDelivered(delivered); };
    auto pauseCpu = [&]() {
        cpuPause.store(true);
        console->OpenGate();        // a CPU held at a lockstep frame must get back out of Run() to park
        while (!cpuPaused.load() && !cpuShouldStop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    // After a fork-run the parent stays paused, so back-to-back fork-runs all start from the same instruction; the
    // first other command resumes it.
    bool heldByFork = false;
    if (headless) console->SetCommandHook([&](const std::vector<std::string>& args) -> bool {
        if (heldByFork && args[0] != "fork-run" && args[0] != "dump-mem" && args[0] != "snapshot-mem" && args[0] != "barrier" &&
            args[0] != "audio-pos") {
            heldByFork = false;
            cpuPause.store(false);
        }
        if (args[0] == "barrier" && args.size() == 2) {
            std::ofstream out(args[1]);
            out << "ok\n";
            out.close();
            if (!out) throw std::runtime_error("barrier: cannot write " + args[1]);
            return true;
        }
        if (args[0] == "audio-pos" && args.size() == 2) {
            // audio-pos PATH: append "<display frame> <sport0 dump bytes>" read with the CPU stopped, so a film
            // capture can place every screenshot exactly on the audio timeline.
            const bool held = console->GateHeld();
            if (!held) pauseCpu();
            const long bytes = cpu->AudioDumpBytes();
            const auto frame = console->FrameCount();
            const auto delivered = cpu->AudioDelivered();
            if (!held && !heldByFork) cpuPause.store(false);
            std::ofstream out(args[1], std::ios::app);
            out << frame << ' ' << bytes << ' ' << delivered << '\n';
            if (!out) throw std::runtime_error("audio-pos: cannot write " + args[1]);
            return true;
        }
        if (args[0] == "poke" && args.size() == 3) {       // poke ADDR VALUE (hex, a 32-bit word): a probe's request
            const u32 addr = (u32)std::stoul(args[1], nullptr, 16), value = (u32)std::stoul(args[2], nullptr, 16);
            const bool held = console->GateHeld();
            if (!held) pauseCpu();
            cpu->GetEmulator().MemoryWrite(addr, &value, 4);
            if (!held && !heldByFork) cpuPause.store(false);
            LogInfo("poke %#x = %#x", addr, value);
            return true;
        }
        if (args[0] == "snapshot-mem") {
            // Multiple RAM observations at one guest instruction. No MMR reads:
            // their read side effects would violate this command's observer contract.
            if (args.size() < 4 || (args.size() - 1) % 3 || args.size() > 49)
                throw std::runtime_error("snapshot-mem: ADDR LEN PATH triples, 1..16 ranges");
            struct Range { u32 addr, len; std::string path; std::vector<u8> bytes; };
            std::vector<Range> ranges;
            u64 total = 0;
            for (size_t i = 1; i < args.size(); i += 3) {
                size_t ap = 0, lp = 0;
                const auto addr = std::stoull(args[i], &ap, 16), len = std::stoull(args[i + 1], &lp, 16);
                if (ap != args[i].size() || lp != args[i + 1].size())
                    throw std::runtime_error("snapshot-mem: malformed hexadecimal range");
                const auto end = addr + len;
                const bool ram = (addr < 0x08000000 && end <= 0x08000000) ||
                    (addr >= 0xFF800000 && end <= 0xFF808000) ||
                    (addr >= 0xFF900000 && end <= 0xFF908000) ||
                    (addr >= 0xFFA00000 && end <= 0xFFA0C000) ||
                    (addr >= 0xFFA10000 && end <= 0xFFA14000) ||
                    (addr >= 0xFFB00000 && end <= 0xFFB01000);
                if (!len || end < addr || !ram || (total += len) > 0x02000000)
                    throw std::runtime_error("snapshot-mem: RAM ranges only; total <= 32 MiB");
                ranges.push_back({(u32)addr, (u32)len, args[i + 2], std::vector<u8>((size_t)len)});
            }
            const bool held = console->GateHeld();
            if (!held) pauseCpu();
            try {
                for (auto& r : ranges) cpu->GetEmulator().MemoryRead(r.addr, r.bytes.data(), (int)r.len);
            } catch (...) {
                if (!held && !heldByFork) cpuPause.store(false);
                throw;
            }
            if (!held && !heldByFork) cpuPause.store(false);
            for (const auto& r : ranges) {
                std::ofstream out(r.path, std::ios::binary);
                out.write(reinterpret_cast<const char*>(r.bytes.data()), r.len);
                if (!out) throw std::runtime_error("snapshot-mem: cannot write " + r.path);
            }
            LogInfo("snapshot-mem: %zu RAM ranges, %llu bytes at one guest instruction", ranges.size(), (unsigned long long)total);
            return true;
        }
        if (args[0] == "dump-mem" && args.size() == 4) {
            const u32 addr = (u32)std::stoul(args[1], nullptr, 16), len = (u32)std::stoul(args[2], nullptr, 16);
            if (!len || len > 0x8000000) throw std::runtime_error("dump-mem: LEN must be 1..0x8000000");
            std::vector<u8> buf(len);
            if (console->GateHeld()) {          // lockstep: stopped at a frame already; reading keeps the timing
                cpu->GetEmulator().MemoryRead(addr, buf.data(), (int)len);
            } else {
                pauseCpu();
                cpu->GetEmulator().MemoryRead(addr, buf.data(), (int)len);
                if (!heldByFork) cpuPause.store(false);
            }
            std::ofstream out(args[3], std::ios::binary);
            out.write(reinterpret_cast<const char*>(buf.data()), len);
            if (!out) throw std::runtime_error("dump-mem: cannot write " + args[3]);
            LogInfo("dump-mem %#x..%#x -> %s", addr, addr + len, args[3].c_str());
            return true;
        }
        if (args[0] != "fork-run" || args.size() != 3) return false;
        if (positional.size() > 1) throw std::runtime_error("fork-run needs a NAND boot (no LDR file)");
        if (forkChild) throw std::runtime_error("fork-run inside a fork-run flow");
        if (usbipd.HasClient()) throw std::runtime_error("fork-run: a USB/IP host is attached (detach it first)");
        char script[PATH_MAX];
        if (!realpath(args[1].c_str(), script)) throw std::runtime_error("fork-run: no script " + args[1]);
        mkdir(args[2].c_str(), 0755);
        pauseCpu();
        cpu->SuspendTierWorker();   // its thread would not be in the child
        std::fflush(nullptr);
        const auto t0 = std::chrono::steady_clock::now();
        const pid_t pid = fork();
        if (pid < 0) { cpu->ResumeTierWorker(); cpuPause.store(false); throw std::runtime_error("fork-run: fork failed"); }
        if (pid == 0) {
            forkChild = true;
            if (chdir(args[2].c_str()) != 0) _exit(70);
            if (std::getenv("OP1EMU_AUDIO_DUMP")) setenv("OP1EMU_AUDIO_DUMP", ".", 1);
            if (std::getenv("OP1EMU_BREAK_DIR")) { mkdir("brk", 0755); setenv("OP1EMU_BREAK_DIR", "brk", 1); }
            cpu->ReopenAudioDumps();
            cpu->ResumeTierWorker();
            const int fd = open(script, O_RDONLY | O_NONBLOCK);
            if (fd < 0) _exit(71);
            console->ReplaceInput(fd);
            close(fd);
            // The parent's CPU thread is not in this process: keep its handle where nothing destroys it.
            alignas(std::thread) static unsigned char stale[sizeof(std::thread)];
            new (stale) std::thread(std::move(cpuThread));
            heldByFork = false;
            cpuPaused.store(false);
            cpuPause.store(false);
            cpuThread = std::thread(BootExcutionThread, std::ref(*cpu), true);
            LogInfo("fork-run: flow %s in %s (pid %d)", script, args[2].c_str(), (int)getpid());
            return true;
        }
        cpu->ResumeTierWorker();
        int status = 0;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const int code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        LogInfo("fork-run: %s finished with %d in %.1f s", args[2].c_str(), code, secs);
        const auto statusPath = args[2] + "/fork-run.status";
        { std::ofstream out(statusPath + ".tmp"); out << code << ' ' << secs << '\n'; }
        if (std::rename((statusPath + ".tmp").c_str(), statusPath.c_str()) != 0)
            throw std::runtime_error("fork-run: cannot publish exit status");
        heldByFork = true;          // stays paused until a command other than fork-run / dump-mem
        return true;
    });

    // Main thread handles GLFW display or headless input. cpuShouldStop is in
    // the condition so SIGTERM/SIGINT and CPU thread exit end the loop too.
    int result = 0;
    try {
        while (!cpuShouldStop.load() && !(headless ? console->ShouldClose() : window->ShouldClose())) {
            if (cpu->ResetRequested()) {
                // Watchdog reset: drop the machine and boot a new one from the boot ROM.
#ifdef ENABLE_PROFILING
                if (profiler) throw std::runtime_error("System reset: not with profiling (the profiler watches one machine)");
#endif
                cpuThread.join();
                usbipd.Rebind(nullptr);
                cpuOwner.reset();
                cpu = nullptr;
                makeCpu();
                attachFrontends();
                nandFlash->Rebind(*cpu);
                cpu->AttachNandFlash(nandFlash);
                usbipd.Rebind(&cpu->GetUSB());
                sensorVersion = UINT64_MAX;
                pushSensors();
                cpuThread = std::thread(BootExcutionThread, std::ref(*cpu), false);
                LogInfo("System reset: booting again");
            }
            if (headless) console->PollEvents();
            else window->PollEvents();
            pushSensors();

            // Preserve the existing host frame/input cadence, without a GL swap.
            std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
        }
    } catch (const std::exception& e) {
        LogError("Headless input: %s", e.what());
        result = 1;
    }

    // Signal CPU thread to stop and wait for it
    LogInfo("Stopping CPU thread...");
    cpuShouldStop.store(true);
    if (headless) { console->SetLockstep(false); console->OpenGate(); }
    cpuThread.join();
    // OP1_DUMP_MEM=ADDR:LEN:PATH (hex ADDR/LEN) writes any guest range at exit.
    if (const char* spec = std::getenv("OP1_DUMP_MEM")) {
        char* end = nullptr;
        const u32 addr = (u32)strtoul(spec, &end, 16);
        const u32 len = (end && *end == ':') ? (u32)strtoul(end + 1, &end, 16) : 0;
        if (len && end && *end == ':') {
            std::vector<u8> buf(len);
            cpu->GetEmulator().MemoryRead(addr, buf.data(), (int)len);
            std::ofstream(end + 1, std::ios::binary).write(reinterpret_cast<const char*>(buf.data()), len);
        }
    }
    if (cpu->Failed()) {
        LogError("Emulation stopped: CPU execution failed");
        result = 1;
    }
    if (forkChild) {               // the USB/IP and uv threads of the parent don't exist here: no destructors
        std::fflush(nullptr);
        _exit(result);
    }
#ifdef ENABLE_PROFILING
    if (profiler && !profiler->Finish()) {
        LogError("Profiling output is incomplete");
        result = 1;
    }
#endif

    return result;
}

int main(int argc, char* argv[]) {
    try {
        return RunEmulator(argc, argv);
    } catch (const std::exception& e) {
        LogError("%s", e.what());
        return 1;
    }
}
