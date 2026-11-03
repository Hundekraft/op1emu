#pragma once

#include "peripheral/display.h"
#include "peripheral/keyboard.h"
#include "encoder_queue.h"
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Host frontend only: nothing is drawn, but the PPI/DMA still runs and the
// last complete frame is kept so scripts can wait for and capture it.
// Like GLFWDisplay, frame and key callbacks enqueue work on the CPU thread.
class HeadlessFrontend : public Display, public Keyboard {
public:
    using Clock = std::chrono::steady_clock;
    HeadlessFrontend(const std::string& configPath, int inputFd);
    ~HeadlessFrontend() override;
    HeadlessFrontend(const HeadlessFrontend&) = delete;
    HeadlessFrontend& operator=(const HeadlessFrontend&) = delete;

    // Called on the CPU thread. Re-initializing with the same geometry keeps
    // the pixels: firmware rewrites PPI_CONTROL without starting a new frame.
    void Initialize(int width, int height) override;
    void UpdateRowBuffer(int x, int y, const void* data, int length) override;
    void PollEvents(Clock::time_point now = Clock::now());
    bool ShouldClose() const { return shouldClose_; }

    // Every command goes to this hook first (main.cpp: fork-run, dump-mem, and resuming the CPU a fork-run left
    // paused); true = handled.
    // Script command "trace NAME": starts a trace phase (OP1EMU_TRACE_DIR); the host routes it to the CPU.
    void SetTraceHook(const std::function<void(const std::string&)>& hook) { traceHook_ = hook; }
    void SetCommandHook(std::function<bool(const std::vector<std::string>&)> hook) { commandHook_ = std::move(hook); }
    // Read commands from fd from now on (a fork-run child switches to its flow script); pending input is dropped.
    void ReplaceInput(int fd);

    // Lockstep (--deterministic): when the frame a wait-frames / hold / turn waits for completes, the CPU thread
    // stops there until the main thread has run the commands that follow, so input lands at the same guest
    // instant on every run. OpenGate lets a stopped CPU go (shutdown, fork-run's pause).
    void SetLockstep(bool on) { lockstep_ = on; }
    void OpenGate();
    bool GateHeld() const { return gateHeld_.load() || audioHeld_.load(); }   // CPU stopped at a frame or audio gate
    // wait-audio N: the CPU thread stops once SPORT0 has delivered N TX frames (absolute, since DMA start), so
    // input lands on exact audio positions whatever the screen's frame rate. Called from SPORT::txHook.
    void OnAudioDelivered(uint64_t delivered);
    uint64_t AudioDelivered() const { return delivered_.load(); }

    // Number of complete frames seen so far (a frame starts at pixel (0,0)).
    uint64_t FrameCount() const { return frameCount_.load(std::memory_order_acquire); }

    // Sensor state set by scripts. Main-thread only; the host loop pushes it
    // to the CPU when SensorVersion() changes.
    struct Sensors {
        bool accelFixed = false;
        int16_t ax = 0, ay = 0, az = 0;
        uint8_t volume = 128;
    };
    const Sensors& GetSensors() const { return sensors_; }
    uint64_t SensorVersion() const { return sensorVersion_; }

private:
    void WriteScreenshot(const std::string& path) const;
    bool ReadLine(std::string& line);
    void Execute(const std::string& line, Clock::time_point now);
    using Pin = std::pair<int, int>;
    std::map<std::string, Pin> keys_;
    std::function<bool(const std::vector<std::string>&)> commandHook_;
    std::function<void(const std::string&)> traceHook_;
    int inputFd_ = -1;
    bool eof_ = false;
    bool shouldClose_ = false;
    std::string pending_;
    Clock::time_point resumeAt_{};
    // `at MS`: later key commands are for that guest time (see Keyboard::OnKeyAt).
    std::optional<long> guestAtMs_;
    std::optional<Pin> releaseAtResume_;
    std::optional<Pin> releaseAtFrame_;   // hold KEY FRAMES
    // Encoders: quadrature pins per name, current phase, and pending steps (+cw / -ccw) of a "turn".
    struct Encoder { Pin cw, ccw; int phase = 0; };
    std::map<std::string, Encoder> encoders_;
    Encoder* turning_ = nullptr;
    // "wheel": the GUI's encoder queue, paced in host time like the window (tests the GUI's pacing).
    EncoderQueue wheelQueue_;
    std::map<std::string, size_t> wheelIndex_;
    int turnSteps_ = 0, turnFrames_ = 2;
    uint64_t turnNextFrame_ = 0;

    Sensors sensors_;
    uint64_t sensorVersion_ = 0;

    std::optional<uint64_t> frameTarget_;
    Clock::time_point frameDeadline_{};

    // Frame state. Written by the CPU thread, read by the main thread.
    mutable std::mutex frameMutex_;
    int width_ = 0, height_ = 0;
    std::vector<uint16_t> work_, latest_;
    const bool flipLines_ = PanelLinesReversed();   // OP1EMU_PANEL=b: store lines in screen order
    std::vector<uint8_t> seen_;
    size_t remaining_ = 0;
    bool started_ = false;
    std::atomic<uint64_t> frameCount_{0};

    // Lockstep gate: the CPU thread waits at frame gateFrame_ (0: none) until the main thread opens it.
    bool lockstep_ = false;
    std::atomic<uint64_t> gateFrame_{0};
    std::atomic<bool> gateHeld_{false};
    std::mutex gateMutex_;
    std::condition_variable gateCv_;
    void ArmGate(uint64_t frame);
    void ArmAudioGate(uint64_t frames);
    std::optional<uint64_t> audioTarget_;
    uint64_t audioBase_ = 0;   // audio-anchor: wait-audio @N counts from here
    std::chrono::steady_clock::time_point audioDeadline_;
    std::atomic<uint64_t> audioGate_{0}, delivered_{0};
    std::atomic<bool> audioHeld_{false};
    void WaitAtGate();
};
