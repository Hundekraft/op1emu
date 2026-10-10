#pragma once

#include "peripheral/audio_output.h"
#include <atomic>
#include <cstdint>

class MiniaudioOutput : public AudioOutput {
public:
    MiniaudioOutput();
    ~MiniaudioOutput() override;

    void WriteSamples(const void* data, size_t samples, int channels, int bitsPerSample) override;

private:
    static void DataCallback(void* pDevice, void* pOutput, const void* pInput, unsigned int frameCount);

    void* device_;      // ma_device*
    void* ringBuffer_;  // ma_pcm_rb*
    std::atomic<bool> active_{false};

    // Playback waits until the ring holds kPrefillFrames (at start and after each underrun), so guest-time
    // jitter is absorbed instead of heard as clicks. Counters are logged from WriteSamples.
    std::atomic<bool> playing_{false};
    std::atomic<uint64_t> underruns_{0}, dropped_{0};
    uint64_t written_ = 0, nextReport_ = 0;
};
