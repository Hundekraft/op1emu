#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "audio_output_miniaudio.h"
#include "utils/log.h"
#include "cpu/sport.h"
#include <cstring>
#include <cstdlib>
#include <algorithm>

static constexpr ma_uint32 SAMPLE_RATE = SPORT::kSampleRateHz;  // the guest's rate; miniaudio converts if needed
static constexpr ma_uint32 CHANNELS = 2;
// Playback starts (and restarts after an underrun) once the ring holds the prefill. In --realtime the guest
// also runs up to OP1EMU_LEAD_MS (default 70) ahead of the host clock, and that audio queues on top, so the
// total buffer (and key-to-sound latency) is about prefill + lead = 100 ms by default. OP1EMU_AUDIO_MS sets
// the prefill (default 30 ms); the ring holds 3x that plus 250 ms of room for the lead.
static ma_uint32 PrefillFrames() {
    static const ma_uint32 frames = [] {
        long ms = 30;
        if (const char* e = std::getenv("OP1EMU_AUDIO_MS")) ms = std::clamp(std::strtol(e, nullptr, 10), 10L, 1000L);
        return (ma_uint32)(SAMPLE_RATE * ms / 1000);
    }();
    return frames;
}

MiniaudioOutput::MiniaudioOutput()
    : device_(nullptr), ringBuffer_(nullptr) {

    auto* rb = new ma_pcm_rb;
    auto* dev = new ma_device;

    // Initialize the ring buffer (float32, stereo)
    ma_result result = ma_pcm_rb_init(ma_format_f32, CHANNELS, PrefillFrames() * 3 + SAMPLE_RATE / 4, nullptr, nullptr, rb);
    if (result != MA_SUCCESS) {
        LogError("Failed to initialize audio ring buffer: %d", result);
        delete rb;
        delete dev;
        return;
    }

    // Configure the playback device
    ma_device_config config = ma_device_config_init(ma_device_type_playback);
    config.playback.format = ma_format_f32;
    config.playback.channels = CHANNELS;
    config.sampleRate = SAMPLE_RATE;
    config.periodSizeInFrames = 256; // ~5.3ms low-latency period
    config.dataCallback = reinterpret_cast<ma_device_data_proc>(DataCallback);
    config.pUserData = this;

    result = ma_device_init(nullptr, &config, dev);
    if (result != MA_SUCCESS) {
        LogError("Failed to initialize audio device: %d", result);
        ma_pcm_rb_uninit(rb);
        delete rb;
        delete dev;
        return;
    }

    result = ma_device_start(dev);
    if (result != MA_SUCCESS) {
        LogError("Failed to start audio device: %d", result);
        ma_device_uninit(dev);
        ma_pcm_rb_uninit(rb);
        delete rb;
        delete dev;
        return;
    }

    device_ = dev;
    ringBuffer_ = rb;
    active_.store(true);
    LogInfo("Audio output initialized: %dHz, %dch, period=%d frames, buffer %u ms",
            SAMPLE_RATE, CHANNELS, config.periodSizeInFrames, PrefillFrames() * 1000 / SAMPLE_RATE);
}

MiniaudioOutput::~MiniaudioOutput() {
    if (device_) {
        ma_device_uninit(static_cast<ma_device*>(device_));
        delete static_cast<ma_device*>(device_);
    }
    if (ringBuffer_) {
        ma_pcm_rb_uninit(static_cast<ma_pcm_rb*>(ringBuffer_));
        delete static_cast<ma_pcm_rb*>(ringBuffer_);
    }
}

void MiniaudioOutput::DataCallback(void* pDevice, void* pOutput, const void* pInput, unsigned int frameCount) {
    (void)pInput;

    ma_device* dev = static_cast<ma_device*>(pDevice);
    auto* self = static_cast<MiniaudioOutput*>(dev->pUserData);
    ma_pcm_rb* rb = static_cast<ma_pcm_rb*>(self->ringBuffer_);
    float* output = static_cast<float*>(pOutput);

    ma_uint32 framesRemaining = frameCount;
    ma_uint32 framesWritten = 0;

    if (!self->playing_.load(std::memory_order_relaxed) && ma_pcm_rb_available_read(rb) >= PrefillFrames())
        self->playing_.store(true, std::memory_order_relaxed);

    while (self->playing_.load(std::memory_order_relaxed) && framesRemaining > 0) {
        void* readBuffer;
        ma_uint32 framesToRead = framesRemaining;
        ma_result result = ma_pcm_rb_acquire_read(rb, &framesToRead, &readBuffer);
        if (result != MA_SUCCESS || framesToRead == 0) {
            break;
        }

        memcpy(output + framesWritten * CHANNELS, readBuffer, framesToRead * CHANNELS * sizeof(float));
        ma_pcm_rb_commit_read(rb, framesToRead);

        framesWritten += framesToRead;
        framesRemaining -= framesToRead;
    }

    // Underrun: fill with silence and wait for the prefill again.
    if (framesRemaining > 0) {
        if (self->playing_.load(std::memory_order_relaxed)) {
            self->underruns_.fetch_add(1, std::memory_order_relaxed);
            self->playing_.store(false, std::memory_order_relaxed);
        }
        memset(output + framesWritten * CHANNELS, 0, framesRemaining * CHANNELS * sizeof(float));
    }
}

void MiniaudioOutput::WriteSamples(const void* data, size_t samples, int channels, int bitsPerSample) {
    if (!active_.load() || samples == 0) {
        return;
    }

    ma_pcm_rb* rb = static_cast<ma_pcm_rb*>(ringBuffer_);

    // Convert input PCM to float32 stereo and push to ring buffer.
    // Process in chunks to avoid large stack allocations.
    constexpr size_t CHUNK_FRAMES = 256;
    float chunk[CHUNK_FRAMES * CHANNELS];

    const uint8_t* src = static_cast<const uint8_t*>(data);
    int wordSize = bitsPerSample > 16 ? 4 : 2;
    size_t srcStride = wordSize * channels;
    // Words wider than 24 bits are full-scale 32-bit (R246 sends 32-bit words with the sample in the top bits).
    auto sample = [&](const uint8_t* p) -> float {
        if (wordSize == 2) { int16_t v; memcpy(&v, p, 2); return v / 32768.0f; }
        int32_t v; memcpy(&v, p, 4);
        if (bitsPerSample > 24) return v / 2147483648.0f;
        return ((v << 8) >> 8) / 8388608.0f;
    };

    size_t framesRemaining = samples;
    while (framesRemaining > 0) {
        size_t framesToProcess = std::min(framesRemaining, CHUNK_FRAMES);

        // First two channels -> stereo. R246 leaves the second one silent (speaker mode?); then play the
        // first on both sides rather than in one ear.
        bool rightSilent = channels >= 2;
        for (size_t i = 0; i < framesToProcess && rightSilent; i++) {
            uint32_t w = 0;
            memcpy(&w, src + i * srcStride + wordSize, wordSize);
            rightSilent = w == 0;
        }
        for (size_t i = 0; i < framesToProcess; i++) {
            const float left = sample(src + i * srcStride);
            const float right = (channels >= 2 && !rightSilent) ? sample(src + i * srcStride + wordSize) : left;
            chunk[i * CHANNELS] = left;
            chunk[i * CHANNELS + 1] = right;
        }

        // Push to ring buffer
        ma_uint32 framesToWrite = static_cast<ma_uint32>(framesToProcess);
        void* writeBuffer;
        ma_result result = ma_pcm_rb_acquire_write(rb, &framesToWrite, &writeBuffer);
        if (result != MA_SUCCESS) framesToWrite = 0;
        if (framesToWrite > 0) {
            memcpy(writeBuffer, chunk, framesToWrite * CHANNELS * sizeof(float));
            ma_pcm_rb_commit_write(rb, framesToWrite);
        }
        // If ring buffer is full, we drop samples (guest running ahead of the audio device)
        dropped_.fetch_add(framesToProcess - framesToWrite, std::memory_order_relaxed);

        framesRemaining -= framesToProcess;
        src += framesToProcess * srcStride;
    }

    written_ += samples;
    if (written_ >= nextReport_) {
        if (nextReport_) {
            const uint64_t u = underruns_.exchange(0), d = dropped_.exchange(0);
            if (u || d) LogInfo("Audio: %llu underruns, %llu frames dropped in the last 10 s", (unsigned long long)u, (unsigned long long)d);
        }
        nextReport_ = written_ + 10 * SAMPLE_RATE;
    }
}
