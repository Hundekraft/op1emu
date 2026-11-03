#pragma once
#include "peripheral/audio_input.h"
#include <cstdint>
#include <stdexcept>

// Explicit diagnostic source, not a microphone backend. Integer phase keeps
// the signal independent of host timing and callback chunk size.
class TestAudioInput final : public AudioInput {
public:
    TestAudioInput(unsigned hz, unsigned rate) : hz_(hz), rate_(rate) {
        if (!hz || hz > rate / 2) throw std::invalid_argument("Test input frequency outside 1..rate/2");
    }
    size_t ReadSamples(void* data, size_t frames, int channels, int bits) override {
        if (channels < 1 || channels > 4 || bits < 3 || bits > 32 || (frames && !data))
            throw std::invalid_argument("Unsupported test input PCM format");
        auto* bytes = static_cast<unsigned char*>(data);
        const unsigned width = bits <= 16 ? 2 : 4;
        const int32_t amplitude = static_cast<int32_t>(uint32_t{1} << (bits - 3));
        for (size_t frame = 0; frame < frames; ++frame) {
            const uint32_t sample = static_cast<uint32_t>(phase_ < rate_ / 2 ? amplitude : -amplitude);
            for (int channel = 0; channel < channels; ++channel)
                for (unsigned byte = 0; byte < width; ++byte) *bytes++ = sample >> (byte * 8);
            phase_ = (uint64_t{phase_} + hz_) % rate_;
        }
        return frames;
    }
private:
    unsigned hz_, rate_, phase_ = 0;
};
