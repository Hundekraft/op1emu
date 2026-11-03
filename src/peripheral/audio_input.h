#pragma once
#include <cstddef>

class AudioInput {
public:
    virtual ~AudioInput() = default;
    // CPU-thread callback. One frame contains channels words, each stored in
    // 16 bits (bitsPerSample <= 16) or 32 bits, matching SPORT DMA storage.
    virtual size_t ReadSamples(void* data, size_t frames, int channels, int bitsPerSample) = 0;
};
