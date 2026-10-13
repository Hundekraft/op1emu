#include "test_audio_input.h"
#include <array>
#include <cstdlib>
#include <iostream>
#include <vector>

static void require(bool result) {
    if (!result) { std::cerr << "Audio input assertion failed\n"; std::exit(1); }
}
int main() {
    for (int bits : {16, 24, 32}) for (int channels : {1, 2, 4}) {
        unsigned width = bits <= 16 ? 2 : 4;
        size_t stride = width * channels;
        std::vector<unsigned char> whole(100 * stride + 4, 0xa5), split = whole;
        TestAudioInput a(441, 22050), b(441, 22050);
        require(a.ReadSamples(whole.data(), 100, channels, bits) == 100);
        require(b.ReadSamples(split.data(), 13, channels, bits) == 13);
        require(b.ReadSamples(split.data() + 13 * stride, 87, channels, bits) == 87);
        require(whole == split);
        for (size_t frame = 0; frame < 100; frame++) {
            int32_t amplitude = int32_t{1} << (bits - 3);
            uint32_t expected = (frame % 50 < 25) ? amplitude : -amplitude;
            for (int channel = 0; channel < channels; channel++)
                for (unsigned byte = 0; byte < width; byte++)
                    require(whole[frame * stride + channel * width + byte] == ((expected >> (8 * byte)) & 255));
        }
        for (size_t i = 100 * stride; i < whole.size(); i++) require(whole[i] == 0xa5);
    }
    TestAudioInput input(441, 22050);
    require(input.ReadSamples(nullptr, 0, 2, 32) == 0);
    for (unsigned hz : {0u, 11026u}) {
        bool rejected = false;
        try { TestAudioInput invalid(hz, 22050); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected);
    }
    std::array<unsigned char, 16> buffer{};
    bool rejected = false;
    try { input.ReadSamples(buffer.data(), 1, 5, 32); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
    std::cout << "Diagnostic input PCM, bounds and chunk invariance passed\n";
}
