#pragma once

#include "io.h"
#include "dma.h"
#include <vector>
#include <queue>
#include <optional>
#include <functional>
#include <cstdint>
#include <cstdio>

class TimeSource;

class SPORT : public RegisterDevice, public DMABus {
public:
    static constexpr uint32_t kSampleRateHz = 44100;
    using AudioOutputCallback = std::function<void(const void* data, size_t samples, int channels, int bitsPerSample)>;
    using AudioInputCallback = std::function<size_t(void* data, size_t samples, int channels, int bitsPerSample)>;

    SPORT(u32 baseAddr, int sportNum, const TimeSource& time);

    // DMABus interface
    u32 DMARead(int x, int y, void* dest, u32 length) override;
    u32 DMAWrite(int x, int y, const void* source, u32 length) override;
    u32 DMAWriteCapacity(u32 length) override;
    u32 DMAReadAvailable(u32 length) override;
    struct DueMemo { bool valid = false; uint64_t start = 0, delivered = 0; uint32_t rate = 0; unsigned slow = 0; uint64_t dueNs = 0; };
    DueMemo txDue_, rxDue_;
    bool SampleDue(DueMemo& m, uint64_t startNs, uint64_t delivered);

    // Audio callbacks
    void SetAudioOutputCallback(AudioOutputCallback callback) { audioOutputCallback = callback; }
    void SetAudioInputCallback(AudioInputCallback callback) { audioInputCallback = callback; }

protected:
    void SetTransmitEnable();
    void SetReceiveEnable();

    int sportNumber;

    bool transmitEnabled = false;
    u8 transmitDataFormat = 0;
    bool transmitOrderLsbFirst = false;
    u8 transmitWordLength = 0;
    bool transmitSecondaryEnabled = false;
    bool transmitStereoFrameSync = false;
    bool transmitRightStereoOrderFirst = false;
    bool transmitOverflow = false;
    bool transmitUnderflow = false;

    bool receiveEnabled = false;
    u8 receiveDataFormat = 0;
    bool receiveOrderLsbFirst = false;
    u8 receiveWordLength = 0;
    bool receiveSecondaryEnabled = false;
    bool receiveStereoFrameSync = false;
    bool receiveRightStereoOrderFirst = false;
    bool receiveOverflow = false;
    bool receiveUnderflow = false;

    std::optional<u32> transmitHoldRegister;
    std::optional<u32> receiveHoldRegister;
    // FIFO buffers
    std::queue<u32> transmitFifo;
    std::queue<u32> receiveFifo;

    // Audio callbacks
    AudioOutputCallback audioOutputCallback;
    AudioInputCallback audioInputCallback;

    const TimeSource& time_;

    // DMA timing state — TX path
    uint64_t dmaTxStartNs_ = 0;
    uint64_t totalTxSamplesDelivered_ = 0;
    bool dmaTxActive_ = false;

    // DMA timing state — RX path
    uint64_t dmaRxStartNs_ = 0;
    uint64_t totalRxSamplesDelivered_ = 0;
    bool dmaRxActive_ = false;

    // Frame rate in Hz. R246 runs SPORT0 from external codec clocks (TCR1 ITCLK/ITFS clear), so the
    // registers don't give it; 44.1 kHz is measured from note pitch (C4 = 169 samples per cycle).
    uint32_t sampleRateHz_ = kSampleRateHz;

    // Raw clock/sync config (debug: logged when DMA starts) and OP1EMU_AUDIO_DUMP raw sample dump.
    u32 tcr1Raw_ = 0, tclkdiv_ = 0, tfsdiv_ = 0, rcr1Raw_ = 0, rclkdiv_ = 0, rfsdiv_ = 0;
    FILE* dump_ = nullptr;
    void LogConfig(const char* dir);
};
