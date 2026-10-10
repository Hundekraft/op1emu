#include "sport.h"
#include "emu.h"
#include "time_source.h"
#include "utils/log.h"
#include <cstdlib>
#include <cstring>
#include <string>

enum DataFormat {
    NORMAL = 0,
    COMPANDING_U_LAW = 2,
    COMPANDING_A_LAW = 3,
};

constexpr std::size_t FIFO_SIZE = 8; // 8x16-bit words or 4x32-bit words

SPORT::SPORT(u32 baseAddr, int sportNum, const TimeSource& time)
    : RegisterDevice("SPORT" + std::to_string(sportNum), baseAddr, 0x60), sportNumber(sportNum), time_(time) {

    REG32(SPORT_TCR1, 0x00);
    FIELD(SPORT_TCR1, TSPEN, 0, 1, R(transmitEnabled), W(transmitEnabled));
    FIELD(SPORT_TCR1, TDTYPE, 2, 2, R(transmitDataFormat), W(transmitDataFormat));
    FIELD(SPORT_TCR1, TLSBIT, 4, 1, R(transmitOrderLsbFirst), W(transmitOrderLsbFirst));
    SPORT_TCR1.writeCallback = [this](u32 value) {
        tcr1Raw_ = value;
        SetTransmitEnable();
    };

    REG32(SPORT_TCR2, 0x04);
    FIELD(SPORT_TCR2, SLEN, 0, 5, R(transmitWordLength - 1), [this](u32 v) {
        transmitWordLength = v + 1;
    });
    FIELD(SPORT_TCR2, TSFSE, 8, 1, R(transmitStereoFrameSync), W(transmitStereoFrameSync));
    FIELD(SPORT_TCR2, TXSE, 9, 1, R(transmitSecondaryEnabled), W(transmitSecondaryEnabled));
    FIELD(SPORT_TCR2, TRFST, 10, 1, R(transmitRightStereoOrderFirst), W(transmitRightStereoOrderFirst));

    REG32(SPORT_TCLKDIV, 0x08);
    FIELD(SPORT_TCLKDIV, VAL, 0, 16, R(tclkdiv_), W(tclkdiv_));
    REG32(SPORT_TFSDIV, 0x0C);
    FIELD(SPORT_TFSDIV, VAL, 0, 16, R(tfsdiv_), W(tfsdiv_));
    REG32(SPORT_RCLKDIV, 0x28);
    FIELD(SPORT_RCLKDIV, VAL, 0, 16, R(rclkdiv_), W(rclkdiv_));
    REG32(SPORT_RFSDIV, 0x2C);
    FIELD(SPORT_RFSDIV, VAL, 0, 16, R(rfsdiv_), W(rfsdiv_));

    REG32(SPORT_TX, 0x10);
    SPORT_TX.writeCallback = [this](u32 v) {
        std::size_t fifoSize = transmitWordLength > 16 ? FIFO_SIZE/2 : FIFO_SIZE;
        if (transmitFifo.size() >= fifoSize) {
            transmitOverflow = true;
        } else {
            transmitFifo.push(v);
        }
    };

    REG32(RX, 0x18);
    FIELD(RX, VAL, 0, 32, [this]() {
        if (receiveFifo.empty()) {
            receiveUnderflow = true;
            return 0u;
        } else {
            u32 value = receiveFifo.front();
            receiveFifo.pop();
            return value;
        }
    }, N());

    REG32(SPORT_RCR1, 0x20);
    FIELD(SPORT_RCR1, RSPEN, 0, 1, R(receiveEnabled), W(receiveEnabled));
    FIELD(SPORT_RCR1, RDTYPE, 2, 2, R(receiveDataFormat), W(receiveDataFormat));
    FIELD(SPORT_RCR1, RLSBIT, 4, 1, R(receiveOrderLsbFirst), W(receiveOrderLsbFirst));
    SPORT_RCR1.writeCallback = [this](u32 value) {
        rcr1Raw_ = value;
        SetReceiveEnable();
    };

    REG32(SPORT_RCR2, 0x24);
    FIELD(SPORT_RCR2, SLEN, 0, 5, R(receiveWordLength - 1), [this](u32 v) {
        receiveWordLength = v + 1;
    });
    FIELD(SPORT_RCR2, RSFSE, 8, 1, R(receiveStereoFrameSync), W(receiveStereoFrameSync));
    FIELD(SPORT_RCR2, RXSE, 9, 1, R(receiveSecondaryEnabled), W(receiveSecondaryEnabled));
    FIELD(SPORT_RCR2, RRFST, 10, 1, R(receiveRightStereoOrderFirst), W(receiveRightStereoOrderFirst));

    REG32(SPORT_STAT, 0x30);
    FIELD(SPORT_STAT, RXNE, 0, 1, R(!receiveFifo.empty()), N());
    FIELD(SPORT_STAT, RUVF, 1, 1, R(receiveUnderflow), W1C(receiveUnderflow));
    FIELD(SPORT_STAT, ROVF, 2, 1, R(receiveOverflow), W1C(receiveOverflow));
    FIELD(SPORT_STAT, TXF, 3, 1, R(transmitFifo.size() >= (transmitWordLength > 16 ? FIFO_SIZE/2 : FIFO_SIZE)), N());
    FIELD(SPORT_STAT, TUVF, 4, 1, R(transmitUnderflow), W1C(transmitUnderflow));
    FIELD(SPORT_STAT, TOVF, 5, 1, R(transmitOverflow), W1C(transmitOverflow));
    FIELD(SPORT_STAT, TXHRE, 6, 1, R(transmitHoldRegister.has_value()), N());
}

void SPORT::SetTransmitEnable() {
    if (!transmitEnabled) {
        transmitOverflow = false;
        transmitUnderflow = false;
        transmitHoldRegister.reset();
        dmaTxActive_ = false;
        totalTxSamplesDelivered_ = 0;
    }
}

void SPORT::SetReceiveEnable() {
    if (!receiveEnabled) {
        receiveOverflow = false;
        receiveUnderflow = false;
        receiveHoldRegister.reset();
        dmaRxActive_ = false;
        totalRxSamplesDelivered_ = 0;
    }
}

// Side-effect-free versions of the checks at the top of DMARead/DMAWrite: 0 exactly when those would
// return 0 without changing state. Before the first transfer they return length, so the real call still
// starts the timing.
u32 SPORT::DMAReadAvailable(u32 length)
{
    if (!receiveEnabled) return 0;
    const u32 frameSize = (receiveWordLength > 16 ? 4 : 2) * (receiveStereoFrameSync ? 2 : 1) * (receiveSecondaryEnabled ? 2 : 1);
    if (length / frameSize == 0) return 0;
    if (!dmaRxActive_) return length;
    return SampleDue(rxDue_, dmaRxStartNs_, totalRxSamplesDelivered_) ? length : 0;
}

u32 SPORT::DMAWriteCapacity(u32 length)
{
    if (!transmitEnabled) return 0;
    const u32 frameSize = (transmitWordLength > 16 ? 4 : 2) * (transmitStereoFrameSync ? 2 : 1) * (transmitSecondaryEnabled ? 2 : 1);
    if (length / frameSize == 0) return 0;
    if (!dmaTxActive_) return length;
    return SampleDue(txDue_, dmaTxStartNs_, totalTxSamplesDelivered_) ? length : 0;
}

// True when sample number delivered+1 is due: ((now - start) * rate / 1e9) / slowdown > delivered. Between
// samples the answer is memoized as the guest time the next one becomes due, for the same inputs.
bool SPORT::SampleDue(DueMemo& m, uint64_t startNs, uint64_t delivered)
{
    const uint64_t now = time_.Nanoseconds();
    const unsigned slow = time_.Slowdown();
    if (m.valid && m.start == startNs && m.delivered == delivered && m.rate == sampleRateHz_ && m.slow == slow)
        return now >= m.dueNs;
    const uint64_t due = ((now - startNs) * sampleRateHz_ / 1'000'000'000ULL) / slow;
    if (due > delivered) {
        m.valid = false;
        return true;
    }
    // Smallest elapsed E with floor(E * rate / 1e9) >= (delivered + 1) * slow.
    const uint64_t need = (delivered + 1) * slow;
    const uint64_t e = (need * 1'000'000'000ULL + sampleRateHz_ - 1) / sampleRateHz_;
    m = {true, startNs, delivered, sampleRateHz_, slow, startNs + e};
    return false;
}

u32 SPORT::DMARead(int x, int y, void* dest, u32 length)
{
    if (!receiveEnabled) {
        return 0;
    }

    int wordSize = receiveWordLength > 16 ? 4 : 2;
    int channels = (receiveStereoFrameSync ? 2 : 1) * (receiveSecondaryEnabled ? 2 : 1);
    int bitsPerSample = receiveWordLength;
    int frameSize = wordSize * channels;
    size_t requestedSamples = length / frameSize;
    if (requestedSamples == 0) return 0;

    // Start timing on first DMA call after SPORT enable
    if (!dmaRxActive_) {
        dmaRxStartNs_ = time_.Nanoseconds();
        totalRxSamplesDelivered_ = 0;
        dmaRxActive_ = true;
        LogConfig("RX");
    }

    // How many samples are due based on elapsed time?
    uint64_t elapsedNs = time_.Nanoseconds() - dmaRxStartNs_;
    uint64_t totalDue = (elapsedNs * sampleRateHz_) / 1'000'000'000ULL;
    // FIXME: wall time runs the emulator slower than real time, so it divides
    // the rate by TimeSource::kWallSlowdown; deterministic guest time does not.
    totalDue = totalDue / time_.Slowdown();
    uint64_t available = (totalDue > totalRxSamplesDelivered_)
                         ? (totalDue - totalRxSamplesDelivered_) : 0;

    if (available > requestedSamples) available = requestedSamples;
    if (available == 0) return 0;  // Not time yet — DMA retries next cycle

    if (audioInputCallback) {
        size_t read = audioInputCallback(dest, available, channels, bitsPerSample);
        totalRxSamplesDelivered_ += read;
        return static_cast<u32>(read * frameSize);
    }

    // No input attached: deliver silence. The DMA copies whatever is in dest into guest memory, and its
    // buffer is reused stack (it held the last TX transfer, which fed the output back into the mix).
    std::memset(dest, 0, available * frameSize);
    totalRxSamplesDelivered_ += available;
    return static_cast<u32>(available * frameSize);
}

u32 SPORT::DMAWrite(int x, int y, const void* source, u32 length)
{
    if (!transmitEnabled) {
        return 0;
    }

    int wordSize = transmitWordLength > 16 ? 4 : 2;
    // With the secondary channel enabled, DMA alternates primary and secondary words: one frame carries both.
    int channels = (transmitStereoFrameSync ? 2 : 1) * (transmitSecondaryEnabled ? 2 : 1);
    int bitsPerSample = transmitWordLength;
    int frameSize = wordSize * channels;
    size_t requestedSamples = length / frameSize;
    if (requestedSamples == 0) return 0;

    // Start timing on first DMA call after SPORT enable
    if (!dmaTxActive_) {
        dmaTxStartNs_ = time_.Nanoseconds();
        totalTxSamplesDelivered_ = 0;
        dmaTxActive_ = true;
        LogConfig("TX");
    }

    // How many samples are due based on elapsed time?
    uint64_t elapsedNs = time_.Nanoseconds() - dmaTxStartNs_;
    uint64_t totalDue = (elapsedNs * sampleRateHz_) / 1'000'000'000ULL;
    // FIXME: wall time runs the emulator slower than real time, so it divides
    // the rate by TimeSource::kWallSlowdown; deterministic guest time does not.
    totalDue = totalDue / time_.Slowdown();
    uint64_t available = (totalDue > totalTxSamplesDelivered_)
                         ? (totalDue - totalTxSamplesDelivered_) : 0;

    if (available > requestedSamples) available = requestedSamples;
    if (available == 0) return 0;  // Not time yet — DMA retries next cycle

    if (dump_) fwrite(source, frameSize, available, dump_);
    if (audioOutputCallback) {
        audioOutputCallback(source, available, channels, bitsPerSample);
    }

    totalTxSamplesDelivered_ += available;
    return static_cast<u32>(available * frameSize);
}

void SPORT::LogConfig(const char* dir)
{
    const bool tx = dir[0] == 'T';
    LogInfo("SPORT%d %s DMA start: CR1=%#x wordlen=%d stereo=%d CLKDIV=%u FSDIV=%u secondary=%d", sportNumber, dir,
            tx ? tcr1Raw_ : rcr1Raw_, tx ? transmitWordLength : receiveWordLength,
            tx ? transmitStereoFrameSync : receiveStereoFrameSync, tx ? tclkdiv_ : rclkdiv_, tx ? tfsdiv_ : rfsdiv_,
            tx ? transmitSecondaryEnabled : receiveSecondaryEnabled);
    if (tx && !dump_) {
        if (const char* d = std::getenv("OP1EMU_AUDIO_DUMP")) {
            const std::string path = std::string(d) + "/sport" + std::to_string(sportNumber) + ".raw";
            dump_ = std::fopen(path.c_str(), "wb");
            if (dump_) LogInfo("SPORT%d: dumping TX samples to %s", sportNumber, path.c_str());
        }
    }
}
