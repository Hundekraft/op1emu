#pragma once

#include <algorithm>
#include <functional>
#include <vector>

// Encoder input from the mouse (click, drag, wheel): queued quadrature edges, sent one per encoder every
// kEdgeSeconds so R246's sampler sees each one (edges closer than ~20 ms of guest time get lost, see
// ARCHITECTURE.md "Keys and encoders"). Shared by the GLFW window and the headless "wheel" command.
class EncoderQueue {
public:
    static constexpr double kEdgeSeconds = 0.025;
    static constexpr int kEdgesPerStep = 8;      // a click or wheel notch: one engine-browser item
    static constexpr int kMaxPendingEdges = 64;

    struct Pins { int cwBank, cwIndex, ccwBank, ccwIndex; };
    using SetPin = std::function<void(int bank, int index, bool pressed)>;

    size_t Add(const Pins& pins) { enc_.push_back(Encoder{pins}); return enc_.size() - 1; }
    size_t Size() const { return enc_.size(); }

    void Step(size_t i, int edges) {
        if (i < enc_.size()) enc_[i].pending = std::clamp(enc_[i].pending + edges, -kMaxPendingEdges, kMaxPendingEdges);
    }
    // Wheel or trackpad: up = clockwise, one notch = kEdgesPerStep; fractions accumulate.
    void Wheel(size_t i, double notches) {
        if (i >= enc_.size()) return;
        auto& e = enc_[i];
        e.wheelAccum += notches * kEdgesPerStep;
        const int edges = static_cast<int>(e.wheelAccum);
        e.wheelAccum -= edges;
        Step(i, edges);
    }
    // GUI mouse wheel / trackpad: one browser item per gesture step. macOS delivers one physical notch as a
    // burst of events (scroll acceleration, momentum), so wheel events are ignored while an item is still
    // being sent (stopping half-way would leave the encoder between items). Small trackpad deltas add up.
    void Notch(size_t i, double delta) {
        if (i >= enc_.size() || delta == 0) return;
        auto& e = enc_[i];
        const int dir = delta > 0 ? 1 : -1;
        if (e.pending) { e.wheelAccum = 0; return; }
        if ((e.wheelAccum > 0) != (dir > 0)) e.wheelAccum = 0;
        e.wheelAccum += delta;
        if (e.wheelAccum >= 1 || e.wheelAccum <= -1) {
            e.wheelAccum = 0;
            Step(i, dir * kEdgesPerStep);
        }
    }
    // Sends at most one queued edge per encoder whose time has come (now in seconds, any monotonic clock).
    void Pump(double now, const SetPin& set) {
        static constexpr int kSequence[] = {0, 2, 3, 1};  // bit 1 = clockwise pin, bit 0 = counter-clockwise pin
        for (auto& e : enc_) {
            if (!e.pending || now < e.nextEdge) continue;
            e.phase = (e.phase + (e.pending > 0 ? 1 : 3)) % 4;
            e.pending += e.pending > 0 ? -1 : 1;
            const int value = kSequence[e.phase];
            set(e.pins.cwBank, e.pins.cwIndex, (value & 2) != 0);
            set(e.pins.ccwBank, e.pins.ccwIndex, (value & 1) != 0);
            e.nextEdge = now + kEdgeSeconds;
        }
    }
    bool Idle() const {
        return std::all_of(enc_.begin(), enc_.end(), [](const Encoder& e) { return e.pending == 0; });
    }

private:
    struct Encoder {
        Pins pins;
        int phase = 0, pending = 0;
        double nextEdge = 0, wheelAccum = 0;
    };
    std::vector<Encoder> enc_;
};
