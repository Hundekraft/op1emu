#pragma once

#include "cpu/io.h"
#include "cpu/gpio.h"
#include <vector>
#include <array>
#include <map>

class OLED : public GPIOPeripheral {
public:
    constexpr static int DATA_PINS = 16;

    OLED(std::array<GPIOConnection, DATA_PINS> bus, GPIOConnection cs, GPIOConnection rs, GPIOConnection rd, GPIOConnection wr);

    GPIOPinDirection GetDirection(int pin) const override;
    bool SetPinInput(int pin, GPIOPinLevel level) override;
    GPIOPinLevel GetPinOutput(int pin) const override;
    int GetPinCount() const override;

protected:
    u32 ReadFromBus() const;
    void WriteToBus(u32 value) const;

    u32 data = 0;
    bool chipSelection = false;
    bool registerSelection = false;
    bool read = false;
    bool write = false;
    std::map<u32, Register> registers;

    std::array<GPIOConnection, DATA_PINS> dataBus;
    Register* selectedRegister = nullptr;

    // OP1EMU_PANEL=c2|a|b: answer R246's panel probes as that panel variant would (OP1-Emulator
    // docs/firmware/display.md "Panel families and selection"). Unset or "default": no answers, as before.
    char panel = 0;
    u32 lastWord = 0;      // the bus word at the last WR rising edge
    int readIndex = 0;     // reads since that word
    bool AnswerProbe();
};