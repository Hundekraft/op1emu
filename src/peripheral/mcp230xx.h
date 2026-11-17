#pragma once

#include "cpu/twi.h"
#include "cpu/gpio.h"
#include <functional>
#include <array>

enum class MCP230XXModel {
    MCP23008,   // 8 GPIOs
    MCP23017,   // 16 GPIOs
};

class MCP230XX : public RegisterI2CPeripheral, public GPIOPeripheral {
public:
    MCP230XX(u32 address, MCP230XXModel model);

    bool Write(const u8* buffer, u32 length) override;
    bool Read(u8* buffer, u32 length) override;
    Register* Next(u32 addr) const override;
    void Stop() override { RegisterI2CPeripheral::Stop(); skipWrite = false; }

    void SetInterruptCallback(std::function<void(int)> callback) {
        interruptCallback = callback;
    }

    GPIOPinDirection GetDirection(int pin) const override;

    bool SetPinInput(int pin, GPIOPinLevel level) override;
    GPIOPinLevel GetPinOutput(int pin) const override;

    int GetPinCount() const override;

    // A board strap on an input pin (a jack switch): the pin reads `l` whenever it is an input, also after
    // the firmware has used it as an output and switched it back, and without the pull-up reading it high.
    void SetFixedInput(int pin, GPIOPinLevel l);

protected:
    void SwitchRegisterBank();
    void ForwardInterrupt(int bank);
    void ClearInterrupt(int bank);

    MCP230XXModel model;
    int gpioCount;
    int registerCount;

    // Register state storage (2 banks for MCP23017)
    std::array<u8, 2> iodirInput = {0xFF, 0xFF};
    std::array<u8, 2> ipol = {0, 0};
    std::array<u8, 2> inten = {0, 0};
    std::array<u8, 2> defaultValue = {0, 0};
    std::array<u8, 2> intCompareDef = {0, 0};
    std::array<u8, 2> pullup = {0, 0};
    std::array<u8, 2> level = {0, 0};
    std::array<u8, 2> olat = {0, 0};
    std::array<u8, 2> intFlag = {0, 0};
    std::array<u8, 2> intcap = {0, 0};
    std::array<u8, 2> inputConnected = {0, 0};
    std::array<u8, 2> fixedMask = {0, 0};
    std::array<u8, 2> fixedLevel = {0, 0};
    std::array<bool, 2> intActive = {false, false};

    bool skipWrite = false;   // MCP23008: the data of a write to a register the chip lacks
    bool readZero = false;    // MCP23008: the register pointer is one the chip lacks (R246 probes 0x0C): reads give 0
    bool registerBank = false;
    bool intMirror = false;
    bool byteMode = false;
    bool intOutputOpenDrain = false;
    bool intActiveHigh = false;
    bool intClearOnReadIntcap = false;

    std::function<void(int)> interruptCallback;
    bool log_ = false;   // OP1EMU_MCP_LOG=ADDR (hex I2C address): trace this expander's transfers
};