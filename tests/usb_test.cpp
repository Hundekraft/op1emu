#include "cpu/usb.h"
#include "cpu/time_source.h"
#include <cstdio>
#include <cstdlib>

static void Require(bool ok, const char* why) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", why); std::exit(1); }
}
int main() {
    TimeSource time(true);
    USB usb(0xffc03800, time);
    bool irq = false, irq2 = false;
    usb.BindInterrupt(1, 2, 3, 4, [&](int n, int level) { if (n == 1) irq = level; if (n == 3) irq2 = level; });
    usb.Write32(0x30, 1); // global enable
    usb.Write32(0x04, 0x60); // soft connection, high-speed capability
    usb.Write32(0x1c, 8); // SOF interrupt
    time.Retire(400000);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 0 && !irq, "no frames without a host");
    usb.SetHostConnected(true);
    Require(usb.Read32(0x18) == 4, "host enumeration begins with bus reset");
    usb.Write32(0x18, 4);
    usb.ProcessWithInterrupt(0);
    time.Retire(399999);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 0 && !irq, "no early frame");
    time.Retire(1);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 1 && irq && usb.Read32(0x18) == 8, "1 ms frame and SOF IRQ");
    usb.Write32(0x18, 8);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x18) == 0 && !irq, "SOF write-one-to-clear, no repeat at same time");
    usb.Write32(0x1c, 0);
    time.Retire(400000 * 3);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 4 && usb.Read32(0x18) == 8 && !irq, "masked IRQ still advances frames");
    usb.Write32(0x20, 2047);
    time.Retire(400000);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 0, "frame wraps at 2048");
    usb.SetHostConnected(false);
    time.Retire(4000000);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x20) == 0 && usb.Read32(0x18) == 0, "detach stops frames and clears SOF");
    usb.SetHostConnected(true);
    usb.Write32(0x30, 0x41); // EP0 + EP6 TX
    usb.Write32(0x380, 828);
    usb.Write32(0x384, 0x4000); // isochronous
    bool received = false;
    usb.HandleDataRead(6, 8, 6, [&](const u8* data, std::size_t size) {
        received = size == 6 && data[0] == 1 && data[5] == 6;
    });
    usb.ProcessWithInterrupt(0);
    Require((usb.Read32(0x08) & 0x40) && (usb.Read32(0x384) & 4), "empty ISO IN raises underrun/TX interrupt");
    usb.Write32(0x08, 0x40);
    usb.Write32(0x3a8, 6);
    usb.Write32(0xb0, 0x0201);
    usb.Write32(0xb0, 0x0403);
    usb.Write32(0xb0, 0x0605);
    usb.Write32(0x384, 0x4001);
    usb.ProcessWithInterrupt(0);
    Require(!received, "ISO packet waits for next host poll");
    time.Retire(400000);
    usb.ProcessWithInterrupt(0);
    Require(received && !(usb.Read32(0x384) & 1), "ISO IN transfers packet at requested interval");
    USBSetupBytes setup{};
    setup.bmRequestType = 0x22;
    setup.bRequest = 1;
    setup.wValue = 0x100;
    setup.wIndex = 0x86;
    setup.wLength = 3;
    const u8 rate[] = {0x44, 0xac, 0};
    bool controlDone = false;
    usb.HandleSetupPacket(setup, rate, sizeof(rate), [&](const u8*, std::size_t size) {
        controlDone = size == 0;
    });
    Require(usb.Read32(0x210) == 8, "SETUP arrives separately from OUT payload");
    Require(usb.Read32(0x80) == 0x0122, "SETUP request bytes intact");
    for (int n = 0; n < 3; ++n) usb.Read32(0x80);
    usb.Write32(0x08, 1);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x210) == 0 && !controlDone, "OUT data waits for serviced SETUP");
    usb.Write32(0x204, 0x40);
    usb.ProcessWithInterrupt(0);
    Require(usb.Read32(0x210) == 3 && (usb.Read32(0x08) & 1), "OUT data raises EP0 TX interrupt");
    Require(usb.Read32(0x80) == 0xac44 && usb.Read32(0x80) == 0, "OUT payload intact");
    usb.Write32(0x204, 0x48);
    usb.ProcessWithInterrupt(0);
    Require(controlDone, "control OUT completes at DATAEND");
    bool isoDone = false;
    usb.HandleIsoRead(6, 8, {6, 6}, [&](const std::vector<u8>& data, const std::vector<u32>& lengths) {
        isoDone = lengths == std::vector<u32>({0, 3}) && data == std::vector<u8>({7, 8, 9});
    });
    time.Retire(400000);
    usb.ProcessWithInterrupt(0);
    Require(!isoDone, "empty ISO poll counts as a packet, URB still pending");
    usb.Write32(0x3a8, 3);
    usb.Write32(0xb0, 0x0807);
    usb.Write32(0xb0, 9);
    usb.Write32(0x384, 0x4001);
    time.Retire(399999);
    usb.ProcessWithInterrupt(0);
    Require(!isoDone, "second ISO packet cannot be polled early");
    time.Retire(1);
    usb.ProcessWithInterrupt(0);
    Require(isoDone, "ISO URB preserves empty and short packet lengths and payload");
    usb.Write32(0x2c, 7); // firmware routes every source to INT0
    usb.ProcessWithInterrupt(0);
    Require(irq && !irq2, "GLOBINTR bit 8 clears default TX-to-INT2 route");
    usb.Write32(0x3a8, 2);
    usb.Write32(0xb0, 0x1234);
    Require(usb.Read32(0x384) & 2, "FIFO_NOT_EMPTY reports short packets below MAXP");
    usb.Write32(0x384, 0x4009);
    Require(!(usb.Read32(0x384) & 3), "TX flush discards packet and clears TXPKTRDY");
    bool phaseDone = false;
    usb.HandleIsoRead(6, 8, {6, 6}, [&](const std::vector<u8>& data, const std::vector<u32>& lengths) {
        phaseDone = data.empty() && lengths == std::vector<u32>({0, 0});
    });
    time.Retire(400004); // CPU observes the first token 10 ns late
    usb.ProcessWithInterrupt(0);
    Require(!phaseDone, "first delayed token leaves second packet pending");
    time.Retire(399996); // next token is still due on the original clock phase
    usb.ProcessWithInterrupt(0);
    Require(phaseDone, "CPU polling latency does not accumulate in ISO host clock");
    std::puts("USB frame, control, ISO and TX register regressions passed");
}
