#pragma once

#include <cstdlib>
#include <cstring>

// OP1EMU_PANEL=b (SSD1963): R246 sends each frame bottom-up (DMA from memory row 375 down to 0) and that panel scans it
// back, so a frontend that keeps lines in arrival order shows the screen mirrored. True when lines must be stored
// reversed to show what the unit shows.
inline bool PanelLinesReversed() {
    const char* v = std::getenv("OP1EMU_PANEL");
    return v && !std::strcmp(v, "b");
}

class Display {
public:
    virtual ~Display() {}

    virtual void Initialize(int rows, int lines) = 0;
    virtual void UpdateRowBuffer(int x, int y, const void* data, int length) = 0;
};