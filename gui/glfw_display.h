#pragma once

#include "peripheral/display.h"
#include "peripheral/keyboard.h"
#include "encoder_queue.h"
#include <GLFW/glfw3.h>
#include <vector>
#include <tuple>
#include <cstdint>
#include <string>
#include <mutex>

struct DisplayRect {
    int left;
    int top;
    int width;
    int height;
};

struct ButtonConfig {
    int left;
    int top;
    int width;
    int height;
    int bank;
    int index;
    std::string name;
};

struct KeycapConfig {
    std::string name;
    int glfwKey; // GLFW key code
    int bank;
    int index;
};

struct KnobConfig {
    int clockwiseLeft;
    int clockwiseTop;
    int clockwiseWidth;
    int clockwiseHeight;
    int counterClockwiseLeft;
    int counterClockwiseTop;
    int counterClockwiseWidth;
    int counterClockwiseHeight;
};

// A knob cap cut from the skin (centre and radius in skin pixels), drawn rotated over the skin.
struct KnobCap {
    std::string name;   // encoder name, or "volume_knob"
    float cx = 0, cy = 0, r = 0;
    GLuint texture = 0;
};

struct UIConfig {
    std::string background;
    DisplayRect display;
    float scale = 1.0f;
    std::vector<ButtonConfig> buttons;
    std::vector<KeycapConfig> keycaps;
    std::vector<std::tuple<ButtonConfig, ButtonConfig, uint8_t>> encoders;
    KnobConfig volumeKnob;
    std::vector<KnobCap> knobCaps;
};

class GLFWDisplay : public Display, public Keyboard {
public:
    GLFWDisplay();
    ~GLFWDisplay() override;

    // Display interface
    void Initialize(int rows, int lines) override;
    void UpdateRowBuffer(int x, int y, const void* data, int length) override;
    void SetOnFrameStartCallback(const std::function<void(Display&)>& callback) override {
        onFrameStartCallback_ = callback;
    }
    void RenderFramebuffer();
    void PollEvents();
    bool ShouldClose() const;
    void SwapBuffers();
    void SetDisplayRotation(int degrees);
    uint8_t GetVolumeValue() const { return volumeValue_; }

private:
    void CreateTexture();
    void LoadUIConfig(const std::string& path);
    GLuint LoadBackgroundTexture(const std::string& path, int& width, int& height);
    void DrawTexturedQuad(GLuint texture, float x, float y, float w, float h, float texW = 1.0f, float texH = 1.0f);
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void CursorPosCallback(GLFWwindow* window, double x, double y);
    int EncoderAt(double x, double y) const;
    void StepEncoder(size_t index, int edges);
    void PumpEncoders();
    void HandleMouseButton(int button, int action, double x, double y);
    bool InVolumeKnob(double x, double y) const;
    void AddVolume(double delta);
    void MakeKnobCaps(const unsigned char* rgba, int width, int height);
    void DrawKnobCaps();
    void HandleKeyboard(int key, int action);
    int MapKeyNameToGLFW(const std::string& keyName);

    GLFWwindow* window_ = nullptr;
    GLuint texture_ = 0;
    GLuint backgroundTexture_ = 0;
    std::vector<uint16_t> framebuffer_;
    int width_ = 0;
    int height_ = 0;
    int scale_ = 2;  // Scale factor for display
    int rotation_ = 90;  // Rotation in degrees
    bool dirty_ = false;
    std::mutex framebufferMutex_;  // Protect framebuffer access
    std::function<void(Display&)> onFrameStartCallback_;

    UIConfig uiConfig_;
    int windowWidth_ = 0;
    int windowHeight_ = 0;
    uint8_t volumeValue_ = 192;  // 0-255; the skin's knob is drawn at this value
    double volumeExact_ = 192;   // volumeValue_ before rounding (drag and trackpad deltas add up)
    bool volumeActive_ = false, volumeMoved_ = false;   // left button held on the volume knob

    // Encoder input from the mouse (see encoder_queue.h) and each encoder's push button.
    struct EncoderInput {
        int pushBank = -1, pushIndex = -1;
        DisplayRect pushRect{0, 0, 0, 0};
        double pushReleaseAt = 0;   // > 0: release the push button at this time
    };
    EncoderQueue encoderQueue_;
    std::vector<EncoderInput> encoderInput_;
    int activeEncoder_ = -1;        // encoder under a held left button
    double dragLastY_ = 0, dragAccum_ = 0;
    bool dragMoved_ = false, pressInPush_ = false;
};
