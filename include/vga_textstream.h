/*
 * VGA Text Streaming Driver
 *
 * Streams VGA text mode output over Unix sockets with a framed
 * protocol. Handles keyboard and mouse input.
 *
 * Copyright (C) 2025 Moonshot Enterprises
 */

#ifndef DOSBOX_VGA_TEXTSTREAM_H
#define DOSBOX_VGA_TEXTSTREAM_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>

#include "vga.h"
#include "dosbox.h"

// Protocol version
constexpr uint16_t STREAM_PROTOCOL_VERSION = 0x0001;

// Maximum supported text mode dimensions
constexpr int TEXTSTREAM_MAX_COLS = 132;
constexpr int TEXTSTREAM_MAX_ROWS = 60;

// Channel IDs
enum class StreamChannel : uint8_t {
    CONTROL      = 0x00,
    TEXT_OUT     = 0x01,
    KEYBOARD_IN  = 0x02,
    MOUSE_IN     = 0x03,

    GFX_RAW      = 0x40,
    GFX_PNG      = 0x41,
    GFX_JPEG     = 0x42,
    GFX_H264     = 0x43,
    AUDIO_PCM    = 0x50,
    AUDIO_OPUS   = 0x51,
};

// Control message types
enum class ControlMsg : uint8_t {
    HELLO              = 0x01,
    GOODBYE            = 0x02,

    MODE_TEXT          = 0x10,
    MODE_GRAPHICS      = 0x11,
    MODE_UNSUPPORTED   = 0x12,

    REFRESH            = 0x20,
    RESIZE             = 0x21,

    CAPS_QUERY         = 0x30,
    CAPS_REPLY         = 0x31,
};

// Capability IDs
enum class StreamCap : uint8_t {
    TEXT_OUTPUT    = 0x01,
    KEYBOARD_INPUT = 0x02,
    MOUSE_INPUT    = 0x03,
    GRAPHICS_PNG   = 0x10,
    GRAPHICS_JPEG  = 0x11,
    GRAPHICS_H264  = 0x12,
    AUDIO_PCM      = 0x20,
    AUDIO_OPUS     = 0x21,
};

struct TextCell {
    uint8_t character;
    uint8_t attribute;
    bool operator==(const TextCell& o) const {
        return character == o.character && attribute == o.attribute;
    }
    bool operator!=(const TextCell& o) const { return !(*this == o); }
};

struct TextCursor {
    uint16_t row;
    uint16_t col;
    bool visible;
    bool operator!=(const TextCursor& o) const {
        return row != o.row || col != o.col || visible != o.visible;
    }
};

class VGATextStream {
public:
    VGATextStream();
    ~VGATextStream();

    // Socket management
    bool Listen(const std::string& primary_path, const std::string& bulk_path = "");
    void Close();
    bool IsConnected() const { return client_fd_ >= 0; }

    // Called from VGA vsync
    void OnVSync();

    // Force full redraw
    void Invalidate();

    // Mode queries
    bool IsTextMode() const;
    bool IsGraphicsMode() const;

    // Enable/disable
    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

    // Graphics streaming
    void CaptureGraphicsFrame(Bitu width, Bitu height, Bitu bpp,
                              Bitu pitch, Bitu flags,
                              const uint8_t* data, const uint8_t* pal);
    bool ShouldSendFrame();
    bool ClientWantsGraphics() const { return client_wants_graphics_; }

private:
    // Protocol framing
    void SendMessage(StreamChannel channel, const uint8_t* data, size_t len);
    void SendControl(ControlMsg msg, const uint8_t* data = nullptr, size_t len = 0);
    bool ReadMessage(StreamChannel& channel, std::vector<uint8_t>& payload);

    // Session management
    void HandleHello(const std::vector<uint8_t>& payload);
    void SendHello();
    void SendModeNotification();

    // Text mode
    void SnapshotTextBuffer();
    void SnapshotCursor();
    void GenerateTextOutput();

    // ANSI generation
    void EmitMoveCursor(int row, int col);
    void EmitSetAttribute(uint8_t attr);
    void EmitCharacter(uint8_t ch);
    void EmitClearScreen();
    void EmitCursorVisibility(bool visible);
    void FlushTextOutput();

    // Input handling
    void InputThreadFunc();
    void ProcessMessage(StreamChannel channel, const std::vector<uint8_t>& payload);
    void HandleKeyboardInput(const uint8_t* data, size_t len);
    void HandleMouseInput(const uint8_t* data, size_t len);
    void HandleControlInput(const uint8_t* data, size_t len);

    // Graphics encoding
    bool EncodePNG(const uint8_t* data, int width, int height,
                   int bpp, Bitu pitch, const uint8_t* pal,
                   std::vector<uint8_t>& output);

    // ANSI input parsing
    void ProcessInputByte(uint8_t byte);
    void InjectKey(uint8_t scancode, uint8_t ascii, bool extended = false);

    // Sockets
    int listen_fd_ = -1;
    int client_fd_ = -1;
    int bulk_listen_fd_ = -1;
    int bulk_client_fd_ = -1;
    std::string primary_path_;
    std::string bulk_path_;

    // Threading
    std::thread input_thread_;
    std::atomic<bool> running_{false};
    std::mutex send_mutex_;

    // Frame buffers
    TextCell current_[TEXTSTREAM_MAX_ROWS][TEXTSTREAM_MAX_COLS];
    TextCell previous_[TEXTSTREAM_MAX_ROWS][TEXTSTREAM_MAX_COLS];
    TextCursor cursor_;
    TextCursor prev_cursor_;

    // Screen dimensions
    int cols_ = 80;
    int rows_ = 25;
    int prev_cols_ = 0;
    int prev_rows_ = 0;

    // ANSI state
    uint8_t ansi_attr_ = 0xFF;
    int ansi_row_ = -1;
    int ansi_col_ = -1;

    // Output buffer (for TEXT_OUT channel)
    std::vector<uint8_t> text_buffer_;

    // Input parser state
    enum class InputState { NORMAL, ESC, CSI, SS3 };
    InputState input_state_ = InputState::NORMAL;
    std::string csi_params_;

    // Mode tracking
    VGAModes last_mode_ = M_ERROR;
    bool mode_notified_ = false;

    // Client capabilities
    bool client_wants_text_ = true;
    bool client_wants_graphics_ = false;
    bool client_wants_audio_ = false;

    // Configuration
    bool enabled_ = false;
    bool force_redraw_ = true;
    int vsync_count_ = 0;
    bool handshake_done_ = false;

    // Graphics streaming
    std::vector<uint8_t> png_buffer_;
    int graphics_width_ = 0;
    int graphics_height_ = 0;
    int graphics_bpp_ = 0;

    // Frame rate limiting
    std::chrono::steady_clock::time_point last_frame_time_;
    int target_fps_ = 15;  // Conservative for PNG
};

// Global instance
extern VGATextStream* g_textstream;

// Init/shutdown
void VGA_TextStream_Init(const char* primary_path, const char* bulk_path = nullptr);
void VGA_TextStream_Shutdown();

#endif // DOSBOX_VGA_TEXTSTREAM_H
