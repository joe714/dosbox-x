/*
 * VGA Text Streaming Driver - Implementation
 *
 * Copyright (C) 2025 Moonshot Enterprises
 */

#include "dosbox.h"
#include "vga.h"
#include "vga_textstream.h"
#include "bios.h"
#include "mem.h"
#include "logging.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <algorithm>

VGATextStream* g_textstream = nullptr;

//-----------------------------------------------------------------------------
// CP437 to Unicode mapping
//-----------------------------------------------------------------------------

static const uint16_t cp437_to_unicode[256] = {
    // 0x00-0x1F: Special graphics
    0x0000, 0x263A, 0x263B, 0x2665, 0x2666, 0x2663, 0x2660, 0x2022,
    0x25D8, 0x25CB, 0x25D9, 0x2642, 0x2640, 0x266A, 0x266B, 0x263C,
    0x25BA, 0x25C4, 0x2195, 0x203C, 0x00B6, 0x00A7, 0x25AC, 0x21A8,
    0x2191, 0x2193, 0x2192, 0x2190, 0x221F, 0x2194, 0x25B2, 0x25BC,
    // 0x20-0x7E: ASCII
    0x0020, 0x0021, 0x0022, 0x0023, 0x0024, 0x0025, 0x0026, 0x0027,
    0x0028, 0x0029, 0x002A, 0x002B, 0x002C, 0x002D, 0x002E, 0x002F,
    0x0030, 0x0031, 0x0032, 0x0033, 0x0034, 0x0035, 0x0036, 0x0037,
    0x0038, 0x0039, 0x003A, 0x003B, 0x003C, 0x003D, 0x003E, 0x003F,
    0x0040, 0x0041, 0x0042, 0x0043, 0x0044, 0x0045, 0x0046, 0x0047,
    0x0048, 0x0049, 0x004A, 0x004B, 0x004C, 0x004D, 0x004E, 0x004F,
    0x0050, 0x0051, 0x0052, 0x0053, 0x0054, 0x0055, 0x0056, 0x0057,
    0x0058, 0x0059, 0x005A, 0x005B, 0x005C, 0x005D, 0x005E, 0x005F,
    0x0060, 0x0061, 0x0062, 0x0063, 0x0064, 0x0065, 0x0066, 0x0067,
    0x0068, 0x0069, 0x006A, 0x006B, 0x006C, 0x006D, 0x006E, 0x006F,
    0x0070, 0x0071, 0x0072, 0x0073, 0x0074, 0x0075, 0x0076, 0x0077,
    0x0078, 0x0079, 0x007A, 0x007B, 0x007C, 0x007D, 0x007E, 0x2302,
    // 0x80-0xFF: Extended
    0x00C7, 0x00FC, 0x00E9, 0x00E2, 0x00E4, 0x00E0, 0x00E5, 0x00E7,
    0x00EA, 0x00EB, 0x00E8, 0x00EF, 0x00EE, 0x00EC, 0x00C4, 0x00C5,
    0x00C9, 0x00E6, 0x00C6, 0x00F4, 0x00F6, 0x00F2, 0x00FB, 0x00F9,
    0x00FF, 0x00D6, 0x00DC, 0x00A2, 0x00A3, 0x00A5, 0x20A7, 0x0192,
    0x00E1, 0x00ED, 0x00F3, 0x00FA, 0x00F1, 0x00D1, 0x00AA, 0x00BA,
    0x00BF, 0x2310, 0x00AC, 0x00BD, 0x00BC, 0x00A1, 0x00AB, 0x00BB,
    0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561, 0x2562, 0x2556,
    0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B, 0x2510,
    0x2514, 0x2534, 0x252C, 0x251C, 0x2500, 0x253C, 0x255E, 0x255F,
    0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567,
    0x2568, 0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x256B,
    0x256A, 0x2518, 0x250C, 0x2588, 0x2584, 0x258C, 0x2590, 0x2580,
    0x03B1, 0x00DF, 0x0393, 0x03C0, 0x03A3, 0x03C3, 0x00B5, 0x03C4,
    0x03A6, 0x0398, 0x03A9, 0x03B4, 0x221E, 0x03C6, 0x03B5, 0x2229,
    0x2261, 0x00B1, 0x2265, 0x2264, 0x2320, 0x2321, 0x00F7, 0x2248,
    0x00B0, 0x2219, 0x00B7, 0x221A, 0x207F, 0x00B2, 0x25A0, 0x00A0
};

// VGA attribute to ANSI color mapping
static const int vga_fg[16] = {30,34,32,36,31,35,33,37,90,94,92,96,91,95,93,97};
static const int vga_bg[8]  = {40,44,42,46,41,45,43,47};

// ASCII to scancode table
static const uint8_t ascii_scancode[128] = {
    0x00,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, 0x0E,0x0F,0x1C,0x25,0x26,0x1C,0x31,0x18,
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, 0x2D,0x15,0x2C,0x01,0x2B,0x1B,0x07,0x0C,
    0x39,0x02,0x28,0x04,0x05,0x06,0x08,0x28, 0x0A,0x0B,0x09,0x0D,0x33,0x0C,0x34,0x35,
    0x0B,0x02,0x03,0x04,0x05,0x06,0x07,0x08, 0x09,0x0A,0x27,0x27,0x33,0x0D,0x34,0x35,
    0x03,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, 0x23,0x17,0x24,0x25,0x26,0x32,0x31,0x18,
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, 0x2D,0x15,0x2C,0x1A,0x2B,0x1B,0x07,0x0C,
    0x29,0x1E,0x30,0x2E,0x20,0x12,0x21,0x22, 0x23,0x17,0x24,0x25,0x26,0x32,0x31,0x18,
    0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11, 0x2D,0x15,0x2C,0x1A,0x2B,0x1B,0x29,0x0E,
};

//-----------------------------------------------------------------------------
// Constructor / Destructor
//-----------------------------------------------------------------------------

VGATextStream::VGATextStream() {
    memset(current_, 0, sizeof(current_));
    memset(previous_, 0, sizeof(previous_));
    text_buffer_.reserve(16384);
}

VGATextStream::~VGATextStream() {
    Close();
}

//-----------------------------------------------------------------------------
// Socket Management
//-----------------------------------------------------------------------------

bool VGATextStream::Listen(const std::string& primary_path, const std::string& bulk_path) {
    primary_path_ = primary_path;
    bulk_path_ = bulk_path;

    // Remove existing socket files
    unlink(primary_path.c_str());

    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        LOG_MSG("TEXTSTREAM: socket() failed: %s", strerror(errno));
        return false;
    }

    struct sockaddr_un addr = {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, primary_path.c_str(), sizeof(addr.sun_path) - 1);

    if (bind(listen_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_MSG("TEXTSTREAM: bind() failed: %s", strerror(errno));
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (listen(listen_fd_, 1) < 0) {
        LOG_MSG("TEXTSTREAM: listen() failed: %s", strerror(errno));
        close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    fcntl(listen_fd_, F_SETFL, O_NONBLOCK);

    // TODO: Set up bulk socket if path provided (Phase 2)

    running_ = true;
    input_thread_ = std::thread(&VGATextStream::InputThreadFunc, this);

    LOG_MSG("TEXTSTREAM: Listening on %s (protocol v%d.%d)",
            primary_path.c_str(),
            STREAM_PROTOCOL_VERSION >> 8,
            STREAM_PROTOCOL_VERSION & 0xFF);
    return true;
}

void VGATextStream::Close() {
    running_ = false;

    if (input_thread_.joinable()) {
        input_thread_.join();
    }

    if (client_fd_ >= 0) {
        close(client_fd_);
        client_fd_ = -1;
    }

    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }

    if (!primary_path_.empty()) {
        unlink(primary_path_.c_str());
        primary_path_.clear();
    }

    handshake_done_ = false;
}

//-----------------------------------------------------------------------------
// Protocol Framing
//-----------------------------------------------------------------------------

void VGATextStream::SendMessage(StreamChannel channel, const uint8_t* data, size_t len) {
    if (client_fd_ < 0 || len > 0xFFFFFF) return;

    std::lock_guard<std::mutex> lock(send_mutex_);

    uint8_t header[4];
    header[0] = static_cast<uint8_t>(channel);
    header[1] = (len >> 16) & 0xFF;
    header[2] = (len >> 8) & 0xFF;
    header[3] = len & 0xFF;

    // Send header
    ssize_t sent = write(client_fd_, header, 4);
    if (sent != 4) {
        LOG_MSG("TEXTSTREAM: Header write failed");
        return;
    }

    // Send payload
    if (len > 0 && data) {
        sent = write(client_fd_, data, len);
        if (sent != (ssize_t)len) {
            LOG_MSG("TEXTSTREAM: Payload write failed");
        }
    }
}

void VGATextStream::SendControl(ControlMsg msg, const uint8_t* data, size_t len) {
    std::vector<uint8_t> payload;
    payload.reserve(1 + len);
    payload.push_back(static_cast<uint8_t>(msg));
    if (data && len > 0) {
        payload.insert(payload.end(), data, data + len);
    }
    SendMessage(StreamChannel::CONTROL, payload.data(), payload.size());
}

bool VGATextStream::ReadMessage(StreamChannel& channel, std::vector<uint8_t>& payload) {
    if (client_fd_ < 0) return false;

    uint8_t header[4];
    ssize_t n = read(client_fd_, header, 4);
    if (n != 4) return false;

    channel = static_cast<StreamChannel>(header[0]);
    size_t len = ((size_t)header[1] << 16) | ((size_t)header[2] << 8) | header[3];

    payload.resize(len);
    if (len > 0) {
        size_t total = 0;
        while (total < len) {
            n = read(client_fd_, payload.data() + total, len - total);
            if (n <= 0) return false;
            total += n;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------
// Session Management
//-----------------------------------------------------------------------------

void VGATextStream::SendHello() {
    // Build HELLO: version (2) + cap count (1) + caps (N)
    std::vector<uint8_t> caps = {
        static_cast<uint8_t>(StreamCap::TEXT_OUTPUT),
        static_cast<uint8_t>(StreamCap::KEYBOARD_INPUT),
        static_cast<uint8_t>(StreamCap::MOUSE_INPUT),
        // Phase 2: add graphics/audio caps here
    };

    std::vector<uint8_t> payload;
    payload.push_back((STREAM_PROTOCOL_VERSION >> 8) & 0xFF);
    payload.push_back(STREAM_PROTOCOL_VERSION & 0xFF);
    payload.push_back(static_cast<uint8_t>(caps.size()));
    payload.insert(payload.end(), caps.begin(), caps.end());

    SendControl(ControlMsg::HELLO, payload.data(), payload.size());
}

void VGATextStream::HandleHello(const std::vector<uint8_t>& payload) {
    if (payload.size() < 3) return;

    uint16_t client_version = ((uint16_t)payload[0] << 8) | payload[1];
    uint8_t cap_count = payload[2];

    LOG_MSG("TEXTSTREAM: Client version %d.%d, %d capabilities",
            client_version >> 8, client_version & 0xFF, cap_count);

    // Parse client capabilities
    client_wants_text_ = false;
    client_wants_graphics_ = false;
    client_wants_audio_ = false;

    for (size_t i = 0; i < cap_count && (3 + i) < payload.size(); i++) {
        uint8_t cap = payload[3 + i];
        switch (static_cast<StreamCap>(cap)) {
            case StreamCap::TEXT_OUTPUT:
                client_wants_text_ = true;
                break;
            case StreamCap::GRAPHICS_PNG:
            case StreamCap::GRAPHICS_JPEG:
            case StreamCap::GRAPHICS_H264:
                client_wants_graphics_ = true;
                break;
            case StreamCap::AUDIO_PCM:
            case StreamCap::AUDIO_OPUS:
                client_wants_audio_ = true;
                break;
            default:
                break;
        }
    }

    handshake_done_ = true;

    // Send mode notification
    SendModeNotification();
}

void VGATextStream::SendModeNotification() {
    if (!handshake_done_) return;

    if (IsTextMode()) {
        uint8_t data[4];
        data[0] = (cols_ >> 8) & 0xFF;
        data[1] = cols_ & 0xFF;
        data[2] = (rows_ >> 8) & 0xFF;
        data[3] = rows_ & 0xFF;
        SendControl(ControlMsg::MODE_TEXT, data, 4);
        mode_notified_ = true;
    } else if (IsGraphicsMode()) {
        // Phase 2: send MODE_GRAPHICS with dimensions
        SendControl(ControlMsg::MODE_UNSUPPORTED);
        mode_notified_ = true;
    }
}

//-----------------------------------------------------------------------------
// Input Thread
//-----------------------------------------------------------------------------

void VGATextStream::InputThreadFunc() {
    while (running_) {
        // Accept new connections
        if (client_fd_ < 0 && listen_fd_ >= 0) {
            client_fd_ = accept(listen_fd_, nullptr, nullptr);
            if (client_fd_ >= 0) {
                fcntl(client_fd_, F_SETFL, O_NONBLOCK);
                LOG_MSG("TEXTSTREAM: Client connected");
                handshake_done_ = false;
                mode_notified_ = false;
                SendHello();
                Invalidate();
            }
        }

        // Read messages from client
        if (client_fd_ >= 0) {
            struct pollfd pfd = {client_fd_, POLLIN, 0};
            if (poll(&pfd, 1, 10) > 0) {
                StreamChannel channel;
                std::vector<uint8_t> payload;

                if (ReadMessage(channel, payload)) {
                    ProcessMessage(channel, payload);
                } else {
                    // Check if connection closed
                    char tmp;
                    ssize_t n = recv(client_fd_, &tmp, 1, MSG_PEEK);
                    if (n == 0 || (n < 0 && errno != EAGAIN)) {
                        LOG_MSG("TEXTSTREAM: Client disconnected");
                        close(client_fd_);
                        client_fd_ = -1;
                        handshake_done_ = false;
                    }
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

void VGATextStream::ProcessMessage(StreamChannel channel, const std::vector<uint8_t>& payload) {
    switch (channel) {
        case StreamChannel::CONTROL:
            HandleControlInput(payload.data(), payload.size());
            break;
        case StreamChannel::KEYBOARD_IN:
            HandleKeyboardInput(payload.data(), payload.size());
            break;
        case StreamChannel::MOUSE_IN:
            HandleMouseInput(payload.data(), payload.size());
            break;
        default:
            LOG_MSG("TEXTSTREAM: Unknown channel 0x%02X", static_cast<int>(channel));
            break;
    }
}

void VGATextStream::HandleControlInput(const uint8_t* data, size_t len) {
    if (len < 1) return;

    ControlMsg msg = static_cast<ControlMsg>(data[0]);
    const uint8_t* msg_data = len > 1 ? data + 1 : nullptr;
    size_t msg_len = len > 1 ? len - 1 : 0;

    switch (msg) {
        case ControlMsg::HELLO:
            HandleHello(std::vector<uint8_t>(data + 1, data + len));
            break;
        case ControlMsg::GOODBYE:
            LOG_MSG("TEXTSTREAM: Client sent GOODBYE");
            close(client_fd_);
            client_fd_ = -1;
            handshake_done_ = false;
            break;
        case ControlMsg::REFRESH:
            LOG_MSG("TEXTSTREAM: Refresh requested");
            Invalidate();
            break;
        case ControlMsg::RESIZE:
            if (msg_len >= 4) {
                uint16_t new_cols = ((uint16_t)msg_data[0] << 8) | msg_data[1];
                uint16_t new_rows = ((uint16_t)msg_data[2] << 8) | msg_data[3];
                LOG_MSG("TEXTSTREAM: Client resize %dx%d", new_cols, new_rows);
                // Could notify DOS programs via interrupt (future)
            }
            break;
        default:
            LOG_MSG("TEXTSTREAM: Unknown control message 0x%02X", static_cast<int>(msg));
            break;
    }
}

void VGATextStream::HandleKeyboardInput(const uint8_t* data, size_t len) {
    // Keyboard channel receives raw terminal input (ANSI sequences)
    for (size_t i = 0; i < len; i++) {
        ProcessInputByte(data[i]);
    }
}

void VGATextStream::HandleMouseInput(const uint8_t* data, size_t len) {
    // Phase 2: Parse mouse events and inject into DOSBox
    (void)data;
    (void)len;
}

//-----------------------------------------------------------------------------
// Keyboard Input Parsing (ANSI -> DOS scancodes)
//-----------------------------------------------------------------------------

void VGATextStream::ProcessInputByte(uint8_t byte) {
    switch (input_state_) {
    case InputState::NORMAL:
        if (byte == 0x1B) {
            input_state_ = InputState::ESC;
        } else if (byte == 0x7F) {
            InjectKey(0x0E, 0x08);  // DEL -> Backspace
        } else if (byte < 0x20) {
            // Control characters
            if (byte == 0x0D) InjectKey(0x1C, 0x0D);      // Enter
            else if (byte == 0x09) InjectKey(0x0F, 0x09); // Tab
            else if (byte == 0x08) InjectKey(0x0E, 0x08); // Backspace
            else if (byte >= 1 && byte <= 26) {
                uint8_t sc = ascii_scancode['a' + byte - 1];
                InjectKey(sc, byte);
            }
        } else if (byte < 0x80) {
            uint8_t sc = ascii_scancode[byte];
            InjectKey(sc, byte);
        }
        break;

    case InputState::ESC:
        if (byte == '[') {
            input_state_ = InputState::CSI;
            csi_params_.clear();
        } else if (byte == 'O') {
            input_state_ = InputState::SS3;
        } else {
            // Alt+key
            if (byte >= 'a' && byte <= 'z') {
                uint8_t sc = ascii_scancode[byte];
                BIOS_AddKeyToBuffer((uint16_t)sc << 8);
            } else {
                InjectKey(0x01, 0x1B);  // Just ESC
            }
            input_state_ = InputState::NORMAL;
        }
        break;

    case InputState::CSI:
        if (byte >= 0x30 && byte <= 0x3F) {
            csi_params_ += (char)byte;
        } else if (byte >= 0x40 && byte <= 0x7E) {
            // Final byte - dispatch based on it
            switch (byte) {
                case 'A': InjectKey(0x48, 0, true); break; // Up
                case 'B': InjectKey(0x50, 0, true); break; // Down
                case 'C': InjectKey(0x4D, 0, true); break; // Right
                case 'D': InjectKey(0x4B, 0, true); break; // Left
                case 'H': InjectKey(0x47, 0, true); break; // Home
                case 'F': InjectKey(0x4F, 0, true); break; // End
                case '~': {
                    int param = csi_params_.empty() ? 0 : atoi(csi_params_.c_str());
                    switch (param) {
                        case 1: InjectKey(0x47, 0, true); break; // Home
                        case 2: InjectKey(0x52, 0, true); break; // Insert
                        case 3: InjectKey(0x53, 0, true); break; // Delete
                        case 4: InjectKey(0x4F, 0, true); break; // End
                        case 5: InjectKey(0x49, 0, true); break; // PgUp
                        case 6: InjectKey(0x51, 0, true); break; // PgDn
                        case 11: InjectKey(0x3B, 0, false); break; // F1
                        case 12: InjectKey(0x3C, 0, false); break; // F2
                        case 13: InjectKey(0x3D, 0, false); break; // F3
                        case 14: InjectKey(0x3E, 0, false); break; // F4
                        case 15: InjectKey(0x3F, 0, false); break; // F5
                        case 17: InjectKey(0x40, 0, false); break; // F6
                        case 18: InjectKey(0x41, 0, false); break; // F7
                        case 19: InjectKey(0x42, 0, false); break; // F8
                        case 20: InjectKey(0x43, 0, false); break; // F9
                        case 21: InjectKey(0x44, 0, false); break; // F10
                        case 23: InjectKey(0x85, 0, false); break; // F11
                        case 24: InjectKey(0x86, 0, false); break; // F12
                    }
                    break;
                }
            }
            csi_params_.clear();
            input_state_ = InputState::NORMAL;
        } else {
            input_state_ = InputState::NORMAL;
        }
        break;

    case InputState::SS3:
        switch (byte) {
            case 'A': InjectKey(0x48, 0, true); break; // Up
            case 'B': InjectKey(0x50, 0, true); break; // Down
            case 'C': InjectKey(0x4D, 0, true); break; // Right
            case 'D': InjectKey(0x4B, 0, true); break; // Left
            case 'P': InjectKey(0x3B, 0, false); break; // F1
            case 'Q': InjectKey(0x3C, 0, false); break; // F2
            case 'R': InjectKey(0x3D, 0, false); break; // F3
            case 'S': InjectKey(0x3E, 0, false); break; // F4
        }
        input_state_ = InputState::NORMAL;
        break;
    }
}

void VGATextStream::InjectKey(uint8_t scancode, uint8_t ascii, bool extended) {
    uint16_t keycode;
    if (extended) {
        keycode = ((uint16_t)scancode << 8) | 0x00;
    } else {
        keycode = ((uint16_t)scancode << 8) | ascii;
    }
    BIOS_AddKeyToBuffer(keycode);
}

//-----------------------------------------------------------------------------
// VSync Handler
//-----------------------------------------------------------------------------

bool VGATextStream::IsTextMode() const {
    return vga.mode == M_TEXT || vga.mode == M_HERC_TEXT || vga.mode == M_TANDY_TEXT;
}

bool VGATextStream::IsGraphicsMode() const {
    switch (vga.mode) {
        case M_CGA2: case M_CGA4: case M_CGA16:
        case M_EGA: case M_VGA:
        case M_LIN4: case M_LIN8: case M_LIN15: case M_LIN16: case M_LIN24: case M_LIN32:
            return true;
        default:
            return false;
    }
}

void VGATextStream::OnVSync() {
    if (!enabled_ || client_fd_ < 0 || !handshake_done_) return;

    vsync_count_++;

    // Detect mode changes
    VGAModes current_mode = vga.mode;
    if (current_mode != last_mode_) {
        last_mode_ = current_mode;
        mode_notified_ = false;
        SendModeNotification();
        force_redraw_ = true;
    }

    // Only stream if client wants this mode
    if (IsTextMode() && client_wants_text_) {
        // Periodic full refresh (every 2 seconds for sync)
        if ((vsync_count_ % 120) == 0) {
            force_redraw_ = true;
        }

        SnapshotTextBuffer();
        SnapshotCursor();
        GenerateTextOutput();
    } else if (IsGraphicsMode() && client_wants_graphics_) {
        // Phase 2: Graphics streaming
    }
}

void VGATextStream::Invalidate() {
    force_redraw_ = true;
    ansi_attr_ = 0xFF;
    ansi_row_ = -1;
    ansi_col_ = -1;
}

//-----------------------------------------------------------------------------
// Text Buffer Snapshot
//-----------------------------------------------------------------------------

void VGATextStream::SnapshotTextBuffer() {
    cols_ = (vga.crtc.offset > 0) ? (int)(vga.crtc.offset * 2) : 80;
    if (cols_ > TEXTSTREAM_MAX_COLS) cols_ = TEXTSTREAM_MAX_COLS;

    Bitu max_sl = vga.crtc.maximum_scan_line & 0x1F;
    rows_ = (max_sl > 0) ? (int)((vga.crtc.vertical_display_end + 1) / (max_sl + 1)) : 25;
    if (rows_ > TEXTSTREAM_MAX_ROWS) rows_ = TEXTSTREAM_MAX_ROWS;
    // Sanity check: standard text modes are at least 25 rows
    // Values less than this are likely transient states during mode switches
    if (rows_ < 24) rows_ = 25;

    if (cols_ != prev_cols_ || rows_ != prev_rows_) {
        force_redraw_ = true;
        prev_cols_ = cols_;
        prev_rows_ = rows_;
        SendModeNotification();  // Notify client of dimension change
    }

    PhysPt base = 0xB8000 + (vga.config.display_start * 2);

    for (int row = 0; row < rows_; row++) {
        for (int col = 0; col < cols_; col++) {
            PhysPt addr = base + (row * cols_ + col) * 2;
            current_[row][col].character = mem_readb(addr);
            current_[row][col].attribute = mem_readb(addr + 1);
        }
    }
}

void VGATextStream::SnapshotCursor() {
    Bitu pos = ((Bitu)vga.crtc.cursor_location_high << 8) | vga.crtc.cursor_location_low;
    cursor_.row = (cols_ > 0) ? (uint16_t)(pos / cols_) : 0;
    cursor_.col = (cols_ > 0) ? (uint16_t)(pos % cols_) : 0;
    cursor_.visible = !(vga.crtc.cursor_start & 0x20);
}

//-----------------------------------------------------------------------------
// ANSI Output Generation
//-----------------------------------------------------------------------------

void VGATextStream::GenerateTextOutput() {
    text_buffer_.clear();

    bool full = force_redraw_;
    force_redraw_ = false;

    if (full) {
        // Full redraw - use line-by-line output with explicit newlines
        // This works regardless of client terminal width
        EmitClearScreen();
        // Emit reset sequence to ensure terminal has default attributes
        EmitSetAttribute(0x07);

        for (int row = 0; row < rows_; row++) {
            // Move to start of row (needed after clear screen)
            if (row > 0) {
                // Reset to default before newline to prevent background bleeding
                if (ansi_attr_ != 0x07) {
                    EmitSetAttribute(0x07);
                }
                text_buffer_.push_back('\r');
                text_buffer_.push_back('\n');
            }

            // Find last non-space character OR last character with non-default background
            // Spaces with colored backgrounds must be output to display correctly
            int last_col = cols_ - 1;
            while (last_col >= 0 && current_[row][last_col].character == ' ' &&
                   (current_[row][last_col].attribute & 0x70) == 0) {  // Check if bg is black
                last_col--;
            }

            for (int col = 0; col <= last_col; col++) {
                const TextCell& curr = current_[row][col];
                if (curr.attribute != ansi_attr_) {
                    EmitSetAttribute(curr.attribute);
                }
                EmitCharacter(curr.character);
            }

            // Reset after line content to prevent background bleeding to edge
            if (ansi_attr_ != 0x07 && last_col < cols_ - 1) {
                EmitSetAttribute(0x07);
            }
        }
        ansi_row_ = rows_ - 1;
        ansi_col_ = 0;
    } else {
        // Differential update - use cursor positioning for efficiency
        int write_row = -1, write_col = -1;

        for (int row = 0; row < rows_; row++) {
            for (int col = 0; col < cols_; col++) {
                const TextCell& curr = current_[row][col];
                const TextCell& prev = previous_[row][col];

                if (curr != prev) {
                    if (row != write_row || col != write_col) {
                        EmitMoveCursor(row, col);
                    }
                    if (curr.attribute != ansi_attr_) {
                        EmitSetAttribute(curr.attribute);
                    }
                    EmitCharacter(curr.character);

                    write_row = row;
                    write_col = col + 1;
                    if (write_col >= cols_) {
                        write_col = 0;
                        write_row++;
                    }
                }
            }
        }
    }

    // Cursor handling - position first, then show/hide
    // This prevents the cursor from briefly appearing at the wrong location
    if (cursor_ != prev_cursor_) {
        if (cursor_.visible) {
            EmitMoveCursor(cursor_.row, cursor_.col);
        }
        if (cursor_.visible != prev_cursor_.visible) {
            EmitCursorVisibility(cursor_.visible);
        }
    }

    FlushTextOutput();

    memcpy(previous_, current_, sizeof(previous_));
    prev_cursor_ = cursor_;
}

void VGATextStream::EmitMoveCursor(int row, int col) {
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "\x1b[%d;%dH", row + 1, col + 1);
    text_buffer_.insert(text_buffer_.end(), buf, buf + n);
    ansi_row_ = row;
    ansi_col_ = col;
}

void VGATextStream::EmitSetAttribute(uint8_t attr) {
    int fg = vga_fg[attr & 0x0F];
    int bg = vga_bg[(attr >> 4) & 0x07];
    bool blink = (attr & 0x80) != 0;

    char buf[32];
    int n;
    if (blink) {
        n = snprintf(buf, sizeof(buf), "\x1b[0;%d;%d;5m", fg, bg);
    } else {
        n = snprintf(buf, sizeof(buf), "\x1b[0;%d;%dm", fg, bg);
    }
    text_buffer_.insert(text_buffer_.end(), buf, buf + n);
    ansi_attr_ = attr;
}

void VGATextStream::EmitCharacter(uint8_t ch) {
    uint16_t u = cp437_to_unicode[ch];

    if (u < 0x80) {
        text_buffer_.push_back((uint8_t)u);
    } else if (u < 0x800) {
        text_buffer_.push_back((uint8_t)(0xC0 | (u >> 6)));
        text_buffer_.push_back((uint8_t)(0x80 | (u & 0x3F)));
    } else {
        text_buffer_.push_back((uint8_t)(0xE0 | (u >> 12)));
        text_buffer_.push_back((uint8_t)(0x80 | ((u >> 6) & 0x3F)));
        text_buffer_.push_back((uint8_t)(0x80 | (u & 0x3F)));
    }

    ansi_col_++;
    if (ansi_col_ >= cols_) {
        ansi_col_ = 0;
        ansi_row_++;
    }
}

void VGATextStream::EmitClearScreen() {
    // Hide cursor first to prevent it being visible at wrong position during redraw
    const char* hide = "\x1b[?25l";
    text_buffer_.insert(text_buffer_.end(), hide, hide + 6);
    // Clear screen and home cursor
    const char* seq = "\x1b[2J\x1b[H";
    text_buffer_.insert(text_buffer_.end(), seq, seq + 7);
    ansi_row_ = 0;
    ansi_col_ = 0;
}

void VGATextStream::EmitCursorVisibility(bool visible) {
    const char* seq = visible ? "\x1b[?25h" : "\x1b[?25l";
    text_buffer_.insert(text_buffer_.end(), seq, seq + 6);
}

void VGATextStream::FlushTextOutput() {
    if (text_buffer_.empty()) return;
    SendMessage(StreamChannel::TEXT_OUT, text_buffer_.data(), text_buffer_.size());
}

//-----------------------------------------------------------------------------
// Global Init/Shutdown
//-----------------------------------------------------------------------------

void VGA_TextStream_Init(const char* primary_path, const char* bulk_path) {
    if (!g_textstream && primary_path && primary_path[0]) {
        g_textstream = new VGATextStream();
        g_textstream->Listen(primary_path, bulk_path ? bulk_path : "");
        g_textstream->SetEnabled(true);
    }
}

void VGA_TextStream_Shutdown() {
    if (g_textstream) {
        delete g_textstream;
        g_textstream = nullptr;
    }
}
