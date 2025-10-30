#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

namespace sphero_rvr_control
{

// Official Sphero API protocol constants from sphero-sdk-raspberrypi-python
// Reference: https://sdk.sphero.com/documentation/api-documents
// Control bytes for packet framing
static constexpr uint8_t SOP = 0x8D;      // Start Of Packet
static constexpr uint8_t EOP = 0xD8;      // End Of Packet
static constexpr uint8_t ESC = 0xAB;      // Escape character
static constexpr uint8_t ESC_SOP = 0x05;  // Escaped SOP value
static constexpr uint8_t ESC_EOP = 0x50;  // Escaped EOP value
static constexpr uint8_t ESC_ESC = 0x23;  // Escaped ESC value

// Flags (bitfield) from api_sphero_protocol.py Flags(IntEnum)
static constexpr uint8_t FLAG_IS_RESPONSE = 0x01;
static constexpr uint8_t FLAG_REQUESTS_RESPONSE = 0x02;
static constexpr uint8_t FLAG_REQUESTS_RESPONSE_IF_ERROR = 0x04;
static constexpr uint8_t FLAG_IS_ACTIVITY = 0x08;
static constexpr uint8_t FLAG_HAS_TARGET = 0x10;
static constexpr uint8_t FLAG_HAS_SOURCE = 0x20;
static constexpr uint8_t FLAG_UNUSED = 0x40;
static constexpr uint8_t FLAG_HAS_MORE_FLAGS = 0x80;

// Device IDs from devices.py DevicesEnum
static constexpr uint8_t DID_API_AND_SHELL = 0x10;
static constexpr uint8_t DID_SYSTEM_INFO = 0x11;
static constexpr uint8_t DID_POWER = 0x13;
static constexpr uint8_t DID_DRIVE = 0x16;
static constexpr uint8_t DID_SENSOR = 0x18;
static constexpr uint8_t DID_CONNECTION = 0x19;
static constexpr uint8_t DID_IO = 0x1A;

// Drive Command IDs from drive_enums.py CommandsEnum
static constexpr uint8_t CID_RAW_MOTORS = 0x01;
static constexpr uint8_t CID_RESET_YAW = 0x06;
static constexpr uint8_t CID_DRIVE_WITH_HEADING = 0x07;
static constexpr uint8_t CID_DRIVE_TANK_NORMALIZED = 0x33;
static constexpr uint8_t CID_DRIVE_RC_SI_UNITS = 0x34;
static constexpr uint8_t CID_DRIVE_RC_NORMALIZED = 0x35;
static constexpr uint8_t CID_DRIVE_STOP = 0x42;

// Sensor Command IDs from sensor_enums.py CommandsEnum
static constexpr uint8_t CID_CONFIGURE_STREAMING_SERVICE = 0x39;
static constexpr uint8_t CID_START_STREAMING_SERVICE = 0x3A;
static constexpr uint8_t CID_STOP_STREAMING_SERVICE = 0x3B;
static constexpr uint8_t CID_CLEAR_STREAMING_SERVICE = 0x3C;
static constexpr uint8_t CID_STREAMING_SERVICE_DATA_NOTIFY = 0x3D;
static constexpr uint8_t CID_GET_ENCODER_COUNTS = 0x53;

// API Command IDs from api_and_shell_enums.py
static constexpr uint8_t CID_ECHO = 0x00;

// Raw Motor Modes from drive_enums.py RawMotorModesEnum
static constexpr uint8_t MOTOR_MODE_OFF = 0x00;
static constexpr uint8_t MOTOR_MODE_FORWARD = 0x01;
static constexpr uint8_t MOTOR_MODE_REVERSE = 0x02;

// Streaming Service IDs (from sensor_streaming_control.py)
static constexpr uint16_t STREAM_SERVICE_ENCODERS = 0x000B;   // Left/Right encoder ticks
static constexpr uint16_t STREAM_SERVICE_IMU = 0x0001;        // Pitch, Roll, Yaw
static constexpr uint16_t STREAM_SERVICE_VELOCITY = 0x0007;   // X, Y velocity
static constexpr uint16_t STREAM_SERVICE_GYROSCOPE = 0x0004;  // X, Y, Z gyro

// Streaming data sizes from sensor_enums.py StreamingDataSizesEnum
static constexpr uint8_t STREAM_DATA_SIZE_8BIT = 0x00;
static constexpr uint8_t STREAM_DATA_SIZE_16BIT = 0x01;
static constexpr uint8_t STREAM_DATA_SIZE_32BIT = 0x02;

// Streaming slot tokens
static constexpr uint8_t STREAM_SLOT_TOKEN_2 = 0x02;  // ST processor, token 2 (encoders, velocity, speed)

// Target addressing constants
// For UART connections to RVR, target is PORT_UART + NODE (usually ST=0x02)
static constexpr uint8_t PORT_INTERNAL = 0x00;  // Internal port (within device)
static constexpr uint8_t PORT_UART = 0x02;      // UART port ID for RVR
static constexpr uint8_t NODE_WILDCARD = 0x00;  // Wildcard node (any node can handle)
static constexpr uint8_t NODE_NORDIC = 0x01;    // Nordic BLE processor
static constexpr uint8_t NODE_ST = 0x02;        // ST microcontroller (drive/sensors)

// Helper to pack port+node into address byte (port in upper nibble, node in lower)
inline uint8_t make_address(uint8_t port, uint8_t node)
{
  return ((port & 0x0F) << 4) | (node & 0x0F);
}

struct RvrFrame
{
  std::vector<uint8_t> bytes;
};

// Checksum helper (computed over header+payload, excluding SOP/EOP)
// Checksum = ~(sum of all bytes) & 0xFF
inline uint8_t checksum8(const std::vector<uint8_t>& data)
{
  uint32_t sum = 0;
  for (uint8_t b : data)
    sum += b;
  return static_cast<uint8_t>(~(sum & 0xFF));
}

class RvrProtocol
{
public:
  // Helper: big-endian appenders
  static void append_u16_be(std::vector<uint8_t>& buf, uint16_t v)
  {
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
  }

  static void append_f32_be(std::vector<uint8_t>& buf, float v)
  {
    static_assert(sizeof(float) == 4, "Unexpected float size");
    uint32_t u;
    std::memcpy(&u, &v, sizeof(float));
    buf.push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
    buf.push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(u & 0xFF));
  }

  // Encode payload using SLIP-like escaping for SOP/EOP/ESC as per Sphero API
  static void encode_payload(std::vector<uint8_t>& payload)
  {
    // SLIP-like encoding: replace ESC, SOP, EOP with two-byte escape sequences
    // ESC -> ESC + ESC_ESC (0xAB 0x23)
    // SOP -> ESC + ESC_SOP (0xAB 0x05)
    // EOP -> ESC + ESC_EOP (0xAB 0x50)
    std::vector<uint8_t> encoded;
    encoded.reserve(payload.size() * 2);  // worst case: all bytes need escaping

    for (uint8_t byte : payload)
    {
      if (byte == ESC)
      {
        encoded.push_back(ESC);
        encoded.push_back(ESC_ESC);
      }
      else if (byte == SOP)
      {
        encoded.push_back(ESC);
        encoded.push_back(ESC_SOP);
      }
      else if (byte == EOP)
      {
        encoded.push_back(ESC);
        encoded.push_back(ESC_EOP);
      }
      else
      {
        encoded.push_back(byte);
      }
    }

    payload = std::move(encoded);
  }

  // Build a raw motors command (duty cycle 0-255, mode: off/forward/reverse)
  static RvrFrame build_raw_motors(uint8_t seq, uint8_t left_mode, uint8_t left_duty, uint8_t right_mode,
                                   uint8_t right_duty)
  {
    // Build payload: 4 uint8_t values (left_mode, left_duty, right_mode, right_duty)
    std::vector<uint8_t> payload;
    payload.reserve(4);
    payload.push_back(left_mode);
    payload.push_back(left_duty);
    payload.push_back(right_mode);
    payload.push_back(right_duty);

    // Encode payload to escape SOP/EOP/ESC if required
    std::vector<uint8_t> enc_payload = payload;
    encode_payload(enc_payload);

    // Build header
    uint8_t flags = FLAG_HAS_TARGET;  // Target ST node for drive commands
    uint8_t target = make_address(PORT_UART, NODE_ST);

    std::vector<uint8_t> header;
    header.push_back(flags);
    header.push_back(target);
    header.push_back(DID_DRIVE);
    header.push_back(CID_RAW_MOTORS);
    header.push_back(seq);

    // Compute checksum over header + raw (un-encoded) payload
    std::vector<uint8_t> chk_input = header;
    chk_input.insert(chk_input.end(), payload.begin(), payload.end());
    uint8_t chk = checksum8(chk_input);

    // Assemble full packet: SOP + header + encoded payload + CHK + EOP
    std::vector<uint8_t> pkt;
    pkt.reserve(1 + header.size() + enc_payload.size() + 1 + 1);
    pkt.push_back(SOP);
    pkt.insert(pkt.end(), header.begin(), header.end());
    pkt.insert(pkt.end(), enc_payload.begin(), enc_payload.end());
    pkt.push_back(chk);
    pkt.push_back(EOP);

    return RvrFrame{ pkt };
  }

  // Build a stop command
  static RvrFrame build_drive_stop(uint8_t seq)
  {
    // Stop command has no payload
    uint8_t flags = FLAG_HAS_TARGET;
    uint8_t target = make_address(PORT_UART, NODE_ST);

    std::vector<uint8_t> header;
    header.push_back(flags);
    header.push_back(target);
    header.push_back(DID_DRIVE);
    header.push_back(CID_DRIVE_STOP);
    header.push_back(seq);

    // Compute checksum over header (no payload)
    uint8_t chk = checksum8(header);

    // Assemble packet: SOP + header + CHK + EOP
    std::vector<uint8_t> pkt;
    pkt.reserve(1 + header.size() + 1 + 1);
    pkt.push_back(SOP);
    pkt.insert(pkt.end(), header.begin(), header.end());
    pkt.push_back(chk);
    pkt.push_back(EOP);

    return RvrFrame{ pkt };
  }

  // Build get encoder counts command
  static RvrFrame build_get_encoder_counts(uint8_t seq)
  {
    // Get encoder counts command has no payload but requests a response
    uint8_t flags = FLAG_HAS_TARGET | FLAG_REQUESTS_RESPONSE;
    uint8_t target = make_address(PORT_UART, NODE_ST);

    std::vector<uint8_t> header;
    header.push_back(flags);
    header.push_back(target);
    header.push_back(DID_SENSOR);
    header.push_back(CID_GET_ENCODER_COUNTS);
    header.push_back(seq);

    // Compute checksum over header (no payload)
    uint8_t chk = checksum8(header);

    // Assemble packet: SOP + header + CHK + EOP
    std::vector<uint8_t> pkt;
    pkt.reserve(1 + header.size() + 1 + 1);
    pkt.push_back(SOP);
    pkt.insert(pkt.end(), header.begin(), header.end());
    pkt.push_back(chk);
    pkt.push_back(EOP);

    return RvrFrame{ pkt };
  }
};

}  // namespace sphero_rvr_control
