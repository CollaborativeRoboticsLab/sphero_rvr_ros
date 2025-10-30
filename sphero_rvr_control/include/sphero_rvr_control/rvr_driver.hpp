#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <sphero_rvr_control/rvr_transport_serial.hpp>
#include <sphero_rvr_control/rvr_protocol.hpp>

namespace sphero_rvr_control
{

class RvrDriver
{
public:
  RvrDriver();
  ~RvrDriver();

  bool connect(const std::string& port, int baudrate);
  void disconnect();
  bool is_connected() const;

  // Set wheel duty cycles using raw motors command
  // mode: 0=off, 1=forward, 2=reverse
  // duty: 0-255 (normalized PWM duty cycle)
  bool set_raw_motors(uint8_t left_mode, uint8_t left_duty, uint8_t right_mode, uint8_t right_duty);

  // Stop all motors
  bool stop();

  // Get encoder counts directly from hardware
  bool get_encoder_counts(int32_t& left_ticks, int32_t& right_ticks);

private:
  // Parse encoder response frame
  bool parse_encoder_response(const std::vector<uint8_t>& frame, int32_t& left_ticks, int32_t& right_ticks);

  // Extract int32_t from big-endian bytes
  static int32_t extract_i32_be(const uint8_t* data)
  {
    return (static_cast<int32_t>(data[0]) << 24) | (static_cast<int32_t>(data[1]) << 16) |
           (static_cast<int32_t>(data[2]) << 8) | static_cast<int32_t>(data[3]);
  }

  RvrTransportSerial transport_;
  uint8_t seq_;
  std::mutex mtx_;

  // Frame accumulation buffer
  std::vector<uint8_t> frame_buffer_;
  bool in_frame_{ false };
  bool escape_next_{ false };
};

}  // namespace sphero_rvr_control
