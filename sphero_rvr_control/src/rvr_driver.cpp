#include "sphero_rvr_control/rvr_driver.hpp"

#include <vector>
#include <cstring>

namespace sphero_rvr_control
{

RvrDriver::RvrDriver() : seq_(0)
{
}
RvrDriver::~RvrDriver()
{
  disconnect();
}

bool RvrDriver::connect(const std::string& port, int baudrate)
{
  std::lock_guard<std::mutex> lock(mtx_);
  return transport_.open(port, baudrate);
}

void RvrDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mtx_);
  transport_.close();
}

bool RvrDriver::is_connected() const
{
  return transport_.is_open();
}

bool RvrDriver::set_raw_motors(uint8_t left_mode, uint8_t left_duty, uint8_t right_mode, uint8_t right_duty)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!transport_.is_open())
    return false;
  auto frame = RvrProtocol::build_raw_motors(seq_++, left_mode, left_duty, right_mode, right_duty);
  return transport_.write_bytes(frame.bytes);
}

bool RvrDriver::stop()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!transport_.is_open())
    return false;
  auto frame = RvrProtocol::build_drive_stop(seq_++);
  return transport_.write_bytes(frame.bytes);
}

bool RvrDriver::get_encoder_counts(int32_t& left_ticks, int32_t& right_ticks)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!transport_.is_open())
    return false;

  // Send get encoder counts command
  auto frame = RvrProtocol::build_get_encoder_counts(seq_++);
  if (!transport_.write_bytes(frame.bytes))
    return false;

  // Wait for response (poll with timeout)
  const int max_attempts = 20;  // 20 * 5ms = 100ms timeout
  for (int attempt = 0; attempt < max_attempts; ++attempt)
  {
    uint8_t buf[256];
    int n = transport_.read_bytes(buf, sizeof(buf), 5);
    if (n <= 0)
      continue;

    // Process response frames
    for (int i = 0; i < n; ++i)
    {
      uint8_t byte = buf[i];

      if (byte == SOP && !in_frame_)
      {
        frame_buffer_.clear();
        in_frame_ = true;
        escape_next_ = false;
        continue;
      }

      if (!in_frame_)
        continue;

      if (byte == EOP && !escape_next_)
      {
        if (!frame_buffer_.empty())
        {
          // Try to parse encoder response
          if (parse_encoder_response(frame_buffer_, left_ticks, right_ticks))
          {
            in_frame_ = false;
            frame_buffer_.clear();
            return true;
          }
        }
        in_frame_ = false;
        frame_buffer_.clear();
        escape_next_ = false;
        continue;
      }

      if (byte == ESC && !escape_next_)
      {
        escape_next_ = true;
        continue;
      }

      if (escape_next_)
      {
        escape_next_ = false;
        if (byte == ESC_ESC)
          frame_buffer_.push_back(ESC);
        else if (byte == ESC_SOP)
          frame_buffer_.push_back(SOP);
        else if (byte == ESC_EOP)
          frame_buffer_.push_back(EOP);
        else
        {
          in_frame_ = false;
          frame_buffer_.clear();
        }
        continue;
      }

      frame_buffer_.push_back(byte);
    }
  }

  return false;  // Timeout
}

bool RvrDriver::parse_encoder_response(const std::vector<uint8_t>& frame, int32_t& left_ticks, int32_t& right_ticks)
{
  // Frame format: FLAGS + [TARGET] + [SOURCE] + DID + CID + SEQ + [ERR] + PAYLOAD + CHK
  if (frame.size() < 13)
    return false;  // Minimum size for encoder response

  // Validate checksum
  std::vector<uint8_t> data_for_checksum(frame.begin(), frame.end() - 1);
  uint8_t computed_chk = 0;
  for (uint8_t b : data_for_checksum)
    computed_chk += b;
  computed_chk = ~computed_chk;

  if (computed_chk != frame.back())
    return false;

  // Parse header
  size_t idx = 0;
  uint8_t flags = frame[idx++];

  if (!(flags & FLAG_IS_RESPONSE))
    return false;  // Not a response

  if (flags & FLAG_HAS_TARGET)
    idx++;
  if (flags & FLAG_HAS_SOURCE)
    idx++;

  if (idx + 3 >= frame.size())
    return false;
  uint8_t did = frame[idx++];
  uint8_t cid = frame[idx++];
  idx++;  // seq

  // Check if this is an encoder response
  if (did != DID_SENSOR || cid != CID_GET_ENCODER_COUNTS)
    return false;

  idx++;  // error code (should be 0 for success)

  // Payload should be 8 bytes (two int32_t values)
  size_t payload_len = frame.size() - idx - 1;  // -1 for checksum
  if (payload_len < 8)
    return false;

  // Extract encoders (big-endian int32_t)
  left_ticks = extract_i32_be(&frame[idx]);
  right_ticks = extract_i32_be(&frame[idx + 4]);

  return true;
}

}  // namespace sphero_rvr_control
