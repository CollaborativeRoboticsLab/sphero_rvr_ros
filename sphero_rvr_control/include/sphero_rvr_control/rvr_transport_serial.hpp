#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sphero_rvr_control
{

class RvrTransportSerial
{
public:
  RvrTransportSerial();
  ~RvrTransportSerial();

  // Open and configure the serial port
  bool open(const std::string& port, int baudrate);
  void close();
  bool is_open() const;

  // Write all bytes, returns true on success
  bool write_bytes(const std::vector<uint8_t>& data);

  // Read up to max_len bytes with a timeout in milliseconds; returns number of bytes read
  int read_bytes(uint8_t* buffer, int max_len, int timeout_ms);

private:
  int fd_;
};

}  // namespace sphero_rvr_control
