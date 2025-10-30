#include <sphero_rvr_control/rvr_transport_serial.hpp>

#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace sphero_rvr_control
{

RvrTransportSerial::RvrTransportSerial() : fd_(-1)
{
}

RvrTransportSerial::~RvrTransportSerial()
{
  close();
}

bool RvrTransportSerial::open(const std::string& port, int baudrate = 115200)
{
  close();
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0)
  {
    close();
    return false;
  }

  cfmakeraw(&tio);

  // Set baud rate
  speed_t speed = B115200;
  switch (baudrate)
  {
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    default:
      speed = B115200;
      break;
  }
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;  // no parity
  tio.c_cflag &= ~CSTOPB;  // 1 stop bit
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;  // 8 data bits

  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    close();
    return false;
  }

  return true;
}

void RvrTransportSerial::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool RvrTransportSerial::is_open() const
{
  return fd_ >= 0;
}

bool RvrTransportSerial::write_bytes(const std::vector<uint8_t>& data)
{
  if (fd_ < 0)
    return false;
  ssize_t total = 0;
  const uint8_t* ptr = data.data();
  ssize_t remaining = static_cast<ssize_t>(data.size());
  while (remaining > 0)
  {
    ssize_t n = ::write(fd_, ptr + total, remaining);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EINTR)
        continue;
      return false;
    }
    total += n;
    remaining -= n;
  }
  return true;
}

int RvrTransportSerial::read_bytes(uint8_t* buffer, int max_len, int timeout_ms)
{
  if (fd_ < 0)
    return -1;
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd_, &rfds);

  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  int rv = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
  if (rv < 0)
    return -1;
  if (rv == 0)
    return 0;  // timeout

  ssize_t n = ::read(fd_, buffer, max_len);
  if (n < 0)
  {
    if (errno == EAGAIN || errno == EINTR)
      return 0;
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace sphero_rvr_control
