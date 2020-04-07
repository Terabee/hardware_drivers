#ifndef SERIAL_WINDOWS_H
#define SERIAL_WINDOWS_H

#include "serial_communication/iserial.hpp"
#include <windows.h>

namespace terabee
{
namespace serial_communication
{

class SerialWindows : public ISerial
{
public:
  SerialWindows() = delete;
  /**
   * Constructs Serial from given port, e.g. /dev/ttyUSB0
   *
   * with default settings:
   *    - baud 9600
   *    - parity none
   *    - bytesize 8
   *    - 1 stop bit
   *    - no flow control
   *    - timeout == 0 (non-blocking)
   */
  SerialWindows(const std::string &port);
  SerialWindows(const SerialWindows &other) = delete;
  SerialWindows(SerialWindows &&other) = delete;
  ~SerialWindows() override;
  bool open() override;
  bool close() override;
  bool clear() override;
  bool isOpen() override;
  size_t available() override;
  bool setPortName(std::string portname) override;
  bool setBaudrate(uint32_t baudrate) override;
  bool setParity(parity_t iparity) override;
  bool setBytesize(bytesize_t ibytesize) override;
  bool setStopbits(stopbits_t istopbits) override;
  bool setFlowcontrol(flowcontrol_t iflowcontrol) override;
  bool setTimeout(std::chrono::milliseconds timeout) override;
  std::string readline(size_t max_buffer_size, char eol) override;
  bool flushInput() override;
  bool flushOutput() override;
  size_t write(const uint8_t *data, size_t size) override;
  size_t read(uint8_t *buffer, size_t size) override;

private:
  bool is_open_;
  std::string portname_;
  DWORD baudrate_;
  BYTE parity_;
  BYTE stop_bits_;
  BYTE byte_size_;
  std::chrono::milliseconds timeout_;
  bool blocking_;
  HANDLE serial_handle_;
};

}  // namespace serial_communication
}  // namespace terabee

#endif // SERIAL_WINDOWS_H
