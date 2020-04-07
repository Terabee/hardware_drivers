#include "serial_communication/serial_windows.hpp"

#include <windows.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace terabee
{
namespace serial_communication
{

SerialWindows::SerialWindows(const std::string &port):
  is_open_(false),
  portname_(port),
  baudrate_(CBR_9600),
  parity_(NOPARITY),
  stop_bits_(ONESTOPBIT),
  byte_size_(DATABITS_8),
  timeout_(0),
  blocking_(false),
  serial_handle_()
{
  setPortName(portname_);
  setBaudrate(9600);
  setParity(parity_none);
  setBytesize(eightbits);
  setStopbits(stopbits_one);
  setFlowcontrol(flowcontrol_none);
  // TODO: some kind of unified Logger instead of cout, maybe the one I have?
  std::cout << "SerialWindows::SerialWindows(" << portname_ << ")" << std::endl;
}

SerialWindows::~SerialWindows()
{
  std::cout << "SerialWindows::~SerialWindows(" << portname_ << ")" << std::endl;
  close();
}

bool SerialWindows::open()
{
  if (isOpen())
  {
    std::cout << "Cannot open port. Already open." << std::endl;
    return false;
  }

  std::string port_path = "\\\\.\\" + portname_;
  std::cout << "Port Name: " << port_path << std::endl;
  serial_handle_ = CreateFile(port_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  if (serial_handle_ == INVALID_HANDLE_VALUE)
  {
    std::cout << "Port cannot be opened: INVALID_HANDLE_VALUE" << std::endl;
    return false;
  }

  DCB serialParams = { 0 };
  serialParams.DCBlength = sizeof(serialParams);
  if (GetCommState(serial_handle_, &serialParams) == FALSE)
  {
    std::cout << "Error in GetCommmState" << std::endl;
    return false;
  }

  serialParams.BaudRate = baudrate_;
  serialParams.ByteSize = byte_size_;
  serialParams.StopBits = stop_bits_;
  serialParams.Parity = parity_;
  if (SetCommState(serial_handle_, &serialParams) == FALSE)
  {
    std::cout << "Error in SetCommState" << std::endl;
    return false;
  }

  COMMTIMEOUTS timeout = { 0 };
  timeout.ReadIntervalTimeout = 50;
  timeout.ReadTotalTimeoutConstant = timeout_.count();
  timeout.ReadTotalTimeoutMultiplier = 50;
  timeout.WriteTotalTimeoutConstant = 50;
  timeout.WriteTotalTimeoutMultiplier = 10;
  if (SetCommTimeouts(serial_handle_, &timeout) == FALSE)
  {
    std::cout << "Error in SetCommTimeouts" << std::endl;
    return false;
  }
  is_open_ = true;
  return is_open_;
}

bool SerialWindows::close()
{
  if (!isOpen())
  {
    std::cout << "Cannot close port. Already closed." << std::endl;
    return false;
  }

  if (CloseHandle(serial_handle_) == FALSE)
  {
      std::cout << "Error in CloseHandle" << std::endl;
      return false;
  }
  is_open_ = false;
  return true;
}

bool SerialWindows::clear()
{
  return PurgeComm(serial_handle_, PURGE_RXABORT|PURGE_TXABORT|PURGE_RXCLEAR|PURGE_TXCLEAR);
}

bool SerialWindows::isOpen()
{
  return is_open_;
}

size_t SerialWindows::available()
{
  std::cout << "available() Not supported!" << std::endl;
  return 0;
}

bool SerialWindows::setPortName(std::string portname)
{
  if (isOpen())
  {
    std::cout << "Cannot change port name on the open port" << std::endl;
    return false;
  }
  portname_ = portname;
  return true;
}

bool SerialWindows::setBaudrate(uint32_t baudrate)
{
  if(isOpen())
  {
    std::cout << "Cannot change settings on the open port" << std::endl;
    return false;
  }

  switch (baudrate)
  {
  case 1200:
    baudrate_ = CBR_1200;
    break;
  case 2400:
    baudrate_ = CBR_2400;
    break;
  case 4800:
    baudrate_ = CBR_4800;
    break;
  case 9600:
    baudrate_ = CBR_9600;
    break;
  case 19200:
    baudrate_ = CBR_19200;
    break;
  case 38400:
    baudrate_ = CBR_38400;
    break;
  case 57600:
    baudrate_ = CBR_57600;
    break;
  case 115200:
    baudrate_ = CBR_115200;
    break;
  default:
    std::cout << "Unsupported baudrate: " << baudrate << std::endl;
    return false;
  }

  return true;
}

bool SerialWindows::setParity(parity_t iparity)
{
  if(isOpen())
  {
    std::cout << "Cannot change settings on the open port" << std::endl;
    return false;
  }

  switch (iparity)
  {
  case parity_none:
    parity_ = NOPARITY;
    break;
  case parity_odd:
    parity_ = ODDPARITY;
    break;
  case parity_even:
    parity_ = EVENPARITY;
    break;
  case parity_mark:
    parity_ = MARKPARITY;
    break;
  case parity_space:
    parity_ = SPACEPARITY;
    break;
  default:
    std::cout << "Unsupported parity: " << iparity << std::endl;
    return false;
  }

  return true;
}

bool SerialWindows::setBytesize(bytesize_t ibytesize)
{
  if(isOpen())
  {
    std::cout << "Cannot change settings on the open port" << std::endl;
    return false;
  }

  switch (ibytesize)
  {
  case fivebits:
    byte_size_ = DATABITS_5;
    break;
  case sixbits:
    byte_size_ = DATABITS_6;
    break;
  case sevenbits:
    byte_size_ = DATABITS_7;
    break;
  case eightbits:
    byte_size_ = DATABITS_8;
    break;
  default:
    std::cout << "Unsupported bytesize: " << ibytesize << std::endl;
    return false;
  }

  return true;
}

bool SerialWindows::setStopbits(stopbits_t istopbits)
{
  if(isOpen())
  {
    std::cout << "Cannot change settings on the open port" << std::endl;
    return false;
  }

  switch (istopbits)
  {
  case stopbits_one:
    stop_bits_ = ONESTOPBIT;
    break;
  case stopbits_two:
    stop_bits_ = TWOSTOPBITS;
    break;
  case stopbits_one_point_five:
    stop_bits_ = ONE5STOPBITS;
    break;
  default:
    std::cout << "Unsupported bytesize: " << istopbits << std::endl;
    return false;
  }

  return true;
}

bool SerialWindows::setFlowcontrol(flowcontrol_t iflowcontrol)
{
  std::cout << "NOT IMPLEMENTED!\n";
  return true;
}

bool SerialWindows::setTimeout(std::chrono::milliseconds timeout)
{
  if(isOpen())
  {
    std::cout << "Cannot change settings on the open port" << std::endl;
    return false;
  }
  timeout_ = timeout;
  if (timeout_.count() > 0)
  {
    blocking_ = true;
  }
  return true;
}

std::string SerialWindows::readline(size_t max_buffer_size, char eol = '\n')
{
  if (!isOpen())
  {
      std::cout << "Cannot read because not open" << std::endl;
      return "";
  }
  std::string result;
  bool retval(false);
  char c(0);
  for (size_t i = 0; i < max_buffer_size; i++)
  {
    DWORD nRead;
    retval = ReadFile(serial_handle_, &c, 1, &nRead, NULL);
    if (!retval && GetLastError() != ERROR_IO_PENDING)
    {
        std::cout << "Failed to read: " << std::strerror(errno) << std::endl;
        return "";
    }
    if (c == eol) break;
    result += c;
  }
  return result;
}

bool SerialWindows::flushInput()
{
  throw std::logic_error("Not implemented: SerialWindows::flushInput");
}

bool SerialWindows::flushOutput()
{
  throw std::logic_error("Not implemented: SerialWindows::flushOutput");
}

size_t SerialWindows::write(const uint8_t *data, size_t size)
{
  if (!isOpen())
  {
      std::cout << "Cannot write because not open" << std::endl;
      return 0;
  }
  DWORD numBytesSent;
  if (WriteFile(serial_handle_, data, size, &numBytesSent, NULL) == FALSE)
  {
    std::cout << "Error in WriteFile" << std::endl;
    return 0;
  }
  if (numBytesSent <= 0)
  {
      std::cout << "Failed to write: " << std::strerror(errno);
  }
  return (numBytesSent >= 0) ? numBytesSent : 0;
}

size_t SerialWindows::read(uint8_t *buffer, size_t size)
{
  if (!isOpen())
  {
      std::cout << "Cannot read because not open" << std::endl;
      return 0;
  }
  DWORD nRead;
  if (ReadFile(serial_handle_, buffer, size, &nRead, NULL) == FALSE)
  {
    std::cout << "Error in ReadFile" << std::endl;
  }
  return nRead;
}

}  // namespace serial_communication
}  // namespace terabee
