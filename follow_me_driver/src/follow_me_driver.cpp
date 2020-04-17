#include <iostream>
#include "follow_me_driver/follow_me_driver.hpp"
#include "follow_me_driver/helper_lib.hpp"

namespace terabee {

FollowMeMasterBeacon::FollowMeMasterBeacon(std::shared_ptr<serial_communication::ISerial> serialIf):
  serial_port_(serialIf)
{
}

FollowMeMasterBeacon::~FollowMeMasterBeacon()
{
}

bool FollowMeMasterBeacon::process(PolarPoint2D &point)
{
  if (binary_mode_)
  {
    return processFrameBinary(point);
  }
  else
  {
    return processFrameText(point);
  }
}

bool FollowMeMasterBeacon::printoutModeText()
{
  if (!writeCommandCheckACK(CMD_PRINTOUT_TEXT))
  {
    std::cerr << "Failed: CMD_PRINTOUT_TEXT" << std::endl;
    return false;
  }

  binary_mode_ = false;
  std::cout << "Output mode: Text" << std::endl;
  return true;
}

bool FollowMeMasterBeacon::printoutModeBin()
{
  if (!writeCommandCheckACK(CMD_PRINTOUT_BINARY))
  {
    std::cerr << "Failed: CMD_PRINTOUT_BINARY" << std::endl;
    return false;
  }

  binary_mode_ = true;
  std::cout << "Output mode: Binary" << std::endl;
  return true;
}

bool FollowMeMasterBeacon::spanAutoCalibrate() const
{
  if (!writeCommandCheckACK(CMD_SPAN_AUTOCALIBRATE))
  {
    std::cerr << "Failed: CMD_SPAN_AUTOCALIBRATE" << std::endl;
    return false;
  }
  std::cout << "Span autocalibration executed" << std::endl;
  return true;
}

bool FollowMeMasterBeacon::swapBeacons(bool is_swapped) const
{
  std::array<uint8_t, 4> write_buffer_ = { 0x00, 0x31, is_swapped, 0x00 };
  write_buffer_.at(3) = helper_lib::crc8(write_buffer_.data(), 3);
  if (!writeCommandCheckACK(write_buffer_.data()))
  {
    std::cout << "Failed: Swap Beacons" << std::endl;
    return false;
  }
  std::cout << "Beacons swapped: " << static_cast<int>(is_swapped) << std::endl;
  return true;
}

bool FollowMeMasterBeacon::setBeaconsSpan(uint16_t span_mm) const
{
  uint8_t span_mm_hi = span_mm >> 8;
  uint8_t span_mm_lo = span_mm & 0x00FF;

  std::array<uint8_t, 5> write_buffer_ = { 0x00, 0x22, span_mm_hi, span_mm_lo, 0x00 };
  write_buffer_.at(4) = helper_lib::crc8(write_buffer_.data(), 4);
  if (!writeCommandCheckACK(write_buffer_.data()))
  {
    std::cerr << "Failed: Set beacons span" << std::endl;
    return false;
  }
  std::cout << "Beacons span: " << static_cast<int>(span_mm) << std::endl;
  return true;
}

bool FollowMeMasterBeacon::setEMAWindow(uint8_t ema_window) const
{
  std::array<uint8_t, 5> write_buffer_ = {0x00, 0x52, 0x01, ema_window, 0x00};
  write_buffer_.at(4) = helper_lib::crc8(write_buffer_.data(), 4);
  if (!writeCommandCheckACK(write_buffer_.data()))
  {
    std::cerr << "Failed: Set EMA window" << std::endl;
    return false;
  }
  std::cout << "EMA window: " << static_cast<int>(ema_window) << std::endl;
  return true;
}

bool FollowMeMasterBeacon::setRS485_Parameters(uint8_t slave_id, uint32_t baudrate, rs485_parity rs485_parity) const
{
  std::array<uint8_t, 8> write_buffer_ = { 0x00, 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
  write_buffer_.at(3) = (baudrate >> 16) & 0xFF;
  write_buffer_.at(4) = (baudrate >> 8) & 0xFF;
  write_buffer_.at(5) = (baudrate) & 0xFF;
  write_buffer_.at(6) = rs485_parity;
  write_buffer_.at(7) = helper_lib::crc8(write_buffer_.data(), 7);

  if (!writeCommandCheckACK(write_buffer_.data()))
  {
    std::cerr << "Failed: Set RS485 parameters" << std::endl;
    return false;
  }

  std::array<uint8_t, 5> write_buffer_slave_id_ = { 0x00, 0x52, 0x03, 0x00, 0x00 };
  write_buffer_slave_id_.at(3) = slave_id;
  write_buffer_slave_id_.at(4) = helper_lib::crc8(write_buffer_slave_id_.data(), 4);

  if (!writeCommandCheckACK(write_buffer_slave_id_.data()))
  {
    std::cerr << "Failed: Set RS485 slave id" << std::endl;
    return false;
  }

  std::cout << "RS485 parameters set." << std::endl;
  return true;
}

std::string FollowMeMasterBeacon::testCommand() const
{
  if (!writeCommand(CMD_TEST))
  {
    std::cerr << "Failed: CMD_TEST" << std::endl;
    return "";
  }
  uint8_t eol;
  serial_port_->read(&eol, 1);
  return serial_port_->readline(SERIAL_READ_TEST_CMD_BUFFER_SIZE);
}

bool FollowMeMasterBeacon::processFrameBinary(PolarPoint2D &point)
{
  size_t bytes_read = serial_port_->read(read_buffer_.data(), read_buffer_.size());

  if (bytes_read <= 0)
  {
    return false;
  }

  if (bytes_read != read_buffer_.size() || read_buffer_[0] != 'F' || read_buffer_[1] != 'M')
  {
    std::cout << "Frame incorrect:" << bytes_read << " bytes; Header: " << read_buffer_[0] << read_buffer_[1] << std::endl;
    return false;
  }

  if (helper_lib::crc8(read_buffer_.data(), read_buffer_.size() - 1) !=
      read_buffer_.back())
  {
    std::cout << "CRC incorrect" << std::endl;
    return false;
  }

  point.distance = MM_TO_M_FACTOR*static_cast<float>((read_buffer_[2] << 8) + read_buffer_[3]);
  int8_t heading_read = static_cast<int8_t>((read_buffer_[4]));
  point.heading = DEG_TO_RAD_FACTOR*static_cast<float>(heading_read);

  return true;
}

bool FollowMeMasterBeacon::processFrameText(PolarPoint2D &point)
{
  std::string data = serial_port_->readline(SERIAL_READ_TEXT_BUFFER_SIZE, '\n');
  if (data.empty())
  {
    return false;
  }

  size_t distance_pos = data.find('\t') + 1;
  size_t heading_pos = data.find('\t', distance_pos) + 1;

  if (data.substr(0, distance_pos) != "FM\t")
  {
    return false;
  }

  point.distance = MM_TO_M_FACTOR*std::stof(data.substr(distance_pos, heading_pos));
  point.heading = DEG_TO_RAD_FACTOR*std::stof(data.substr(heading_pos));

  return true;
}

bool FollowMeMasterBeacon::writeCommandCheckACK(const uint8_t frame[]) const
{
  if (!writeCommand(frame))
  {
    return false;
  }
  if (!readACK(frame))
  {
    return false;
  }
  return true;
}

bool FollowMeMasterBeacon::writeCommand(const uint8_t frame[]) const
{
  if(!serial_port_->write(frame, calculateCommandLength(frame)))
  {
    std::cerr << "Timeout or error while writing to serial" << std::endl;
    return false;
  }
  return true;
}

bool FollowMeMasterBeacon::readACK(const uint8_t frame[]) const
{
  std::array<uint8_t, 4> ack_buffer = { 0, 0, 0, 0};
  int bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());
  if (bytes_read == ack_buffer.size() &&
      ack_buffer[0] == frame[0] &&
      ack_buffer[1] == (frame[1] >> 4) &&
      ack_buffer[2] == 0x00 &&
      ack_buffer[3] == helper_lib::crc8(ack_buffer.data(), ack_buffer.size() - 1))
  {
    return true;
  }

  std::cerr << "ACK Error" << std::endl;
  return false;
}

FollowMeRemoteControl::FollowMeRemoteControl(std::shared_ptr<serial_communication::ISerial> serialIf):
  serial_port_(serialIf)
{
}

FollowMeRemoteControl::~FollowMeRemoteControl()
{
}

bool FollowMeRemoteControl::setButtonMode(button_mode mode)
{
  std::array<uint8_t, 3> remote_write = { 0x08, 0x00, 0x00 };
  bool success = false;

  switch (mode)
  {
  case toggle:
    remote_write.at(1) = 0x01;
    remote_write.at(2) = 0xaf;
    success = writeCommandCheckACK(remote_write.data(), remote_write.size());
    break;
  case hold:
    remote_write.at(1) = 0x00;
    remote_write.at(2) = 0xa8;
    success = writeCommandCheckACK(remote_write.data(), remote_write.size());
    break;
  default:
    std::cerr << "Unsupported remote control button mode" << std::endl;
    success = false;
    break;
  }

  if (success)
  {
    std::cout << "Button mode changed to " << (mode == toggle ? "toggle" : "hold") << std::endl;
  }
  return success;
}

bool FollowMeRemoteControl::setBuzzer(bool enabled)
{
  std::array<uint8_t, 3> remote_write = { 0x09, 0x00, 0x00 };
  if (enabled)
  {
    remote_write.at(1) = 0x01;
    remote_write.at(2) = 0xba;
  }
  else
  {
    remote_write.at(1) = 0x00;
    remote_write.at(2) = 0xbd;
  }

  bool success = writeCommandCheckACK(remote_write.data(), remote_write.size());

  if (success)
  {
    std::cout << "Buzzer " << (enabled ? "activated" : "deactivated") << std::endl;
  }
  return success;
}

bool FollowMeRemoteControl::retrieveRemoteParameters(FollowMeRemoteControl::button_mode &mode, bool &buzzer) const
{
  std::array<uint8_t, 3> remote_write = { 0x04, 0x00, 0x54 };
  std::array<uint8_t, 3> ack_buffer = {};

  if (!serial_port_->write(remote_write.data(), remote_write.size()))
  {
    std::cerr << "Timeout or error while writing to serial" << std::endl;
    return false;
  }

  size_t bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());
  if (bytes_read == ack_buffer.size() &&
      ack_buffer.at(0) == 0x40 &&
      ack_buffer.at(2) == terabee::helper_lib::crc8(ack_buffer.data(), ack_buffer.size() - 1))
  {
    buzzer = ack_buffer.at(1) & 0x0F;
    mode = static_cast<FollowMeRemoteControl::button_mode>(ack_buffer.at(1) >> 4);
    return true;
  }
  return false;
}

bool FollowMeRemoteControl::writeCommandCheckACK(const uint8_t frame[], size_t size)
{
  if (!serial_port_->write(frame, size))
  {
    std::cerr << "Timeout or error while writing to serial" << std::endl;
    return false;
  }

  if (!readACK())
  {
    return false;
  }

  return true;
}

bool FollowMeRemoteControl::readACK() const
{
  std::array<uint8_t, 6> ack_buffer;
  size_t bytes_read = serial_port_->read(ack_buffer.data(), ack_buffer.size());

  if (bytes_read < ack_buffer.size())
  {
    std::cerr << "ACK Error: Not enough bytes read" << std::endl;
    return false;
  }

  for (uint8_t el : ack_buffer)
  {
    if (el != 0x00)
    {
      std::cerr << "ACK Error: NACK" << std::endl;
      return false;
    }
  }

  return true;
}

}
