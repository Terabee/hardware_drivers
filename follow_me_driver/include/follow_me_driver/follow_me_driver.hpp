#ifndef FOLLOW_ME_DRIVER_H
#define FOLLOW_ME_DRIVER_H

#include <memory>
#include <cmath>
#include "serial_communication/iserial.hpp"

namespace terabee {

struct PolarPoint2D
{
  float distance;
  float heading;
};

class FollowMeMasterBeacon
{
public:
  enum rs485_parity
  {
    rs485_parity_none = 0,
    rs485_parity_odd = 1,
    rs485_parity_even = 2
  };

  FollowMeMasterBeacon() = delete;
  FollowMeMasterBeacon(std::shared_ptr<serial_communication::ISerial> serialIf);
  FollowMeMasterBeacon(const FollowMeMasterBeacon &other) = delete;
  FollowMeMasterBeacon(FollowMeMasterBeacon &&other) = delete;
  ~FollowMeMasterBeacon();

  bool process(PolarPoint2D &point);

  bool printoutModeText();
  bool printoutModeBin();
  bool spanAutoCalibrate() const;
  bool swapBeacons(bool is_swapped) const;
  bool setBeaconsSpan(uint16_t span_mm) const;
  bool setEMAWindow(uint8_t ema_window) const;
  bool setRS485_Parameters(uint8_t slave_id, uint32_t baudrate, rs485_parity parity) const;
  std::string testCommand() const;

  static uint8_t calculateCommandLength(const uint8_t frame[]) { return (frame[1] & 0x0F) + 3; }

private:
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  const float MM_TO_M_FACTOR = 0.001f;
  const float DEG_TO_RAD_FACTOR = M_PI/180.0;
  bool binary_mode_ = true;

  const uint8_t CMD_PRINTOUT_TEXT[4] = { 0x00, 0x11, 0x01, 0x45 };
  const uint8_t CMD_PRINTOUT_BINARY[4] = { 0x00, 0x11, 0x02, 0x4C };
  const uint8_t CMD_SPAN_AUTOCALIBRATE[5] = { 0x00, 0x22, 0x00, 0x00, 0x95 };
  const uint8_t CMD_TEST[4] = { 0x00, 0x00, 0x00, 0x00 };

  static constexpr size_t SERIAL_READ_TEST_CMD_BUFFER_SIZE = 120;
  static constexpr size_t SERIAL_READ_TEXT_BUFFER_SIZE = 20;
  std::array<uint8_t, 6> read_buffer_;

  bool processFrameBinary(PolarPoint2D &point);
  bool processFrameText(PolarPoint2D &point);

  bool writeCommandCheckACK(const uint8_t frame[]) const;
  bool writeCommand(const uint8_t frame[]) const;
  bool readACK(const uint8_t frame[]) const;
};

class FollowMeRemoteControl
{
public:
  enum button_mode
  {
    hold = 0,
    toggle = 1
  };

  FollowMeRemoteControl() = delete;
  FollowMeRemoteControl(std::shared_ptr<serial_communication::ISerial> serialIf);
  FollowMeRemoteControl(const FollowMeRemoteControl &other) = delete;
  FollowMeRemoteControl(FollowMeRemoteControl &&other) = delete;
  ~FollowMeRemoteControl();

  bool setButtonMode(button_mode mode);
  bool setBuzzer(bool enabled);
  bool retrieveRemoteParameters(button_mode &mode, bool &buzzer) const;

private:
  std::shared_ptr<serial_communication::ISerial> serial_port_;

  bool writeCommandCheckACK(const uint8_t frame[], size_t size);
  bool readACK() const;
};

}  // namespace terabee

#endif // FOLLOW_ME_DRIVER_H
