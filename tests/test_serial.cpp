#include <sys/stat.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <future>
#include <string>

#ifdef __linux__

#include "serial_communication/serial.hpp"
using SerialInterface = terabee::serial_communication::Serial;

#elif defined(__MINGW32__) || defined(__MINGW64__) || defined(_WIN32) || defined(_WIN64)

#include "serial_communication/serial_windows.hpp"
using SerialInterface = terabee::serial_communication::SerialWindows;

#endif

using terabee::serial_communication::ISerial;

/*
 * Linux null modem emulator is required to test some functionalities of
 * this SerialCommunication. If the appropriate devices are not found in the
 * system, some tests will be aborted and just return success.
 *
 * https://github.com/freemed/tty0tty
 */
namespace {
bool fileExists(const std::string& name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}
}  // unnamed namespace

class SerialFixture: public testing::Test {
protected:
  SerialFixture():
    device0Filename_("/dev/tnt0"),
    device1Filename_("/dev/tnt1")
  {
    sut0_.reset(new SerialInterface(device0Filename_));
    sut1_.reset(new SerialInterface(device1Filename_));
  }
  void SetUp() override
  {
    ASSERT_TRUE(fileExists(device0Filename_));
    ASSERT_TRUE(fileExists(device1Filename_));
  }
    std::string device0Filename_;
    std::string device1Filename_;
    std::unique_ptr<ISerial> sut0_;
    std::unique_ptr<ISerial> sut1_;
};

TEST_F(SerialFixture, returnFalseIfOpenedOrClosedTwice)
{
  ASSERT_TRUE(sut0_->open());
  ASSERT_FALSE(sut0_->open());
  ASSERT_TRUE(sut0_->isOpen());
  ASSERT_TRUE(sut0_->close());
  ASSERT_FALSE(sut0_->close());
  ASSERT_FALSE(sut0_->isOpen());

  ASSERT_TRUE(sut0_->open());
  ASSERT_FALSE(sut0_->open());
  ASSERT_TRUE(sut0_->isOpen());
  ASSERT_TRUE(sut0_->close());
  ASSERT_FALSE(sut0_->close());
  ASSERT_FALSE(sut0_->isOpen());
}

TEST_F(SerialFixture, returnFalseWhenFailToOpenDevice)
{
  sut0_.reset(new SerialInterface("garbage device"));
  ASSERT_FALSE(sut0_->open());
}

TEST_F(SerialFixture, returnNumberOfBytesAvailableForRead)
{
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  size_t num_bytes = sizeof(data);
  ASSERT_TRUE(sut0_->setBaudrate(115200));
  ASSERT_TRUE(sut1_->setBaudrate(115200));
  ASSERT_TRUE(sut0_->open());
  ASSERT_TRUE(sut1_->open());
  ASSERT_EQ(num_bytes, sut0_->write(data, num_bytes));
  // hardcoded sleep to make sure transmitted data is available
  // TODO: I don't like this solution, because it might fail with heavy CPU load
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_EQ(num_bytes, sut1_->available());
}

TEST_F(SerialFixture, returnTrueWhenSetBaudrateParityBytesizeStopbitsFlowcontrolTimeoutBeforeOpen)
{
  ASSERT_TRUE(sut0_->setBaudrate(115200));
  ASSERT_TRUE(sut0_->setParity(ISerial::parity_none));
  ASSERT_TRUE(sut0_->setBytesize(ISerial::eightbits));
  ASSERT_TRUE(sut0_->setStopbits(ISerial::stopbits_two));
  ASSERT_TRUE(sut0_->setFlowcontrol(ISerial::flowcontrol_hardware));
  ASSERT_TRUE(sut0_->setTimeout(std::chrono::milliseconds(100)));
  ASSERT_TRUE(sut0_->open());
}

TEST_F(SerialFixture, returnFalseWhenSetBaudrateParityBytesizeStopbitsFlowcontrolTimeoutAfterOpen)
{
  ASSERT_TRUE(sut0_->open());
  ASSERT_FALSE(sut0_->setBaudrate(115200));
  ASSERT_FALSE(sut0_->setParity(ISerial::parity_none));
  ASSERT_FALSE(sut0_->setBytesize(ISerial::eightbits));
  ASSERT_FALSE(sut0_->setStopbits(ISerial::stopbits_two));
  ASSERT_FALSE(sut0_->setFlowcontrol(ISerial::flowcontrol_hardware));
  ASSERT_FALSE(sut0_->setTimeout(std::chrono::milliseconds(100)));
}

TEST_F(SerialFixture, readJustOneLineFromBuffer)
{
  uint8_t data[] = {'t', 'e', 's', 't', '\n', 'x', 'x', 'x', '\n'};
  size_t num_bytes = sizeof(data);
  std::string expected_line((char*)data, 4);
  ASSERT_TRUE(sut0_->open());
  ASSERT_TRUE(sut1_->open());
  ASSERT_EQ(num_bytes, sut0_->write(data, num_bytes));
  ASSERT_EQ(expected_line, sut1_->readline());
}

TEST_F(SerialFixture, read0BytesAfterFlush)
{
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  size_t num_bytes = sizeof(data);
  ASSERT_TRUE(sut1_->setTimeout(std::chrono::milliseconds(1)));
  ASSERT_TRUE(sut0_->open());
  ASSERT_TRUE(sut1_->open());
  ASSERT_EQ(num_bytes, sut0_->write(data, num_bytes));
  ASSERT_TRUE(sut1_->flushInput());
  ASSERT_EQ(0, sut1_->read(data, num_bytes));
}

//TODO: not sure how can I test flushOutput

TEST_F(SerialFixture, readWriteCorrectly)
{
  ASSERT_TRUE(sut0_->setBaudrate(115200));
  ASSERT_TRUE(sut0_->setTimeout(std::chrono::milliseconds(100)));
  ASSERT_TRUE(sut0_->setParity(ISerial::parity_even));
  ASSERT_TRUE(sut0_->setStopbits(ISerial::stopbits_two));
  ASSERT_TRUE(sut0_->setFlowcontrol(ISerial::flowcontrol_software));
  ASSERT_TRUE(sut1_->setBaudrate(115200));
  ASSERT_TRUE(sut1_->setTimeout(std::chrono::milliseconds(100)));
  ASSERT_TRUE(sut1_->setParity(ISerial::parity_even));
  ASSERT_TRUE(sut1_->setStopbits(ISerial::stopbits_two));
  ASSERT_TRUE(sut1_->setFlowcontrol(ISerial::flowcontrol_software));
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  size_t num_bytes = sizeof(data);
  uint8_t received_data[5] = {0, 0, 0, 0, 0};
  ASSERT_TRUE(sut0_->open());
  ASSERT_TRUE(sut1_->open());
  ASSERT_EQ(num_bytes, sut0_->write(data, num_bytes));
  ASSERT_EQ(num_bytes, sut1_->read(received_data, num_bytes));
  for (size_t i = 0; i < num_bytes; i++)
  {
    ASSERT_EQ(data[i], received_data[i]);
  }
}

TEST_F(SerialFixture, return0ReadWriteWhenNotOpen)
{
  ASSERT_TRUE(sut0_->setBaudrate(115200));
  ASSERT_TRUE(sut0_->setTimeout(std::chrono::milliseconds(100)));
  ASSERT_TRUE(sut1_->setBaudrate(115200));
  ASSERT_TRUE(sut1_->setTimeout(std::chrono::milliseconds(100)));
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  size_t num_bytes = sizeof(data);
  uint8_t received_data[5] = {0, 0, 0, 0, 0};
  ASSERT_EQ(0, sut0_->write(data, num_bytes));
  ASSERT_EQ(0, sut1_->read(received_data, num_bytes));
}
