#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <future>
#include <cstring>
#include "follow_me_driver/follow_me_driver.hpp"
#include "follow_me_driver/helper_lib.hpp"

#include "MockISerial.hpp"

using testing::_;
using testing::Return;
using testing::Invoke;

namespace terabee {

// TEST Frames
const uint8_t CMD_PRINTOUT_TEXT[4] = { 0x00, 0x11, 0x01, 0x45 };
const uint8_t CMD_PRINTOUT_BINARY[4] = { 0x00, 0x11, 0x02, 0x4C };
const uint8_t CMD_DEACTIVATE_DEBUG[7] = { 0x00, 0x54, 0x0D, 0x00, 0x00, 0x00, 0x8B };
const uint8_t CMD_ACTIVATE_DEBUG[7] = { 0x00, 0x54, 0x0D, 0x01, 0x00, 0x00, 0xE0 };
const uint8_t CMD_AUTOCALIBRATE[5] = { 0x00, 0x22, 0x00, 0x00, 0x95 };
const uint8_t CMD_EMA_WINDOW[5] = { 0x00, 0x52, 0x01, 0x0A, 0xD1 };
const uint8_t FRAME_BINARY[6] = { 'F', 'M', 0x02, 0x04, 0x01, 0x63 };

class FollowMeDriverFixture : public testing::Test
{
protected:
  FollowMeDriverFixture():
    serialInterface_(std::make_shared<serial_communication::MockISerial>())
  {
    ON_CALL(*serialInterface_, open()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, isOpen()).WillByDefault(Return(true));
    ON_CALL(*serialInterface_, available()).WillByDefault(Return(69));
    ON_CALL(*serialInterface_, readline(_, _)).WillByDefault(Return(fakeDataText));
    ON_CALL(*serialInterface_, write(_, _))
        .WillByDefault(Return(1));

    ON_CALL(*serialInterface_, read(_, _))
        .WillByDefault(Invoke(
                         [this](uint8_t* data, size_t size) {
                         std::memcpy(data, fakeDataBin_, 7);
                         return size;
                       }));
    followMe_.reset(new FollowMeMasterBeacon(serialInterface_));
  }

  virtual ~FollowMeDriverFixture() {
    followMe_.reset();
  }

  std::shared_ptr<serial_communication::MockISerial> serialInterface_;
  uint8_t fakeDataBin_[6] = {'F', 'M', 0x02, 0x04, 0xF1, 0xBD};
  std::string fakeDataText = "FM\t2503.5\t79.3\n";

  uint8_t ack_printout_text[4] = { 0x00, 0x01, 0x00, 0x15 };

  std::unique_ptr<FollowMeMasterBeacon> followMe_;

public:

};

TEST(TestCRC8, CorrectFrames)
{
  EXPECT_EQ(helper_lib::crc8(CMD_PRINTOUT_TEXT, 3), 0x45);
  EXPECT_EQ(helper_lib::crc8(CMD_PRINTOUT_BINARY, 3), 0x4C);
  EXPECT_EQ(helper_lib::crc8(CMD_DEACTIVATE_DEBUG, 6), 0x8B);
  EXPECT_EQ(helper_lib::crc8(CMD_ACTIVATE_DEBUG, 6), 0xE0);
  EXPECT_EQ(helper_lib::crc8(CMD_AUTOCALIBRATE, 4), 0x95);
  EXPECT_EQ(helper_lib::crc8(CMD_EMA_WINDOW, 4), 0xD1);
  EXPECT_EQ(helper_lib::crc8(FRAME_BINARY, 5), 0x63);
}

TEST_F(FollowMeDriverFixture, CalcCmdLen)
{
  EXPECT_EQ(4, followMe_->calculateCommandLength(CMD_PRINTOUT_TEXT));
  EXPECT_EQ(4, followMe_->calculateCommandLength(CMD_PRINTOUT_BINARY));
  EXPECT_EQ(7, followMe_->calculateCommandLength(CMD_DEACTIVATE_DEBUG));
  EXPECT_EQ(7, followMe_->calculateCommandLength(CMD_ACTIVATE_DEBUG));
  EXPECT_EQ(5, followMe_->calculateCommandLength(CMD_AUTOCALIBRATE));
  EXPECT_EQ(5, followMe_->calculateCommandLength(CMD_EMA_WINDOW));
}

TEST_F(FollowMeDriverFixture, ProcessFrameText)
{
  ON_CALL(*serialInterface_, read(_,_))
      .WillByDefault(Invoke(
        [this](uint8_t* data, size_t size) {
          std::memcpy(data, ack_printout_text, 4);
          return size;
        }));
  followMe_->printoutModeText();

  PolarPoint2D ref_point = { 2.5035f, 1.38404f };
  PolarPoint2D measured_point;

  followMe_->process(measured_point);

  EXPECT_NEAR(ref_point.distance, measured_point.distance, 1e-4);
  EXPECT_NEAR(ref_point.heading, measured_point.heading, 1e-4);
}

TEST_F(FollowMeDriverFixture, ProcessFrameBin)
{
  ON_CALL(*serialInterface_, read(_,_))
      .WillByDefault(Invoke(
                      [this](uint8_t* data, size_t size) {
                        std::memcpy(data, ack_printout_text, 4);
                        return size;
                      }));
  followMe_->printoutModeBin();

  PolarPoint2D ref_point = { 0.516f, -0.26179f };
  PolarPoint2D measured_point;

  ON_CALL(*serialInterface_, available()).WillByDefault(Return(7));
  ON_CALL(*serialInterface_, read(_, _))
      .WillByDefault(Invoke(
                       [this](uint8_t* data, size_t size) {
                       std::memcpy(data, fakeDataBin_, 7);
                       return size;
                     }));
  followMe_->process(measured_point);

  EXPECT_NEAR(ref_point.distance, measured_point.distance, 1e-4);
  EXPECT_NEAR(ref_point.heading, measured_point.heading, 1e-4);
}

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();

  return ret;
}
