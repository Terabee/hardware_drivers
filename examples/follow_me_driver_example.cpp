#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include "follow_me_driver/follow_me_driver.hpp"
#include "serial_communication/serial.hpp"

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char **argv)
{
  std::signal(SIGTSTP, signal_handler);

  std::string portname = "/dev/ttyACM0";
  std::shared_ptr<terabee::serial_communication::ISerial> serial_port =
    std::make_shared<terabee::serial_communication::Serial>(portname);

  serial_port->setBaudrate(115200);
  serial_port->setTimeout(std::chrono::milliseconds(200));

  serial_port->open();

  if (!serial_port->isOpen())
  {
    std::cout << "Failed to open serial port!";
    return 0;
  }

  terabee::FollowMeMasterBeacon master_beacon(serial_port);
  terabee::PolarPoint2D point = { 0.0, 0.0 };

  master_beacon.printoutModeBin();
  std::cout << "Press Ctrl + Z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    master_beacon.process(point);
    std::cout << "Distance: " << point.distance << "\tHeading: " << point.heading << std::endl;
  }

  return serial_port->close() ? 0 : -1;
}
