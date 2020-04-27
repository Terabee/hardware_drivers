#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include "follow_me_driver/follow_me_driver.hpp"

#ifdef __linux__
  #define STOP_SIGNAL SIGTSTP
  #include "serial_communication/serial.hpp"
  using SerialInterface = terabee::serial_communication::Serial;
#elif defined(__MINGW32__) || defined(__MINGW64__) || defined(_WIN32) || defined(_WIN64)
  #define STOP_SIGNAL SIGTERM
  #include "serial_communication/serial_windows.hpp"
  using SerialInterface = terabee::serial_communication::SerialWindows;
#endif

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "usage: ./follow_me_driver_example PORT_NAME" << std::endl;
    return -1;
  }
  std::signal(STOP_SIGNAL, signal_handler);

  std::string portname = argv[1];
  std::shared_ptr<terabee::serial_communication::ISerial> serial_port =
    std::make_shared<SerialInterface>(portname);

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
  while (g_signal_status != STOP_SIGNAL)
  {
    master_beacon.process(point);
    std::cout << "Distance: " << point.distance << "\tHeading: " << point.heading << std::endl;
  }

  return serial_port->close() ? 0 : -1;
}
