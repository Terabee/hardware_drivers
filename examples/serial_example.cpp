#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include "serial_communication/iserial.hpp"
#include "serial_communication/serial.hpp"

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char **argv)
{
  std::signal(SIGTSTP, signal_handler);

  size_t max_buffer_size = 10;
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

  std::cout << "Reading from serial port: " << portname << std::endl;

  std::cout << "Press Ctrl + Z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << serial_port->readline(max_buffer_size) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return serial_port->close() ? 0 : -1;
}
