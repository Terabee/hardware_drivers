#include <memory>
#include <iostream>
#include <csignal>
#include <thread>
#include "serial_communication/iserial.hpp"

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
    std::cout << "usage: ./serial_example PORT_NAME" << std::endl;
    return -1;
  }
  std::signal(STOP_SIGNAL, signal_handler);

  size_t max_buffer_size = 10;
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

  std::cout << "Reading from serial port: " << portname << std::endl;

  std::cout << "Press Ctrl + Z to quit" << std::endl;
  while (g_signal_status != STOP_SIGNAL)
  {
    std::cout << serial_port->readline(max_buffer_size) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return serial_port->close() ? 0 : -1;
}
