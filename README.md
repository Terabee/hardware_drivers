# hardware_drivers

## serial_communication

General purpose serial communication

### Features

 - support for different baudrate, parity, bytesize (5, 6, 7, or 8 bits), stopbits and flow control
 - implementation for Linux and Windows

### Usage

In order to use serial_communication in your code, include the interface header and appropriate implementation header, e.g.

```
#include "serial_communication/iserial.hpp"
#include "serial_communication/serial.hpp"
```

Add `hardware_drivers` to `find_package` of your `CMakeLists.txt` and link the target against `serial_communication`:

```
target_link_libraries(target_name
  hardware_drivers::serial_communication
)
```

See subdirectory `examples` for a C++ source file presenting usage of `serial_communication`.

## follow_me_driver

Driver dedicated for communication with Terabee Follow-Me system

### Features

 - set configuration parameters: span manual setting and auto-calibration, output mode (text/binary), swap beacons, Exponential Moving Average filter window size,
 - configuration of RS485 connection parameters (slave address, baud rate, parity)
 - set configuration parameters of the remote control: enable/disable buzzer, change button mode (toggle/hold)
 - data reception (distance and heading)

### Usage

Include the following header (already includes `serial_communication` header): 
```
#include "follow_me_driver/follow_me_driver.hpp"
```

and header with appropriate implementation of serial port, e.g.
```
#include "serial_communication/serial.hpp"
```

Add `hardware_drivers` to `find_package` of your `CMakeLists.txt` and link the target against `follow_me_driver`:

```
target_link_libraries(target_name
  hardware_drivers::follow_me_driver
)
```

See subdirectory `examples` for a C++ source file presenting usage of `follow_me_driver`.

## Compile

```
mkdir build
cd build
cmake ../
make
```

## Run tests

```
cd build
make test
```
For tests to pass, linux null modem emulator is required https://github.com/freemed/tty0tty
