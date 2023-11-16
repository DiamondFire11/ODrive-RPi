# ODrive-RPi
Library that ports the ODriveArduino library to the Raspberry Pi. The library works identically to how it does on Arduino. Utilizing a UART interface and the ASCII protocol to communicate with the ODrive microcontroller. This library requires the use of a USB TTY or hardware UART interface to function.

Credit to ODrive for the creation of the original library. Majority of the code used in this library was ported directly from that library, just stripped away all the Arduino code so that it could run on a Raspberry Pi natively in C++.
