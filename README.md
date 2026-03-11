# ARPAEthernetMotor

## Motor current node

This repository also includes firmware and a ROS2 node for publishing motor current from a WCS1600 sensor connected to a SAMD21 microcontroller.

### Firmware

Upload the Arduino sketch located in:

`SAMD21_WCS1600_Current_Reader/SAMD21_WCS1600_Current_Reader.ino`

The microcontroller should output one current value per line over USB serial.

Example output:

```text
0.123
0.127
0.119