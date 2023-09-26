# rp2040_pio_stepper
Stepper motor controll from the rp2040.

See BUILD.txt for details on building the RP firmware and the LinuxCNC driver.

The IP address of the RP2040 is 192.168.12.2.
I use a straight point-to-point ethernet connecton to an empty network interface on the LinuxCNC host.
I use 192.168.12.1 for the LinuxCNC host.

There's a sample LinuxCNC .hal and .ini file in the src/driver/ folder.
