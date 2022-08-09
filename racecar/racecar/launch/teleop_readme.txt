1. Put gamepad to D mode

2. Run: lsusb
   NOTE: Bus 001 Device 009: ID 046d:c219 Logitech, Inc. Cordless RumblePad 2

3. Run: sudo gedit /etc/udev/rules.d/99-joypad-f710.rules
   NOTE: 046d:c219 ==> KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"

4. Run: sudo udevadm control --reload-rules

5. Run: sudo udevadm trigger

6. Run: reboot
