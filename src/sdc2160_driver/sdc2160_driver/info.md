

Motor controller will use a file like /dev/ttyACM0 (because its over usb)

must add user running the project to the serial group:
    sudo usermod -aG dialout $USER

Probably want to add a stable device name even though we will move the gps for future errors:
    find controller ID:
        udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
    create this file:
        /etc/udev/rules.d/99-roteq.rules
        SUBSYSTEM=="tty", ATTRS{idVendor}=="20d2", ATTRS{idProduct}=="5740",SYMLINK+="sdc2160"
    Verify changes then run:
        sudo udevadm control --reload && sudo udevadm trigger
    after this use port:=/dev/sdc2160


Motor controller config:
    use Roborun+ on windows to configure
    Need to retune when weight changes occur

    must be closed loop
    set up feedback sensor
    tune the PID gains
    set motor current limit


Progressive Automation PA-04-6-900-MS-24VDC
    This is the linear actuator that the motor controller is attatched to


CANopen in roborun plus
write config to EEPROM
Persist ^MMOD
No ^ECHOF no echo on can
^RWD serial watchdog (check for watchdog in manual)

remove port, baudrate, and serial_timeout_s;

can_interface
can_chennel

bit rate = 250 kbps