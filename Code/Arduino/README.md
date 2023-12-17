# Introduction
Teensy firmware used for crbot_kinematics

# Design goals
- Good HAL to achive portablility to different MCUs, boards, libraries, frameworks
- Re-usable code for different robot types (office robot and warehouse robot...)

# Todos:
- Perform sensor and hardware (industrial speaker, battery reading, motor driver...) checking before going to STATE_READY. Continously perform checking with slow rate. Notify user about the status
- Status LED for fully charged, stop by sonar sensors (obstacles detected)
- Use sonar sensors data for safety stop check
- Unify safety() func + running_permission and motor's set_velocity function
- Software version info -> send UART -> output the firmware version
- Hiện tại nếu encoder không cắm -> fault. Khi vẫn chưa cắm lại encoder -> Nhấn và nhả nút khẩn cấp -> Đèn tím solid -> Chuyển sang màu xanh (NORMAL) khoảng 1.5s thì nó mới bắt đầu báo lỗi => Cần check lỗi trước khi chuyển sang trạng thái NORMAL
- Cover the cases that can make motor keep velocity when jumping out CONTROL STATE
- Check sbus connection, there're three cases: 1. rx isn't connected; 2. rx is connected but lost connection to tx; 3. connection is ok
- Fix build warnings
- Pass sbus data to kinematics 
## Software Design
- Move program flow control functions and variables to a Class
- Combine `OpticalSensor.h` and `digital_io` Class

## Motor Driver (CANOpen)
- Integrate heartbeat msg (CAN failsafe feature) -> configure the driver to stop/disable motors when CAN communication is lost
- Use ppus functions to increase control accuracy
- Handle CANOpen message in ALARM state

## Completed
- Remove ROS_CONNECT block in order to cover Manual controlled Mode -> display led status to notify user -> done
- Notify rosserial connection through status led -> done
- Implement state machine to satisfy increasing code complexity (charging, status led, speaker, error state...) -> In progress
- Publish sonar sensor msg (similar to VOR's)
# Known Bugs, Issues

# Status LED behavior
- At startup_led = PURPLE (about 7 secs)
- In while loop() and rosserial isn't connected: PURPLE blink fast  
- Rosserial is connected -> ROS controlled: GREEN with blink pattern
- Manual controlled: PURPLE with blink pattern
- In Emergency button: RED
- Fault, error: RED with blink
- On lifiting driver: If there's a FAULT happens other than UNDER_VOLTAGE (triggered when Emergency Button is pressed) => Both motors will be disabled, the status led turns YELLOW, user must handle to recover accordingly

# Installation for Teensy4.1 board
## Install platformio-core:

- Comment the following lines in file ~/.pip/pip.conf
```
pip3 install --upgrade pip --trusted-host pypi.org --trusted-host files.pythonhosted.org
```
Reference https://groups.google.com/g/linorobot/c/ZwCIEqNQ6FM?pli=1

- Insall command: ``$pip install platformio`` 

## Install Arduino IDE for Teensy
- Install Arduino IDE: https://www.arduino.cc/en/software. Select Linux ARM 64 bits option if use ARM Linux Computer (such as Jetson)
- Install Teensy Arduino Add-on: https://www.pjrc.com/teensy/teensyduino.html
- Go to: https://www.pjrc.com/teensy/td_download.html and follow the tutorials. In summary: You need to select and download suitable Teensyduino, change execution mode -> run the Installer, select Arduino installation folder

## Fix related errors
1. Error opening USB device: Resource temporarily unavailable / "Found device but unable to open" When uploading code to Teensy via platformio
- Add rule using following steps:

``` bash
$cd /etc/udev/rules.d/
$sudo vim 49-teensy-4.rules
```
- Paste the contents from https://github.com/apmorton/teensy-template/blob/master/tools/49-teensy.rules to `49-teensy-4.rules`
- Reload rule:
``$sudo udevadm control --reload-rules``
- Re-plug USB cable

## Notes
- Note: After OS re-flash or on a fresh machine, platformio will require Internet connection to perform Installing `teensy` board support, therefore it's suggested to connect to a unrestricted network (VTCC) temporarily

# Firmware usage
- Build only:
``$platformio run --environment teensy41``
- Upload:
``$platformio run --target upload --environment teensy41``
## ROS
- From host computer (Jetson board), connect with this MCU via ROS:
``$rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200``

- The kinematic calculation is done by crbot_kinematic ROS package running on Embedded Computer. Which can be called using: 
`$rosrun crbot_kinematics crbot_kinematics_2wd`
