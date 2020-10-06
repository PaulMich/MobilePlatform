# MobilePlatform

This project is my attempt to design and construct a fully functional omnidirectional mobile platform. 

<b>mobile_platform.py</b> contains basic, simple program that allows to controll the platform via Bluetooth commands.\
<b>MP_engines_pwm_controll</b> implements basic controll of the platforms DC motors. It sets PWM signals based on commands received via I2C.\
<b>PWR_supply_board</b> keeps track of voltages on each Li-Po battery cell and shuts down robot power if voltages are too low or too high.\
<b>MP_MANIPULATOR.ino</b> implements forward and inverse kinematics of EEZYbotARM MK2 (designed by @daGHIZmo https://www.thingiverse.com/thing:1454048).\

Project status is still work-in-progress. At the moment I have assembled the whole robot with all the electronics and integrated it with a manipulator.
More detailed description of the project with all STL files is available at Thingiverse:

https://www.thingiverse.com/thing:4603000
