# Satellite

This code is intended for use on the satellite laptop by the robot operator. To ensure proper function, make sure that both the satellite and the core are associated with the proper WiFi network and that they have the following IP's:
* Satellite: 192.168.1.102
* Core: 192.168.1.101

## main.py
Main.py should be started first, and can initialize all other functions for a competition run. Assuming both the core and the satellite are on the same network, that the hardware on the robot itself is powered, and that the controller is plugged into the satellite, a complete startup would be performed as follows:
* From the home directory, start the script: ./main.py
* Press '1' to remotely start all necessary functions on the core laptop
* Press '2' to bring up the Robot Status Display
* Enjoy!

## test_display.py
The graphical monitor for the robot functions. Best if started from main.py, as the interface will not show anything meaningful unless there is a working robot attached to the core laptop. This will provide visual feedback on the status of the following systems:
* Dumping bucket angle, with an indicator of when the back of the bucket is horizontal
* Bucket ladder depth, with an indicator of when the tip of the bottom bucket is roughly coplanar with the bottoms of the wheels
* Motor currents for the bucket ladder motor, dumping bucket actuator, and all four wheels

## .bashrc
The only modifications to the bashrc file are at the end and pertain to configuring the ROS environment.

## check.list
This is the checklist that was executed by the controllers at the beginning of competition runs.
