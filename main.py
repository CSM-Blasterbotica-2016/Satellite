#! /usr/bin/env python

from subprocess import call

# Several commands are defined below. These are strings that can be passed
# to python's subprocess.call function with the argument 'shell = True'
# and will be executed as though they were literally typed in a terminal

# This is the main startup command. It ssh's into the core laptop, starts 
# roscore, initializes the serial communication to the teensy, and starts
# the rate modifier for the joy topic which prevents buffer overflow when
# transmitting commands to the teensy. It then returns from the ssh and
# starts the joy node on js1 (js0 is a built in accelerometer in the laptop
# and needs to not be used. If a usb joystick is plugged in, it will be
# assigned to js1)
core_combined_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
source /opt/ros/indigo/setup.bash
roscore &
sleep 2
rosrun rosserial_python serial_node.py /dev/ttyACM0 &
sleep 5
./ros/joyRateModifier.py &
sleep 2
exit
HERE
rosparam set joy_node/dev /dev/input/js1
rosrun joy joy_node &"""

# This only ssh's into the core laptop and initializes roscore
core_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
source /opt/ros/indigo/setup.bash
roscore &
sleep 2
exit
HERE"""

# This only ssh's into the core laptop and initializes communication
# with the teensy arduino
serial_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
source /opt/ros/indigo/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 &
sleep 5
exit
HERE"""

# This kills all ros processes locally and on the remote core laptop
kill_core_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
killall -r -SIGTERM ros
sleep 1
exit
HERE
killall -r -SIGTERM ros"""

option = -1

call("clear", shell=True)

# Main menu loop
while option != 9:
	print "\nControl Options: "
	print "1. Start Manual Run"
	print "2. Open Status Window"
	print "3. List Active Topics"
	print "4. Start Remote Core Only"
	print "5. Start Remote Teensy Serial Only"
	print "8. Kill All ROS Processes"
	print "9. Exit"
	print ""
	option = input("Make a selection: ")
	
	if (option == 1):
		call(core_combined_command, shell=True)
	
	if (option == 2):
		# Open a new terminal and start the visual feedback display
		call("lxterm -e '/home/satellite/test_display.py' &", shell=True)

	if (option == 3):
		call("rostopic list", shell=True)

	if (option == 4):
		call(core_command, shell=True)

	if (option == 5):
		call(serial_command, shell=True)

	if (option == 8):
		call(kill_core_command, shell=True)

