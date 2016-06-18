#! /usr/bin/env python

from subprocess import call

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

core_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
source /opt/ros/indigo/setup.bash
roscore &
sleep 2
exit
HERE"""

serial_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
source /opt/ros/indigo/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 &
sleep 5
exit
HERE"""

joy_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
./ros/joyRateModifier.py &
sleep 2
exit
HERE"""

kill_core_command = """ssh -t -t blasterbotica@192.168.1.101 << HERE
killall -r -SIGTERM ros
sleep 1
exit
HERE
killall -r -SIGTERM ros"""

option = -1

call("clear", shell=True)

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
		call("lxterm -e '/home/satellite/test_display.py' &", shell=True)

	if (option == 3):
		call("rostopic list", shell=True)

	if (option == 4):
		call(core_command, shell=True)

	if (option == 5):
		call(serial_command, shell=True)

	if (option == 8):
		call(kill_core_command, shell=True)

