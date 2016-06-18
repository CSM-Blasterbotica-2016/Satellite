#!/usr/bin/env python

import pygame, sys
import numpy as np
import rospy
from std_msgs.msg import String
from math import pi

# Callback for subscriber to ROS excavation topic. Updates global variables with new values received
# and calls update_screen function to redraw display
def ex_callback(data):
	global ex_list
	ex_list = data.data.split(',')
	if len(ex_list) > 1:
		ex_list = map(int, ex_list)
		update_screen()

# Callback for subscriber to ROS wheel information topic. Updates global variables with new values
# received and calls update_screen function to redraw display
def wh_callback(data):
	global wh_list
	wh_list = data.data.split(',')
	if len(wh_list) > 1:
		wh_list = map(int, wh_list)
		update_screen()

# Top-level function. Draws screen in initial configuration, establishes variables, and starts ROS subscribers. 
# Once all startup code has been run, enters into loop and waits for callback functions to be triggered
def listener():
	global screen_width, screen_height, myfont, pink, black, screen, wh_list, ex_list
	screen_width = 1024
	screen_height = 550
	wh_list = [0,0,0,0]
	ex_list = [0,0,0,0,0]
	pink = (255,0,127)
	black = (0,0,0)
	
	pygame.init()
	myfont = pygame.font.SysFont("monospace", 14)
	screen = pygame.display.set_mode((screen_width,screen_height))
	update_screen()
	
	rospy.init_node('stat_listener')
	rospy.Subscriber("ex_current", String, ex_callback)
	rospy.Subscriber("wh_current", String, wh_callback)

	while not rospy.is_shutdown():
		rospy.spin()
		
# Calling this function will result in the screen being erased and redrawn to represent the current global 
# variable values. 
def update_screen():
	global ex_list, wh_list, screen, pink
	screen.fill(pink)
	lines = ((0, screen_height/2),(screen_width, screen_height/2), (screen_width/2, screen_height/2), (screen_width/2, 0))
	pygame.draw.lines(screen, black, False, lines, 2)
	make_bucket_ladder_positions(ex_list[2:4])
	make_dumping_bucket_circle(ex_list[4])
	make_bottom_rectangles((ex_list[:2] + wh_list))
	pygame.display.update()

# Helper function to draw bucket ladder related objects
def make_bucket_ladder_positions(values):
	global screen, black, myfont, screen_height, screen_width
	label = myfont.render("Bucket Ladder Depth", 1, black)
	screen.blit(label, (screen_width*5/8, screen_height/2-20))
	pygame.draw.rect(screen, black, (screen_width*5/8.0, screen_height/8, screen_width/8-5, screen_height/4), 1)
	pygame.draw.rect(screen, black, (screen_width*6/8.0, screen_height/8, screen_width/8-5, screen_height/4), 1)
	zero_point = screen_height/8+screen_height/4*5.5/15

	for i in range(len(values)):
		values[i] = (values[i] - 319) * -15 / 641.0 + 5.5
		if values[i] > 0:
			value = (zero_point - screen_height/8)*values[i]/5.5
			pygame.draw.rect(screen, black, (screen_width*(5+i)/8.0, zero_point-value, screen_width/8-5, value), 0)
		elif values[i] < 0:
			value = -1*(screen_height*3/8 - zero_point)*values[i]/9.5
			pygame.draw.rect(screen, black, (screen_width*(5+i)/8.0, zero_point, screen_width/8-5, value), 0)

# Helper function to draw dumping bucket-related object
def make_dumping_bucket_circle(value):
	global black, screen, screen_height, screen_width, myfont
	label = myfont.render("Dumping Bucket Angle", 1, black)
	screen.blit(label, (screen_width*1/8, screen_height/2-20))

	value = -1*((-102/720*(value-130)+63)/6.57+56)

	circle_size = 230
	size = (screen_width/4 - circle_size/2, screen_height/4 - circle_size/2, circle_size, circle_size)
	pygame.draw.arc(screen, black, size, 137*pi/180.0, 247*pi/180.0, 2)
	if value > 0:
		pygame.draw.arc(screen, black, size, (180 - value)*pi/180.0, pi, 90)
	elif value < 0:
		pygame.draw.arc(screen, black, size, pi, (180 - value)*pi/180.0, 90)

# Helper function to draw all current monitor objects
def make_bottom_rectangles(values):
	global screen_width, screen_height, myfont, black, screen
	vertical_offset = 20
	box_width = screen_width/(len(values))
	label = myfont.render("Bucket Ladder", 1, black)
	screen.blit(label, (0 * box_width + 10, screen_height-20))
	label = myfont.render("Dumping Bucket", 1, black)
	screen.blit(label, (1 * box_width + 10, screen_height-20))
	label = myfont.render("Wheel 1", 1, black)
	screen.blit(label, (2 * box_width + 10, screen_height-20))
	label = myfont.render("Wheel 2", 1, black)
	screen.blit(label, (3 * box_width + 10, screen_height-20))
	label = myfont.render("Wheel 3", 1, black)
	screen.blit(label, (4 * box_width + 10, screen_height-20))
	label = myfont.render("Wheel 4", 1, black)
	screen.blit(label, (5 * box_width + 10, screen_height-20))

	max_values = [10, 20, 70, 70, 70, 70]
	top = screen_height/2 + vertical_offset
	height = screen_height/2 - 2*vertical_offset
	for i in range(len(values)): 
		value = height*values[i]/max_values[i]/1000.0
		pygame.draw.rect(screen, black, (i * box_width, top, box_width-5, height), 1)
		pygame.draw.rect(screen, black, (i * box_width, top + height - value, box_width-5, value), 0)

if __name__ == "__main__":
	listener()
