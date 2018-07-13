# -*- coding: utf-8 -*-
#
#	 ||		  _  _ _
#  +------+	  / _ )() /_______________ _  _
#  | 0xBC |	 / _  / / _/ ___/ ___/ _ `/  / / _ \
#  +------+	/ /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||	/_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.



import math
import time
import sys 

import cflib
from cflib.crazyflie import Crazyflie

import os

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

import AltLogger
import logging

logging.basicConfig(level=logging.ERROR)

z1 = 0.35
z2 = 0.6

DT = 0.1 # Default dT for go_straight_d
T = 3 # default time of flight
VMAX = 0.3 # m/s if it goes higher than this then change t.
START_HEIGHT = 0.4 # Initial hover height

locoMode = 0

sequence = [
	# Square
	# (0.0, 0.0, 0.4, 0),
	# (0.0, 0.61, 0.4, 0),
	# (0.61, 0.61, 0.4, 0),
	# (0.61, 0.0, 0.4, 0),
	# (0.0,0.0,0.4,0.0)
	# Corner
	# (0.0, 0.0, 0.4, 0),
	# (0.0, 0.61, 0.4, 0),
	# (0.61, 0.61, 0.4, 0),
	# (0.0, 0.61, 0.4, 0),
	# (0.0, 0.0, 0.4, 0.0)
	# Hourglass
	(0.0, 0.0, 0.4, 0),
	(0.61, 0.61, 0.4, 0),
	(-0.61, 0.61, 0.4, 0),
	(0.0, 0.0, 0.4, 0),
	(0.61, -0.61, 0.4, 0.0),
	(-0.61, -0.61, 0.4, 0),
	(0.0, 0.0, 0.4, 0)
]

position_internal = [0,0,START_HEIGHT,0]

if len(sys.argv) < 2:
	print('''Error: this script takes an input designating the type of path following to be used - 'l' for LocoPosition-based, 'p' for Position-based, or 'v' for Velocity-based''')
	sys.exit(1)
elif not(sys.argv[1] in ['l', 'p', 'v', 'm']):
	print('''Error: this script takes either 'l' for LocoPosition-based path following, 'p' for Position-based path following, 'v' for Velocity-based path following, or 'm' for sending a message''')
	sys.exit(1)

def wait_for_position_estimator(scf):
	print('Waiting for estimator to find position...')

	log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
	log_config.add_variable('kalman.varPX', 'float')
	log_config.add_variable('kalman.varPY', 'float')
	log_config.add_variable('kalman.varPZ', 'float')

	var_y_history = [1000] * 10
	var_x_history = [1000] * 10
	var_z_history = [1000] * 10

	threshold = 0.001

	with SyncLogger(scf, log_config) as logger:
		for log_entry in logger:
			data = log_entry[1]

			var_x_history.append(data['kalman.varPX'])
			var_x_history.pop(0)
			var_y_history.append(data['kalman.varPY'])
			var_y_history.pop(0)
			var_z_history.append(data['kalman.varPZ'])
			var_z_history.pop(0)

			min_x = min(var_x_history)
			max_x = max(var_x_history)
			min_y = min(var_y_history)
			max_y = max(var_y_history)
			min_z = min(var_z_history)
			max_z = max(var_z_history)

			# print("{} {} {}".
			#	   format(max_x - min_x, max_y - min_y, max_z - min_z))

			if (max_x - min_x) < threshold and (
					max_y - min_y) < threshold and (
					max_z - min_z) < threshold:
				break

def reset_estimator(scf):
	cf = scf.cf
	cf.param.set_value('kalman.resetEstimation', '1')
	time.sleep(0.1)
	cf.param.set_value('kalman.resetEstimation', '0')
	time.sleep(0.2)

	wait_for_position_estimator(cf)

def position_callback(timestamp, data, logconf):
	x = data['kalman.stateX']
	y = data['kalman.stateY']
	z = data['kalman.stateZ']
	print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
	log_conf = LogConfig(name='Position', period_in_ms=500)
	log_conf.add_variable('kalman.stateX', 'float')
	log_conf.add_variable('kalman.stateY', 'float')
	log_conf.add_variable('kalman.stateZ', 'float')

	scf.cf.log.add_config(log_conf)
	log_conf.data_received_cb.add_callback(position_callback)
	log_conf.start()

def test_message(scf):
	cf = scf.cf

	cf.commander.send_message("~test")
	time.sleep(2)

def go_straight_d(cf, d_x, d_y, z, t, dt=DT):
	if (t == 0):
		return
	steps = int(t/dt)
	v = [d_x/t, d_y/t]
		
	for r in range(steps):
		cf.commander.send_hover_setpoint(v[0], v[1], 0, z)
		time.sleep(dt)

def go_land(scf):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')
	if (locoMode):
		cf.commander.send_hover_setpoint(0,0,0,0.4)
	else:
		cf.commander.send_setpoint(0,0,0,int(0.4*1000))
	


# def go_vertical(cf, t, dt, z0, base, direction):
#	 steps = int(t / dt)
#	 # Descend
#	 if direction < 0:
#		 for r in range(steps):
#	 cf.commander.send_hover_setpoint(0, 0, 0, base + ((steps - r)/steps) * z0)
#	 time.sleep(dt)
#	 # Ascend
#	 else:
#		 for r in range(steps):
#	 cf.commander.send_hover_setpoint(0, 0, 0, z0 * r / steps)
#	 time.sleep(dt)

def circ_left(scf, r, x, y, z, t, dt=DT, iterations=5):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')

	cf.commander.send_setpoint(0,0,0,400)

	steps = int((t/dt)/iterations)
	print(steps)
	center = [x-r, y, z, 0]
	for i in range(steps):
		position = [r*math.cos(i*2*math.pi/steps), r*math.sin(i*2*math.pi/steps), 0, 0]
		for i in range(len(position)):
			position[i] += center[i]
		print(position)
		for j in range(iterations):
			cf.commander.send_setpoint(position[1], position[0], position[3],
								   int(position[2] * 1000))
			time.sleep(dt)

def go_circular(scf, angle, diameter, z, direction, t, dt):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')
	steps = int(t / dt)
	rad_angle = angle * math.pi / 180

	speed = 0.5 * diameter * rad_angle / t

	if direction > 0:
		for _ in range(steps):
			cf.commander.send_hover_setpoint(speed, 0, angle / t, z)
			time.sleep(dt)
	else:
		for _ in range(steps):
		 	cf.commander.send_hover_setpoint(speed, 0, -angle / t, z)
		 	time.sleep(dt)

def loco_follow_paths(scf):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')

	for position in sequence:
		print('Setting position {}'.format(position))
		for i in range(200):
			cf.commander.send_setpoint(position[1], position[0], position[3],
									   int(position[2] * 1000))
			time.sleep(0.01)

	# Make sure that the last packet leaves before the link is closed
	# since the message queue is not flushed before closing
	time.sleep(0.1)

def pos_follow_paths(scf):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')
	# for position in sequence:
	# 	cf.commander.send_hover_setpoint(position[0], position[1], position[2], position[3])
	# 	time.sleep(2)
	# For future, make passed z a global z
	cf.commander.send_hover_setpoint(0,0,0,START_HEIGHT)
	time.sleep(1)
	print('About to hover at 40 cm')
	cf.commander.send_position_setpoint(0,0,START_HEIGHT,0)
	time.sleep(1)

	for position in sequence:
		print('Setting position {}'.format(position))
		cf.commander.send_position_setpoint(position[0], position[1], position[2], 0)
		time.sleep(1)

def vel_follow_paths(scf):
	cf = scf.cf
	cf.param.set_value('flightmode.posSet', '1')
	print(cf)
	# for position in sequence:
	# 	cf.commander.send_hover_setpoint(position[0], position[1], position[2], position[3])
	# 	time.sleep(2)

	# For future, make passed z a global z
	print('About to hover at 40 cm')
	cf.commander.send_hover_setpoint(0,0,0,START_HEIGHT)
	time.sleep(1)

	print('Hovering at 40 cm')

	movement = sequence[0]

	for position in sequence:
		movement = (position[0]-position_internal[0], 
					position[1]-position_internal[1], 
					position[2], 
					position[3])
		t = T
		if (abs(movement[0]/T) > VMAX or abs(movement[1]/T) > VMAX):
			t = abs(movement[0]/VMAX) if abs(movement[0]/VMAX) > abs(movement[1]/VMAX) else abs(movement[1]/VMAX) 
		go_straight_d(cf, movement[0], movement[1], movement[2], t)
		print('Moving: ({}, {}, {}) in time {}'.format(movement[0], movement[1], movement[2], t))
		print('Now at: ({}, {}, {})'.format(position_internal[0], position_internal[1], position_internal[2]))
		time.sleep(1)
		for i in range(4):
			position_internal[i] = position[i]
		time.sleep(0.1)

if __name__ == '__main__':
	cflib.crtp.init_drivers(enable_debug_driver=False)

	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	#time.sleep(0.5)
	print('Crazyflies found:')
	for i in available:
		print(i[0])

	if len(available) > 0:
		with SyncCrazyflie(available[0][0] + '/E7E7E7E7E8', cf=Crazyflie(rw_cache='./cache')) as scf:
			reset_estimator(scf)

			AltLogger.begin_logging(scf)

			locoMode = (sys.argv[1] == '1')

			if sys.argv[1] == 'l':
				loco_follow_paths(scf)
			elif sys.argv[1] == 'p':
				pos_follow_paths(scf)
			elif sys.argv[1] == 'v': 
				vel_follow_paths(scf)
			else: #sys.argv[1] == 'm':
				test_message(scf)

			print("Landing now...")
			# go_land(scf)
	else:
		print('No Crazyflies found, cannot run example')
