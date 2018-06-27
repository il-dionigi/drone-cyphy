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
"""
Demo for the open house
"""
import math
import time
import sys 

import cflib
from cflib.crazyflie import Crazyflie

import datetime
import logging
import csv
import os

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

dataLog = None
errorLog = None

stab_writer = None
pos_writer = None
acc_writer = None
gyro_writer = None

log_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

DIRECTORY = './LoggedData/'

z1 = 0.35
z2 = 0.6

DT = 0.1 # Default dT for go_straight_d
T = 1 # default time of flight
VMAX = 0.3 # m/s if it goes higher than this then change t.
START_HEIGHT = 0.4 # Initial hover height
# d: diameter of circle
# z: altitude
params0 = {'d': 1.0, 'z': z1, 'ver': -1}
params1 = {'d': 1.0, 'z': z2, 'ver': -1}
params2 = {'d': 0.0, 'z': 0.2, 'ver': 1}
params3 = {'d': 1.0, 'z': 0.2, 'ver': 1}
params4 = {'d': 1.0, 'z': 0.2, 'ver': 1}


# uris = {
#	 URI0,
# #	URI1,
# #	URI2,
# #	URI3,
# #	URI4,
# }

# params = {
#	 URI0: [params0],
#	 URI1: [params1],
#	 URI2: [params2],
#	 URI3: [params3],
#	 URI4: [params4],
# }
locoMode = 0

sequence = [
	# Square
	# (0.0, 0.0, 0.4, 0),
	# (0.0, 0.61, 0.4, 0),
	# (0.61, 0.61, 0.4, 0),
	# (0.61, 0.0, 0.4, 0),
	# (0.0,0.0,0.4,0.0)
	# Corner
	(0.0, 0.0, 0.4, 0),
	(0.0, 0.61, 0.4, 0),
	(0.61, 0.61, 0.4, 0),
	(0.0, 0.61, 0.4, 0),
	(0.0, 0.0, 0.4, 0.0)
	# Hourglass
	# (0.0, 0.0, 0.4, 0),
	# (0.61, 0.61, 0.4, 0),
	# (-0.61, 0.61, 0.4, 0),
	# (0.0, 0.0, 0.4, 0),
	# (0.61, -0.61, 0.4, 0.0),
	# (-0.61, -0.61, 0.4, 0),
	# (0.0, 0.0, 0.4, 0)
]

position_internal = [0,0,START_HEIGHT,0]

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

def start_logging(scf, args):
	global stab_writer, pos_writer, acc_writer, gyro_writer

	if not os.path.exists(DIRECTORY):
		os.makedirs(DIRECTORY)

	if 'stab' in args:
		stab_log_file = open(DIRECTORY + log_timestamp + '_stab.csv', 'wb')
		stab_writer = csv.writer(stab_log_file)
		stab_writer.writerow(['time', 'roll', 'pitch', 'yaw'])

		log_stab = LogConfig(name='Stabilizer', period_in_ms=10)

		try:
			log_stab.add_variable('stabilizer.roll', 'float')
			log_stab.add_variable('stabilizer.pitch', 'float')
			log_stab.add_variable('stabilizer.yaw', 'float')

			scf.cf.log.add_config(log_stab)
			# This callback will receive the data
			log_stab.data_received_cb.add_callback(csv_stab)
			# This callback will be called on errors
			log_stab.error_cb.add_callback(log_error)
			# Start the logging
			log_stab.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add Stabilizer log config, bad configuration.')

	if 'pos' in args:
		pos_log_file = open(DIRECTORY + log_timestamp + '_pos.csv', 'wb')
		pos_writer = csv.writer(pos_log_file)
		pos_writer.writerow(['time', 'x_pos', 'y_pos', 'z_pos'])

		log_pos = LogConfig(name='Position', period_in_ms=10)

		try:
			log_pos.add_variable('kalman.stateX', 'float')
			log_pos.add_variable('kalman.stateY', 'float')
			log_pos.add_variable('kalman.stateZ', 'float')

			scf.cf.log.add_config(log_pos)
			# This callback will receive the data
			log_pos.data_received_cb.add_callback(csv_pos)
			# This callback will be called on errors
			log_pos.error_cb.add_callback(log_error)
			# Start the logging
			log_pos.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add Position log config, bad configuration.')

	if 'acc' in args:
		acc_log_file = open(DIRECTORY + log_timestamp + '_acc.csv', 'wb')
		acc_writer = csv.writer(acc_log_file)
		acc_writer.writerow(['time', 'x_accel', 'y_accel', 'z_accel'])

		log_acc = LogConfig(name='Acceleration', period_in_ms=10)

		try:
			log_acc.add_variable('acc.x', 'float')
			log_acc.add_variable('acc.y', 'float')
			log_acc.add_variable('acc.z', 'float')

			scf.cf.log.add_config(log_acc)
			# This callback will receive the data
			log_acc.data_received_cb.add_callback(csv_acc)
			# This callback will be called on errors
			log_acc.error_cb.add_callback(log_error)
			# Start the logging
			log_acc.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add Acceleration log config, bad configuration.')

	if 'gyro' in args:
		gyro_log_file = open(DIRECTORY + log_timestamp + '_gyro.csv', 'wb')
		gyro_writer = csv.writer(gyro_log_file)
		gyro_writer.writerow(['time', 'x_gyro', 'y_gyro', 'z_gyro'])

		log_gyro = LogConfig(name='Gyroscope', period_in_ms=10)

		try:
			log_gyro.add_variable('gyro.x', 'float')
			log_gyro.add_variable('gyro.y', 'float')
			log_gyro.add_variable('gyro.z', 'float')

			scf.cf.log.add_config(log_gyro)
			# This callback will receive the data
			log_gyro.data_received_cb.add_callback(csv_gyro)
			# This callback will be called on errors
			log_gyro.error_cb.add_callback(log_error)
			# Start the logging
			log_gyro.start()
		except KeyError as e:
			print('Could not start log configuration,'
				  '{} not found in TOC'.format(str(e)))
		except AttributeError:
			print('Could not add Gyroscope log config, bad configuration.')
# def poshold(cf, t, z):
#	 steps = t * 10
#	 for r in range(steps):
#		 cf.commander.send_hover_setpoint(0, 0, 0, z)
#		 time.sleep(0.1)

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

def circ_left(scf, r, x, y, z, t, dt=DT, iterations=10):
    	steps = t/dt
		center = [x-r, y, z, 0]
		for i in range(steps):
			position = [r*math.cos(i*math.pi*dt/t), r*math.sin(i*math.pi*dt/t), 0, 0]
			position += center
			for j in range(iterations):
    			cf.commander.send_setpoint(position[1], position[0], position[3],
                                       int(position[2] * 1000))
				time.sleep(dt/iterations)


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
		for i in range(50):
			cf.commander.send_setpoint(position[1], position[0], position[3],
                                       int(position[2] * 1000))
			time.sleep(0.1)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
	time.sleep(0.1)

def follow_paths(scf):
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

def csv_stab(timestamp, data, self):
	global stab_writer
	stab_writer.writerow([timestamp, data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']])

def csv_pos(timestamp, data, self):
	global pos_writer
	pos_writer.writerow([timestamp, data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']])

def csv_acc(timestamp, data, self):
	global acc_writer
	acc_writer.writerow([timestamp, data['acc.x'], data['acc.y'], data['acc.z']])

def csv_gyro(timestamp, data, self):
	global gyro_writer
	gyro_writer.writerow([timestamp, data['gyro.x'], data['gyro.y'], data['gyro.z']])

def log_error(self, logconf, msg):
	global errorLog
	"""Callback from the log API when an error occurs"""
	print('Error when logging %s: %s' % (logconf.name, msg))
	if errorLog == None:
		if not os.path.exists(DIRECTORY):
			os.makedirs(DIRECTORY)
		errorLog = open(DIRECTORY + datetime.datetime.now().strftime("Error Log %Y-%m-%d_%H:%M:%S"), 'a')
	else:
		errorLog.write('Error when logging %s: %s\n' % (logconf.name, msg))

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
		with SyncCrazyflie((available[0][0] + '/E7E7E7E7E8'), cf=Crazyflie(rw_cache='./cache')) as scf:
			reset_estimator(scf)

			start_logging(scf, ['stab', 'pos', 'acc', 'gyro'])
			locoMode = (sys.argv[1] == '1')
			# start_position_printing(scf)
			if locoMode:
				loco_follow_paths(scf)
			else:
				follow_paths(scf)
			#go_circular(scf, 360, 0.8, 0.4, 0, 4, 0.05)
			print("Landing now...")
			# go_land(scf)
	else:
		print('No Crazyflies found, cannot run example')

# Luigi's code
	# factory = CachedCfFactory(rw_cache='./cache')
	# with Swarm(uris, factory=factory) as swarm:
	#	 swarm.parallel(reset_estimator)
	#	 swarm.parallel(run_sequence, args_dict=params)