import time

import cflib

import datetime
import logging
import csv

import os

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

class NewLogger:
	dataLog = None
	errorLog = None

	stab_writer = None
	pos_writer = None
	acc_writer = None
	gyro_writer = None
	log_timestamp = None
	log_conf = None

	def __init__(self, cf, items, directory='./LoggedData/'):
		self.cf = cf
		self.items = items
		self.directory = directory

		logging.basicConfig(level=logging.ERROR)
		
		self.log_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')


	def start_position_printing(self):
		self.log_conf = LogConfig(name='Position', period_in_ms=500)
		self.log_conf.add_variable('kalman.stateX', 'float')
		self.log_conf.add_variable('kalman.stateY', 'float')
		self.log_conf.add_variable('kalman.stateZ', 'float')

		self.cf.log.add_config(log_conf)
		self.log_conf.data_received_cb.add_callback(position_callback)
		self.log_conf.start()

	def start_logging(self):
		if not os.path.exists(self.directory):
			os.makedirs(self.directory)

		if 'stab' in self.items:
			stab_log_file = open(self.directory + self.log_timestamp + '_stab.csv', 'wb')
			self.stab_writer = csv.writer(stab_log_file)
			self.stab_writer.writerow(['time', 'roll', 'pitch', 'yaw'])

			log_stab = LogConfig(name='Stabilizer', period_in_ms=10)

			try:
				log_stab.add_variable('stabilizer.roll', 'float')
				log_stab.add_variable('stabilizer.pitch', 'float')
				log_stab.add_variable('stabilizer.yaw', 'float')

				self.cf.log.add_config(log_stab)
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

		if 'pos' in self.items:
			pos_log_file = open(self.directory + self.log_timestamp + '_pos.csv', 'wb')
			self.pos_writer = csv.writer(pos_log_file)
			self.pos_writer.writerow(['time', 'x_pos', 'y_pos', 'z_pos'])

			log_pos = LogConfig(name='Position', period_in_ms=10)

			try:
				log_pos.add_variable('kalman.stateX', 'float')
				log_pos.add_variable('kalman.stateY', 'float')
				log_pos.add_variable('kalman.stateZ', 'float')

				self.cf.log.add_config(log_pos)
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

		if 'acc' in self.items:
			acc_log_file = open(self.directory + self.log_timestamp + '_acc.csv', 'wb')
			self.acc_writer = csv.writer(acc_log_file)
			self.acc_writer.writerow(['time', 'x_accel', 'y_accel', 'z_accel'])

			log_acc = LogConfig(name='Acceleration', period_in_ms=10)

			try:
				log_acc.add_variable('acc.x', 'float')
				log_acc.add_variable('acc.y', 'float')
				log_acc.add_variable('acc.z', 'float')

				self.cf.log.add_config(log_acc)
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

		if 'gyro' in self.items:
			gyro_log_file = open(self.directory + self.log_timestamp + '_gyro.csv', 'wb')
			self.gyro_writer = csv.writer(gyro_log_file)
			self.gyro_writer.writerow(['time', 'x_gyro', 'y_gyro', 'z_gyro'])

			log_gyro = LogConfig(name='Gyroscope', period_in_ms=10)

			try:
				log_gyro.add_variable('gyro.x', 'float')
				log_gyro.add_variable('gyro.y', 'float')
				log_gyro.add_variable('gyro.z', 'float')

				self.cf.log.add_config(log_gyro)
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

	def csv_stab(self, timestamp, data):
		self.stab_writer.writerow([timestamp, data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']])

	def csv_pos(self, timestamp, data):
		self.pos_writer.writerow([timestamp, data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']])

	def csv_acc(self, timestamp, data):
		self.acc_writer.writerow([timestamp, data['acc.x'], data['acc.y'], data['acc.z']])

	def csv_gyro(self, timestamp, data):
		self.gyro_writer.writerow([timestamp, data['gyro.x'], data['gyro.y'], data['gyro.z']])

	def log_error(self, logconf, msg):
		"""Callback from the log API when an error occurs"""
		print('Error when logging %s: %s' % (logconf.name, msg))
		if self.errorLog == None:
			if not os.path.exists(DIRECTORY):
				os.makedirs(DIRECTORY)
			self.errorLog = open(DIRECTORY + datetime.datetime.now().strftime("Error Log %Y-%m-%d_%H:%M:%S"), 'a')
		else:
			self.errorLog.write('Error when logging %s: %s\n' % (logconf.name, msg))