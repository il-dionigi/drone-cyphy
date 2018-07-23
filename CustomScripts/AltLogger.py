from Tkinter import *
from random import randint
 
# these two imports are important
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import time
import threading

import cflib

import datetime
import logging
import csv

import sys
import os

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FixedLocator, FormatStrFormatter
import matplotlib

dataLog = None
errorLog = None

stab_writer = None
pos_writer = None
acc_writer = None
gyro_writer = None
log_timestamp = None
log_conf = None

Logger = None

surf = None
fig = None

allowedItems = ['stab', 'pos', 'acc', 'gyro']
defaultPath = './LoggedData/'
argList = []

rtGraphing = None
mod10 = 0
xtenPoints = []
ytenPoints = []
for i in range(10):
	xtenPoints.append(0)
	ytenPoints.append(0)
xdataPoints = []
ydataPoints = []
plot = 0


def begin_logging(handle, arg1=None, arg2=None, arg3=None):
	global Logger, argList, rtGraphing, surf, fig

	itemList = None
	path = None
	rtGraphing = False

	if sys.version_info[0] < 3:
		if isinstance(arg1, basestring):
			if arg1[0] != '-':
				path = arg1
			else:
				argList.append(arg1[1])
		elif isinstance(arg2, basestring):
			if arg2[0] != '-':
				path = arg2
			else:
				argList.append(arg2[1])
		elif isinstance(arg3, basestring):
			if arg3[0] != '-':
				path = arg3
			else:
				argList.append(arg3[1])
	else:
		if isinstance(arg1, (str, unicode)):
			if arg1[0] != '-':
				path = arg1
			else:
				argList.append(arg1[1])
		elif isinstance(arg2, (str, unicode)):
			if arg2[0] != '-':
				path = arg2
			else:
				argList.append(arg2[1])
		elif isinstance(arg3, basestring):
			if arg3[0] != '-':
				path = arg3
			else:
				argList.append(arg3[1])

	if 'g' in argList:
		rtGraphing = True
		print("Graphing on!")

	if type(arg1) is list:
		itemList = arg1
	elif type(arg2) is list:
		itemList = arg2
	elif type(arg3) is list:
		itemList = arg3

	if path != None:
		if path[0:1] != './':
			path = './' + path
		if path[-1] != '/':
			path = path + '/'

	if itemList != None:
		for item in itemList:
			if item not in allowedItems:
				itemList.remove(item)
				print("Item {0} removed due to not being in list of allowed items, {1}".format(item, allowedItems))

	if itemList == None and path == None:
		Logger = AltLogger(handle, rtGraphing=rtGraphing)
	elif itemList != None and path == None:
		Logger = AltLogger(handle, items=allowedItems, rtGraphing=rtGraphing)
	elif itemList == None and path != None:
		Logger = AltLogger(handle, directory=defaultPath, rtGraphing=rtGraphing)
	else:
		Logger = AltLogger(handle, items=allowedItems, directory=defaultPath, rtGraphing=rtGraphing)
    threading.Thread(target=Logger.start_logging).start()
	threading.Thread(target=app).start()
	print("done!?")


class AltLogger:

	def __init__(self, handle, items=allowedItems, directory=defaultPath, rtGraphing=False):
		global fig, ax, surf
		if (rtGraphing):
			systemSideLength = 4
			lowerCutoffLength = 0.1
			fig = plt.figure()
			ax = fig.add_subplot( 111, projection='3d' )
			ax.set_zlim3d( 0, 1 )

			rng = np.arange( 0, systemSideLength, lowerCutoffLength )
			X, Y = np.meshgrid(rng,rng)

			ax.w_zaxis.set_major_locator( LinearLocator( 10 ) )
			ax.w_zaxis.set_major_formatter( FormatStrFormatter( '%.03f' ) )

			heightR = np.zeros( X.shape )
			surf = ax.plot_surface(X, Y, heightR, rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False )

			matplotlib.interactive(True)

		try:
			self.cf = handle.cf
		except:
			self.cf = handle

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
		global stab_writer, pos_writer, acc_writer, gyro_writer

		if not os.path.exists(self.directory):
			os.makedirs(self.directory)

		if 'stab' in self.items:
			stab_log_file = open(self.directory + self.log_timestamp + '_stab.csv', 'wb')
			stab_writer = csv.writer(stab_log_file)
			stab_writer.writerow(['time', 'roll', 'pitch', 'yaw'])

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
			pos_writer = csv.writer(pos_log_file)
			pos_writer.writerow(['time', 'x_pos', 'y_pos', 'z_pos'])

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
			acc_writer = csv.writer(acc_log_file)
			acc_writer.writerow(['time', 'x_accel', 'y_accel', 'z_accel'])

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
			gyro_writer = csv.writer(gyro_log_file)
			gyro_writer.writerow(['time', 'x_gyro', 'y_gyro', 'z_gyro'])

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

def csv_stab(timestamp, data, self):
	global stab_writer
	stab_writer.writerow([timestamp, data['stabilizer.roll'], data['stabilizer.pitch'], data['stabilizer.yaw']])

def csv_pos(timestamp, data, self):
	global pos_writer, rtGraphing, surf, fig, ax, mod10, xtenPoints, ytenPoints, plot
	pos_writer.writerow([timestamp, data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']])
	xtenPoints[mod10] = data['kalman.stateX']
	ytenPoints[mod10] = data['kalman.stateY']
	mod10 += 1
	if mod10 == 10:
		mod10 = 0
		ydataPoints += ytenPoints
		xdataPoints += xtenPoints
		plot = 1
	if rtGraphing:
		surf.remove()
		surf = ax.plot_surface( data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ'], rstride=1, cstride=1, cmap=cm.jet, linewidth=0, antialiased=False )
		plt.draw() # redraw the canvas
		fig.canvas.flush_events()

def csv_acc(timestamp, data, self):
	global acc_writer
	acc_writer.writerow([timestamp, data['acc.x'], data['acc.y'], data['acc.z']])

def csv_gyro(timestamp, data, self):
	global gyro_writer
	gyro_writer.writerow([timestamp, data['gyro.x'], data['gyro.y'], data['gyro.z']])

def log_error(self, logconf, msg):
	"""Callback from the log API when an error occurs"""
	print('Error when logging %s: %s' % (logconf.name, msg))
	if self.errorLog == None:
		if not os.path.exists(DIRECTORY):
			os.makedirs(DIRECTORY)
		self.errorLog = open(DIRECTORY + datetime.datetime.now().strftime("Error Log %Y-%m-%d_%H:%M:%S"), 'a')
	else:
		self.errorLog.write('Error when logging %s: %s\n' % (logconf.name, msg))


def change_state():
    global continuePlotting
    if continuePlotting == True:
        continuePlotting = False
    else:
        continuePlotting = True

def app():
	global xdataPoints, ydataPoints, plot
    # initialise a window.
    root = Tk()
    root.config(background='white')
    root.geometry("1000x700")
    
    lab = Label(root, text="Live Plotting", bg = 'white').pack()
    
    fig = Figure()
    
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.grid()
 
    graph = FigureCanvasTkAgg(fig, master=root)
    graph.get_tk_widget().pack(side="top",fill='both',expand=True)
 
    def plotter():
        while continuePlotting:
            ax.cla()
            ax.grid()
            ax.plot(xdataPoints, ydataPoints, marker='o', color='orange')
            graph.draw()
			plot = 0
			while(not plot):
            	time.sleep(0.05)
 
    def gui_handler():
        change_state()
        threading.Thread(target=plotter).start()
 
    b = Button(root, text="Start/stop", command=gui_handler, bg="red", fg="white")
    b.pack()
    
    root.mainloop()