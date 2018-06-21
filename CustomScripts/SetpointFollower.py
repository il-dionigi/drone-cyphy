# -*- coding: utf-8 -*-
#
#     ||          _  _ _
#  +------+      / _ )() /_______________ _  _
#  | 0xBC |     / _  / / _/ ___/ ___/ _ `/  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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

import cflib
from cflib.crazyflie import Crazyflie

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander


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
#     URI0,
# #    URI1,
# #    URI2,
# #    URI3,
# #    URI4,
# }

# params = {
#     URI0: [params0],
#     URI1: [params1],
#     URI2: [params2],
#     URI3: [params3],
#     URI4: [params4],
# }

sequence = [
    (0.0, 0.0, 0.4, 0),
    (0.6, 0.0, 0.4, 0),
    (0.6, 0.6, 0.4, 0),
    (0.0, 0.6, 0.4, 0),
    (0.0, 0.0, 0.4, 0),
    (0.6, 0.6, 0.6, 0),
    (0, 0, 0.4, 0),
    (0.2, 0.2, 0.4, 0),
    (-0.3, 0.3, 0.4, 0),
    (0.1, 0.1, 0.4, 0),
    (-0.1, -0.1, 0.4, 0),
    (0, 0, 0.4, 0),
    (0, 0, 0.4, 0),
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
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

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

def run_sequence(scf, sequence):
    cf = scf.cf

    cf.param.set_value('flightmode.posSet', '1')

    with MotionCommander(scf) as mc:

	    for position in sequence:
	        cf.commander.send_hover_setpoint(0, 0, 0, position[2])

	        for i in range(len(position)):
	            position_internal[i] = position[i]

	        mc.move_distance(position[0], position[1], 0)

	        time.sleep(1)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

# def poshold(cf, t, z):
#     steps = t * 10
#     for r in range(steps):
#         cf.commander.send_hover_setpoint(0, 0, 0, z)
#         time.sleep(0.1)

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
	cf.commander.send_hover_setpoint(0,0,0,0.4)
	time.sleep(0.2)
	cf.commander.send_hover_setpoint(0,0,0,0.2)
	cf.commander.send_hover_setpoint(0,0,0,0.1)
	cf.commander.send_hover_setpoint(0,0,0,0.05)

# def go_vertical(cf, t, dt, z0, base, direction):
#     steps = int(t / dt)
#     # Descend
#     if direction < 0:
#         for r in range(steps):
#             cf.commander.send_hover_setpoint(0, 0, 0, base + ((steps - r)/steps) * z0)
#             time.sleep(dt)
#     # Ascend
#     else:
#         for r in range(steps):
#             cf.commander.send_hover_setpoint(0, 0, 0, z0 * r / steps)
#             time.sleep(dt)


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

def follow_paths(scf):
	print('In function')
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
		if (position[0]/T > VMAX or position[1]/T > VMAX):
			t = position[0]/VMAX if position[0]/VMAX > position[1]/VMAX else position[1]/VMAX 
		go_straight_d(cf, movement[0], movement[1], movement[2], t)
		print('At pos: ({}, {}, {})'.format(position_internal[0], position_internal[1], position_internal[2]))
		
		for i in range(2):
			position_internal[i] += movement[i]
		for i in range(2):
			position_internal[i+2] = movement[i+2]

		time.sleep(1)

	

	time.sleep(0.1)

# def run_sequence(scf, parameters):
#     cf = scf.cf

#     # Delta T for sending commands
#     fsi = 0.1

#     # Base altitude in meters
#     base = 0.10

#     d = parameters['d']
#     z = parameters['z']
#     verse = parameters['ver']

#     xspeed = 0.6

#     if verse < 0:
#         # GO UP in 2 seconds
#         go_vertical(cf, 2, fsi, z, base, 1)

#         # HOLD THERE for 2 seconds
#         poshold(cf, 5, z)

#         # GO DOWN in 1 seconds
#         go_vertical(cf, 1, fsi, z, base, -1)
#     else:
#         cf.commander.send_setpoint(0, 0, 0, 0)

#         steps = int(6/0.1)
#         for r in range(steps):
#             cf.commander.send_setpoint(0, 0, 0, 36000)
#             time.sleep(0.1)

#     cf.commander.send_stop_setpoint()

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
        	# start_position_printing(scf)
    		follow_paths(scf)
    		#go_circular(scf, 360, 0.8, 0.4, 0, 4, 0.05)
    		go_land(scf)
    else:
        print('No Crazyflies found, cannot run example')

# Luigi's code
    # factory = CachedCfFactory(rw_cache='./cache')
    # with Swarm(uris, factory=factory) as swarm:
    #     swarm.parallel(reset_estimator)
    #     swarm.parallel(run_sequence, args_dict=params)