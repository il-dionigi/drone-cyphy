# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
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
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.
"""
import logging
import time
import datetime
import os
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

dataLog = None
errorLog = None

DIRECTORY = './LoggedData/'

class Logger:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, cf, logIDs):
        """ Initialize and run the example with the specified link_uri """

        self._cf = cf

        self._logIDs

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def __del__(self):
        if errorLog != None:
            errorLog.close()

        if dataLog != None:
            dataLog.close()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        if 'stab' in self._logIDs:
            self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
            self._lg_stab.add_variable('stabilizer.roll', 'float')
            self._lg_stab.add_variable('stabilizer.pitch', 'float')
            self._lg_stab.add_variable('stabilizer.yaw', 'float')

            try:
                self._cf.log.add_config(self._lg_stab)
                # This callback will receive the data
                self._lg_stab.data_received_cb.add_callback(self._log_data)
                # This callback will be called on errors
                self._lg_stab.error_cb.add_callback(self._log_error)
                # Start the logging
                self._lg_stab.start()
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')

        if 'ext_pos' in self._logIDs:
            self._lg_ext_pos = LogConfig(name='External Position', period_in_ms=10)
            self._lg_ext_pos.add_variable('ext_pos.X', 'float')
            self._lg_ext_pos.add_variable('ext_pos.Y', 'float')
            self._lg_ext_pos.add_variable('ext_pos.Z', 'float')
    
            try:
                self._cf.log.add_config(self._lg_ext_pos)
                # This callback will receive the data
                self._lg_ext_pos.data_received_cb.add_callback(self._ext_pos_log_data)
                # This callback will be called on errors
                self._lg_ext_pos.error_cb.add_callback(self._ext_pos_log_error)
                # Start the logging
                self._lg_ext_pos.start()
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add External Position log config, bad configuration.')

        if 'gyro' in self._logIDs:
            self._lg_gyro = LogConfig(name='Gyroscope', period_in_ms=10)
            self._lg_gyro.add_variable('gyro.x', 'float')
            self._lg_gyro.add_variable('gyro.y', 'float')
            self._lg_gyro.add_variable('gyro.z', 'float')

            try:
                self._cf.log.add_config(self._lg_gyro)
                # This callback will receive the data
                self._lg_gyro.data_received_cb.add_callback(self._log_data)
                # This callback will be called on errors
                self._lg_gyro.error_cb.add_callback(self._log_error)
                # Start the logging
                self._lg_gyro.start()
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Gyroscope log config, bad configuation.')
        
        if 'acc' in self._logIDs:
            self._lg_accel = LogConfig(name='Acceleration', period_in_ms=10)
            self._lg_accel.add_variable('acc.x', 'float')
            self._lg_accel.add_variable('acc.y', 'float')
            self._lg_accel.add_variable('acc.z', 'float')

            try:
                self._cf.log.add_config(self._lg_accel)
                # This callback will receive the data
                self._lg_accel.data_received_cb.add_callback(self._log_data)
                # This callback will be called on errors
                self._lg_accel.error_cb.add_callback(self._log_error)
                # Start the logging
                self._lg_accel.start()
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Acceleration log config, bad configuration.')

    def _log_error(self, logconf, msg):
        global errorLog
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))
        if errorLog == None:
            if not os.path.exists(DIRECTORY):
                os.makedirs(DIRECTORY)
            errorLog = open(DIRECTORY + datetime.datetime.now().strftime("Error Log %Y-%m-%d_%H:%M:%S"), 'a')
        else:
            errorLog.write('Error when logging %s: %s\n' % (logconf.name, msg))

    def _log_data(self, timestamp, data, logconf):
        global dataLog
        """Callback froma the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        if dataLog == None:
            if not os.path.exists(DIRECTORY):
                os.makedirs(DIRECTORY)
            dataLog = open(DIRECTORY + datetime.datetime.now().strftime("Data Log %Y-%m-%d_%H:%M:%S"), 'a')
        else:
            dataLog.write('[%d][%s]: %s\n' % (timestamp, logconf.name, data))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = Logger(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

