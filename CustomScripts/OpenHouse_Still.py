# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
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

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Change uris according to your setup
URI0 = 'radio://0/75/1M/E7E7E7E701'
URI1 = 'radio://0/80/1M/E7E7E7E702'

URI2 = 'radio://0/94/2M/E7E7E7E7E7'
URI3 = 'radio://0/5/2M/E7E7E7E702'
URI4 = 'radio://0/110/2M/E7E7E7E703'

z1 = 0.35
z2 = 0.6

# d: diameter of circle
# z: altitude
params0 = {'d': 1.0, 'z': z1, 'ver': -1}
params1 = {'d': 1.0, 'z': z2, 'ver': -1}
params2 = {'d': 0.0, 'z': 0.2, 'ver': 1}
params3 = {'d': 1.0, 'z': 0.2, 'ver': 1}
params4 = {'d': 1.0, 'z': 0.2, 'ver': 1}


uris = {
    URI0,
#    URI1,
#    URI2,
#    URI3,
#    URI4,
}

params = {
    URI0: [params0],
    URI1: [params1],
    URI2: [params2],
    URI3: [params3],
    URI4: [params4],
}


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(3)


def poshold(cf, t, z):
    steps = t * 10
    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)


def go_straight(cf, v, t, dt, z):
    steps = int(t/dt)
    for r in range(steps):
        cf.commander.send_hover_setpoint(v[0], v[1], 0, z)
        time.sleep(dt)


def go_vertical(cf, t, dt, z0, base, direction):
    steps = int(t / dt)
    # Descend
    if direction < 0:
        for r in range(steps):
            cf.commander.send_hover_setpoint(0, 0, 0, base + ((steps - r)/steps) * z0)
            time.sleep(dt)
    # Ascend
    else:
        for r in range(steps):
            cf.commander.send_hover_setpoint(0, 0, 0, z0 * r / steps)
            time.sleep(dt)


def go_circular(cf, angle, diameter, z, direction, t, dt):
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


def run_sequence(scf, parameters):
    cf = scf.cf

    # Delta T for sending commands
    fsi = 0.1

    # Base altitude in meters
    base = 0.10

    d = parameters['d']
    z = parameters['z']
    verse = parameters['ver']

    xspeed = 0.6

    if verse < 0:
        # GO UP in 2 seconds
        go_vertical(cf, 2, fsi, z, base, 1)

        # HOLD THERE for 2 seconds
        poshold(cf, 5, z)

        # GO DOWN in 1 seconds
        go_vertical(cf, 1, fsi, z, base, -1)
    else:
        cf.commander.send_setpoint(0, 0, 0, 0)

        steps = int(6/0.1)
        for r in range(steps):
            cf.commander.send_setpoint(0, 0, 0, 36000)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel(reset_estimator)
        swarm.parallel(run_sequence, args_dict=params)
