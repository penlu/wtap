import aiohttp
import asyncio
import math
import pyvjoy
import requests
import time
import traceback

from util import PID, Estimator

# aircraft control model
# construct with map info
# takes updates via update_state, update_indicators, update_map
class Controller:
    def __init__(self, joystick, map_info):
        self.joystick = joystick

        self.min_x = map_info['map_min'][0]
        self.min_y = map_info['map_min'][1]
        self.max_x = map_info['map_max'][0]
        self.max_y = map_info['map_max'][1]

        self.pitch_pid = PID(0.04, 0, 0.02, anti_windup=(-1, 1))
        self.roll_pid = PID(0.04, 0.01, 0.02, anti_windup=(-1, 1))
        self.yaw_pid = PID(0.04, 0.01, 0.02, anti_windup=(-1, 1))

        self.alpha = 0
        self.beta = 0
        self.Vy = 0
        self.Wx = 0
        self.speed = 0
        self.altitude = 0
        self.roll = 0
        self.pitch = 0
        self.compass = 0

        self.init_time = time.time()
        self.state = 0

        self.state_count = 0
        self.indicator_count = 0
        self.map_count = 0

    def update_state(self, state):
        self.alpha = state['AoA, deg']
        self.beta = state['AoS, deg']
        self.Vy = state['Vy, m/s']
        self.Wx = state['Wx, deg/s']

    def update_indicators(self, indicators):
        self.speed = indicators['speed'] # m/s
        self.altitude = indicators['altitude_10k'] # feet
        self.roll = indicators['aviahorizon_roll']
        self.pitch = indicators['aviahorizon_pitch']
        self.compass = indicators['compass']
        self.get_control()

    def update_map(self, map_obj):
        for e in map_obj:
            if e['icon'] == 'Player':
                self.x = self.min_y + (self.max_y - self.min_y) * e['y']
                self.y = self.min_x + (self.max_x - self.min_x) * e['x']

    def get_control(self):
        if time.time() - self.init_time < 5:
            control = (0, 0, 0, 0)
        elif self.speed < 1:
            control = (0, 0, 0, 0.9)
        elif self.speed < 50:
            elevator = self.pitch_pid.update(-self.alpha)
            aileron = self.roll_pid.update(self.roll)
            rudder = self.yaw_pid.update((225 - self.compass + 180) % 360 - 180)
            control = (elevator, aileron * 50 / self.speed, rudder * 10 / self.speed, 0.9)
        else:
            if self.state == 0:
                self.pitch_pid = PID(0.05, 0.01, 0.01, anti_windup=(-1, 1))
                self.state = 1
            elevator = self.pitch_pid.update(7 - self.alpha)
            aileron = self.roll_pid.update(self.roll)
            rudder = self.yaw_pid.update((225 - self.compass + 180) % 360 - 180)
            control = (elevator, aileron * 50 / self.speed, rudder * 10 / self.speed, 0.9)

        #print(control)
        set_elevator(self.joystick, control[0])
        set_aileron(self.joystick, control[1])
        set_rudder(self.joystick, control[2])
        set_throttle(self.joystick, control[3])

# set_elevator, set_aileron, set_rudder expect float input in [-1, 1]
def set_elevator(j, v):
    v = min(1, max(-1, v))
    j.set_axis(pyvjoy.HID_USAGE_X, int((v + 1) * 0x4000))

def set_aileron(j, v):
    v = min(1, max(-1, v))
    j.set_axis(pyvjoy.HID_USAGE_Y, int((v + 1) * 0x4000))

def set_rudder(j, v):
    v = min(1, max(-1, v))
    j.set_axis(pyvjoy.HID_USAGE_RZ, int((v + 1) * 0x4000))

# set_throttle expects float input in [0, 1]
def set_throttle(j, v):
    v = min(1, max(0, v))
    j.set_axis(pyvjoy.HID_USAGE_Z, int(v * 0x8000))

class Poller:
    def __init__(self, controller):
        self.state_count = 0
        self.indicator_count = 0
        self.map_count = 0
        self.controller = controller

    async def poll_state(self):
        session = aiohttp.ClientSession()
        while True:
            try:
                state = await session.get(
                    'http://localhost:8111/state',
                    timeout=0.05)
                self.state_count += 1
                self.controller.update_state(await state.json())
            except:
                pass

    async def poll_indicators(self):
        session = aiohttp.ClientSession()
        while True:
            try:
                indicators = await session.get(
                    'http://localhost:8111/indicators',
                    timeout=0.05)
                self.indicator_count += 1
                self.controller.update_indicators(await indicators.json())
            except:
                pass

    async def poll_map(self):
        session = aiohttp.ClientSession()
        while True:
            try:
                map_obj = await session.get(
                    'http://localhost:8111/map_obj.json',
                    timeout=0.05)
                self.map_count += 1
                self.controller.update_map(await map_obj.json())
            except:
                pass

    async def print_counts(self):
        last_time = time.time()
        while True:
            await asyncio.sleep(1)
            t = time.time()
            print('in last %f seconds, got %d state %d indicators %d map' % \
                (t - last_time,
                self.state_count, \
                self.indicator_count, \
                self.map_count))
            self.state_count = 0
            self.indicator_count = 0
            self.map_count = 0
            last_time = t

def main():
    # main loop: asynchronously:
    # 1. periodically fetch data from WT and update the controller,
    # 2. periodically request joystick outputs from controller

    print('[INFO] fetching map info')
    while True:
        try:
            map_info = requests.get(
                'http://localhost:8111/map_info.json',
                timeout=0.01).json()
            break
        except KeyboardInterrupt as e:
            raise e
        except:
            pass

    # initialize objects
    joystick = pyvjoy.VJoyDevice(1)
    controller = Controller(joystick, map_info)
    poller = Poller(controller)

    print('[INFO] initializing main loop')
    loop = asyncio.get_event_loop()
    loop.create_task(poller.poll_state())
    loop.create_task(poller.poll_indicators())
    loop.create_task(poller.poll_map())
    loop.create_task(poller.print_counts())
    try:
        loop.run_forever()
    except:
        print('[INFO] cleaning up')
        loop.stop()
        set_elevator(joystick, 0)
        set_aileron(joystick, 0)
        set_rudder(joystick, 0)
        set_throttle(joystick, 0)
        joystick.update()

if __name__ == '__main__':
    main()