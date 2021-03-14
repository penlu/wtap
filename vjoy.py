import aiohttp
import asyncio
import math
import pyvjoy
import requests
import time
import traceback

class PIDController:
    def __init__(self, p, i, d):
        # initialize controller parameters
        self.p = p
        self.i = i
        self.d = d

        # initialize controller state
        self.last_t = time.time()
        self.last_e = 0
        self.tot_e = 0

    def update(self, e):
        # get current time
        t = time.time()

        # update integral term
        self.tot_e += e * (t - self.last_t)

        # calculate controller output
        u = self.p * e + self.i * self.tot_e + self.d * (e - self.last_e) / (t - self.last_t)

        # remember time and error
        self.last_t = t
        self.last_e = e
        return u

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

# aircraft control model
# construct with map info
# takes updates via update_state, update_indicators, update_map
class AircraftController:
    def __init__(self, joystick, map_info):
        self.joystick = joystick

        self.min_x = map_info['map_min'][0]
        self.min_y = map_info['map_min'][1]
        self.max_x = map_info['map_max'][0]
        self.max_y = map_info['map_max'][1]

        self.pitch_controller = PIDController(0.04, 0, 0.02)
        self.roll_controller = PIDController(0.04, 0.01, 0.02)
        self.yaw_controller = PIDController(0.04, 0.01, 0.02)

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
        #print('[DEBUG] %f got state' % time.time())
        self.state_count += 1
        self.alpha = state['AoA, deg']
        self.beta = state['AoS, deg']
        self.Vy = state['Vy, m/s']
        self.Wx = state['Wx, deg/s']

    def update_indicators(self, indicators):
        #print('[DEBUG] %f got indicators' % time.time())
        self.indicator_count += 1
        self.speed = indicators['speed'] # m/s
        self.altitude = indicators['altitude_10k'] # feet
        self.roll = indicators['aviahorizon_roll']
        self.pitch = indicators['aviahorizon_pitch']
        self.compass = indicators['compass']
        self.get_control()

    def update_map(self, map_obj):
        #print('[DEBUG] %f got map_obj' % time.time())
        self.map_count += 1
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
            elevator = self.pitch_controller.update(-self.alpha)
            aileron = self.roll_controller.update(self.roll)
            rudder = self.yaw_controller.update((225 - self.compass + 180) % 360 - 180)
            control = (elevator, aileron * 50 / self.speed, rudder * 10 / self.speed, 0.9)
        else:
            if self.state == 0:
                self.pitch_controller = PIDController(0.05, 0.01, 0.01)
                self.state = 1
            elevator = self.pitch_controller.update(7 - self.alpha)
            aileron = self.roll_controller.update(self.roll)
            rudder = self.yaw_controller.update((225 - self.compass + 180) % 360 - 180)
            control = (elevator, aileron * 50 / self.speed, rudder * 10 / self.speed, 0.9)

        #print(control)
        set_elevator(self.joystick, control[0])
        set_aileron(self.joystick, control[1])
        set_rudder(self.joystick, control[2])
        set_throttle(self.joystick, control[3])

async def poll_state(controller):
    session = aiohttp.ClientSession()
    while True:
        try:
            state = await session.get('http://localhost:8111/state', timeout=0.05)
            controller.update_state(await state.json())
        except:
            pass

async def poll_indicators(controller):
    session = aiohttp.ClientSession()
    while True:
        try:
            indicators = await session.get('http://localhost:8111/indicators', timeout=0.05)
            controller.update_indicators(await indicators.json())
        except:
            pass

async def poll_map(controller):
    session = aiohttp.ClientSession()
    while True:
        try:
            map_obj = await session.get('http://localhost:8111/map_obj.json', timeout=0.05)
            controller.update_map(await map_obj.json())
        except:
            pass

async def poll_controller(controller):
    last_time = time.time()
    while True:
        await asyncio.sleep(1)
        t = time.time()
        print('in last %f seconds, got %d state %d indicators %d map' % (t - last_time, controller.state_count, controller.indicator_count, controller.map_count))
        controller.state_count = 0
        controller.indicator_count = 0
        controller.map_count = 0
        last_time = t

def main():
    # main loop: asynchronously:
    # 1. periodically fetch data from WT and update the controller,
    # 2. periodically request joystick outputs from controller

    print('[INFO] fetching map info')
    while True:
        try:
            map_info = requests.get('http://localhost:8111/map_info.json', timeout=0.01).json()
            break
        except KeyboardInterrupt:
            exit(0)
        except:
            pass
    
    print('[INFO] initializing main loop')
    joystick = pyvjoy.VJoyDevice(1)
    controller = AircraftController(joystick, map_info)

    loop = asyncio.get_event_loop()
    loop.create_task(poll_state(controller))
    loop.create_task(poll_indicators(controller))
    loop.create_task(poll_map(controller))
    loop.create_task(poll_controller(controller))
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