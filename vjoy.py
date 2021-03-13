import aiohttp
import asyncio
import pyvjoy
import requests
import time

# set_elevator, set_aileron, set_rudder expect float input in [-1, 1]
def set_elevator(j, v):
    j.set_axis(pyvjoy.HID_USAGE_X, int((v + 1) * 0x4000))

def set_aileron(j, v):
    j.set_axis(pyvjoy.HID_USAGE_Y, int((v + 1) * 0x4000))

def set_rudder(j, v):
    j.set_axis(pyvjoy.HID_USAGE_RZ, int((v + 1) * 0x4000))

# set_throttle expects float input in [0, 1]
def set_throttle(j, v):
    j.set_axis(pyvjoy.HID_USAGE_Z, int(v * 0x8000))

# aircraft control model
# construct with map info
# takes updates via update_state, update_indicators, update_map
class AircraftController:
    def __init__(self, map_info):
        self.min_x = map_info['map_min'][0]
        self.min_y = map_info['map_min'][1]
        self.max_x = map_info['map_max'][0]
        self.max_y = map_info['map_max'][1]

    def update_state(self, state):
        self.alpha = state['AoA, deg']
        self.beta = state['AoS, deg']
        self.Vy = state['Vy, m/s']
        self.Wx = state['Wx, deg/s']

    def update_indicators(self, indicators):
        self.speed = indicators['speed'] # unknown
        self.altitude = indicators['altitude_10k'] # feet
        self.roll = indicators['aviahorizon_roll']
        self.pitch = indicators['aviahorizon_pitch']
        self.hdg = indicators['compass']

    def update_map(self, map_obj):
        for e in map_obj:
            if e['icon'] == 'Player':
                self.x = self.min_y + (self.max_y - self.min_y) * e['y']
                self.y = self.min_x + (self.max_x - self.min_x) * e['x']

    def get_control(self):
        return (0, 0, 0, 0)

async def poll_state(session, controller):
    while True:
        try:
            state = await session.get('http://localhost:8111/state', timeout=0.01)
            controller.update_state(state.json())
        except:
            pass
        await asyncio.sleep(0.02)

async def poll_indicators(session, controller):
    while True:
        try:
            indicators = await session.get('http://localhost:8111/indicators', timeout=0.01)
            controller.update_indicators(indicators.json())
        except:
            pass
        await asyncio.sleep(0.02)

async def poll_map(session, controller):
    while True:
        try:
            map_obj = await session.get('http://localhost:8111/map_obj.json', timeout=0.01)
            controller.update_map(map_obj.json())
        except:
            pass
        await asyncio.sleep(0.02)

async def update_joystick(controller, joystick):
    while True:
        control = controller.get_control()
        set_elevator(joystick, control[0])
        set_aileron(joystick, control[1])
        set_rudder(joystick, control[2])
        set_throttle(joystick, control[3])
        await asyncio.sleep(0.02)

def main():
    # main loop: asynchronously:
    # 1. periodically fetch data from WT and update the controller,
    # 2. periodically request joystick outputs from controller

    print("[INFO] fetching map info")
    while True:
        try:
            map_info = requests.get('http://localhost:8111/map_info.json', timeout=0.01).json()
            break
        except:
            pass
    
    print("[INFO] initializing main loop")
    controller = AircraftController(map_info)
    session = aiohttp.ClientSession()
    joystick = pyvjoy.VJoyDevice(1)

    loop = asyncio.get_event_loop()
    loop.create_task(poll_state(session, controller))
    loop.create_task(poll_indicators(session, controller))
    loop.create_task(poll_map(session, controller))
    loop.create_task(update_joystick(controller, joystick))
    loop.run_forever()

if __name__ == '__main__':
    main()