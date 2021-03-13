import aiohttp
import asyncio
import pyvjoy
import requests
import time

j = pyvjoy.VJoyDevice(1)

# set_elevator, set_aileron, set_rudder expect float input in [-1, 1]
def set_elevator(v):
    j.set_axis(pyvjoy.HID_USAGE_X, int((v + 1) * 0x4000))

def set_aileron(v):
    j.set_axis(pyvjoy.HID_USAGE_Y, int((v + 1) * 0x4000))

def set_rudder(v):
    j.set_axis(pyvjoy.HID_USAGE_RZ, int((v + 1) * 0x4000))

# set_throttle expects float input in [0, 1]
def set_throttle(v):
    j.set_axis(pyvjoy.HID_USAGE_Z, int(v * 0x8000))

s = requests.Session()
def fetch():
    state = s.get('http://localhost:8111/state', timeout=0.01).json()
    indicators = s.get('http://localhost:8111/indicators', timeout=0.01).json()
    map_info = s.get('http://localhost:8111/map_info.json', timeout=0.01).json()
    map_obj = s.get('http://localhost:8111/map_obj.json', timeout=0.01).json()
    print(state, indicators, map_info, map_obj)

#def main():
    # main loop: asynchronously:
    # 1. periodically fetch data from WT and update the controller,
    # 2. periodically request joystick outputs from controller


while True:
    fetch()
    time.sleep(0.05)

while True:
    j.set_axis(pyvjoy.HID_USAGE_X, 0x4000)
    j.set_axis(pyvjoy.HID_USAGE_Y, 0x4000)
    j.set_axis(pyvjoy.HID_USAGE_Z, 0x0000)
    j.set_axis(pyvjoy.HID_USAGE_RZ, 0x4000)
    time.sleep(0.02)
    print("running")