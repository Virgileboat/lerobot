"""
Toolbox for motor configuration,
on exectution do th efolowing :
-check current protocole used by the motor and motor id
-swith to private protocole
-reboot motor
-change motor id
-change time delay
-open interface for changing what you want
-change protoocole to MIT protocole
-reboot motor
"""

import can
from time import sleep, time
import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack

from robstride_toolkit import *

bus = can.interface.Bus(interface="socketcan", channel="can0")



print(f"Pinging motor using CANopen protocol...")
for i in range(128):
    motor_id = i
    ping = ping_canopen(bus, motor_id)
    if ping:
        print(f"Motor {motor_id} responded to CANopen ping.")
        protocol = "CANopen"
        break

if not ping:
    print(f"Pinging motor using private protocol...")
    for i in range(128):
        motor_id = i
        ping = ping_private(bus, motor_id)
        if ping:
            print(f"Motor {motor_id} responded to private ping.")
            protocol = "private"
            break

if not ping:
    print(f"Pinging motor using MIT protocol...")
    for i in range(128):
        motor_id = i
        ping = ping_MIT(bus, motor_id)
        if ping:
            print(f"Motor {motor_id} responded to mit ping.")
            protocol = "MIT"
            break
if not ping:
    print("No motor responded to any protocol ping.")


if protocol != "MIT":
    if protocol == "CANopen":
        print("Switching motor to private protocol...")
        switch_canopen_to_private(bus, motor_id)
        sleep(1.0)
        input("reboot motor and press enter to continue...")
        print("sxitching motor to MIT protocol...")
        switch_private_to_mit(bus, motor_id)
        input("reboot motor and press enter to continue...")
    elif protocol == "private":
        print("Switching motor to MIT protocol...")
        switch_private_to_mit(bus, motor_id)
        input("reboot motor and press enter to continue...")




print("current motor id is :", motor_id)
new_id = int(input("Enter new motor ID (0-127): "))
if 1 <= new_id <= 127:
    print(f"Changing motor ID from {motor_id} to {new_id}...")
    change_motor_id(bus, motor_id, new_id)
else :
    print("Invalid motor ID. Must be between 1 and 127.")





### frequency test
import time
msg = can.Message(arbitration_id=1, data=[127, 255, 127, 240, 40, 51, 55, 255], is_extended_id=False)
t_ini = time.time()
n_msg = 0
while time.time() - t_ini < 10.0:
    bus.send(msg)
    n_msg += 1
    bus.recv(0.01)
t_end = time.time()
dt = t_end - t_ini
print(f"Sent {n_msg} messages in {dt:.2f} seconds ({n_msg/dt:.2f} Hz)")
