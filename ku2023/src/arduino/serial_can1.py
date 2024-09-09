from __future__ import print_function

import can
from time import sleep

import time
import threading

import can

def send():
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
    i=40
    while True:
            msg = can.Message(arbitration_id=0x011, is_extended_id=False,
                data=[0xaa,'1','0',0,128])
            bus.send(msg)
            i +=20
            sleep(0.2)

    bus.shutdown()


if __name__ == "__main__":
    send()
