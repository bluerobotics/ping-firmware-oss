#!/usr/bin/env python

import argparse
from brping import Ping1D, definitions
import subprocess
import time

parser = argparse.ArgumentParser(description="Ping python library example.")
parser.add_argument('--device', action="store", required=False, type=str, default="/dev/ttyUSB0", help="Ping device port. E.g: /dev/ttyUSB0")
parser.add_argument('--baudrate', action="store", type=int, default=115200, help="Ping device baudrate. E.g: 115200")
parser.add_argument('--hex', action="store", type=str, help="binary hex file to upload")
args = parser.parse_args()
if args.device is None:
    parser.print_help()
    exit(1)

# Check if stm32flash exists in the system
try:
    subprocess.check_call("which stm32flash", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    cmd = "stm32flash -b230400 -w " + args.hex + " -v -g 0x0 " + args.device
except subprocess.CalledProcessError:
    print("Error: stm32flash not found. Please install stm32flash first and make it available in your PATH.")
    exit(1)

try:
    subprocess.check_call(cmd, shell=True)
except subprocess.CalledProcessError as e:
    print("Device not found. Trying to enter bootloader...")

    myPing = Ping1D()
    myPing.connect_serial(args.device, args.baudrate)

    myPing.control_goto_bootloader()
    try:
        print(myPing.wait_message([definitions.COMMON_ACK, definitions.COMMON_NACK]))
    except:
        pass
    myPing.iodev.close()

    time.sleep(0.4)
    subprocess.check_call(cmd, shell=True)
