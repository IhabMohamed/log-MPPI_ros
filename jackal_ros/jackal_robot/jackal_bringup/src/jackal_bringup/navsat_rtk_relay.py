# Software License Agreement (BSD) 
#
# @author    Mike Purvis <mpurvis@clearpathrobotics.com>
# @copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import multicast
import serial
import sys
import time


def get_option_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--group",
                        help="multicast IP", metavar="IP", default="224.1.1.1")
    parser.add_argument("--port", type=int,
                        help="multicast port", metavar="PORT", default=20001)
    parser.add_argument("--device",
                        help="multicast device", metavar="ETH", default="eth0")
    parser.add_argument("--serial-port",
                        help="serial port", metavar="SER", default=None)
    parser.add_argument("--baud", type=int,
                        help="serial baud rate", metavar="RATE", default=57600)
    parser.add_argument("--verbose", action="store_true",
                        help="echo received messages to stdout")
    return parser


def main():
    args, _ = get_option_parser().parse_known_args()

    # This loop is necessary for when the script starts with ROS on the local-filesystems
    # event, it can run before the network devices are established.
    while True:
        try:
            receiver = multicast.MulticastUDPReceiver(args.device, args.group, args.port)
            print "Created multicast receiver on %s:%s, device %s." % (args.group, args.port, args.device)
            break
        except multicast.Receiver.InterfaceNotFound:
            print "Unable to find network interface %s, retrying." % args.device
            time.sleep(1.0)

    ser = None
    if args.serial_port:
        print "Will transmit to %s at %d baud." % (args.serial_port, args.baud)
    else:
        print "No serial port set, listening only."

    try:
        while True:
            s = receiver.read(10240)
            if args.serial_port and not ser:
                try:
                    ser = serial.Serial(port=args.serial_port, baudrate=args.baud, timeout=0)
                    print "Opened serial port."
                except Exception as e:
                    ser = None
                    print "Error opening serial port: %s" % str(e)
            if ser:
                ser.write(s)
                ser.flush()
            if args.verbose:
                sys.stdout.write(str(s).encode("string_escape"))
                sys.stdout.flush()

    except:
        if ser:
            ser.close()
            print "Closed serial port."
        raise
