#!/bin/env python

import socket
import struct
import fcntl
import array

import Receiver

class Datagram(Receiver.Datagram):
    
    def __init__(self, source_address, source_port, target_address, target_port, ttl=32, loop=1):
        Receiver.Datagram.__init__(self, source_address, source_port)

        self.ttl = ttl
        self.loop = loop
        self.target_address = target_address
        self.target_port = target_port

        self.multicast = ord(socket.inet_aton(target_address)[0]) in range(224, 240)

        if self.multicast:
            self._socket.setsockopt (socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.ttl)
            self._socket.setsockopt (socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, self.loop)
            
    def write(self, data, *args):
        self._socket.sendto ( data, (self.target_address, self.target_port))
