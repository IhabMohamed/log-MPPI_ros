#!/bin/env python

import socket
import struct
import fcntl
import array

import network

class InterfaceNotFound(Exception):
    def __init__(self, ifname):
        Exception.__init__(self, ifname)

class Datagram:
    """A datagram socket wrapper.    
    """
    iflist = None

    def __init__(self, address, port):
        
        if address is not None and len(address) > 0:
            try:
                socket.inet_aton ( address )
            except socket.error:           
                try: 
                    ifconfig = network.ifconfig()
                except network.IfConfigNotSupported:
                    raise InterfaceNotFound(address)
                
                if address in ifconfig:
                    iff = ifconfig[address]
                else:
                    raise InterfaceNotFound (address)
                self.local_address = iff.ip
            else:
                self.local_address = address
        else:
            self.local_address = None

        if self.local_address is None: self.local_address = ''

        self._socket = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        self.address = address
        self.port = port
        self.multicast = False

        self.bind()

    def bind(self):        
        self._socket.bind ( (self.local_address, self.port) )

    def pipe(self, ostream):
        while True:
            self.pipeone (ostream)

    def pipeone(self, ostream, size=1500):                
        data, addr = self._socket.recvfrom(size)
        
        if ostream is not None:
            if callable(ostream): target = ostream
            else: target = ostream.write
            return target (data)
        else:
            return data
    
    def read(self, size=1500):
        return self.pipeone (None, size)
    recv = read
                    
    def cleanup(self):
        pass

    def close(self):
        self.cleanup()
        self._socket.close()

    def __unicast__(self):
        return "DatagramReceiver [{0}] {1}:{2}".format (self._socket, self.local_address, self.port)

class Multicast(Datagram):
    def __init__(self, bind_address_or_interface, multicast_address, port, ttl=32, loop=1):        
        Datagram.__init__ (self, bind_address_or_interface, port )

        self.ttl = ttl
        self.loop = loop
        self.multicast_address = multicast_address
        self.multicast = True

        self._socket.setsockopt (socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.ttl)
        self._socket.setsockopt (socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, self.loop)

        self._socket.setsockopt (socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.local_address))
        self._socket.setsockopt (socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton (self.multicast_address) + socket.inet_aton(self.local_address))

    def bind(self):
        self._socket.bind ( ('', self.port) )

    def cleanup(self):
        self._socket.setsockopt (socket.SOL_IP, socket.IP_DROP_MEMBERSHIP, socket.inet_aton (self.multicast_address) + socket.inet_aton(self.local_address))

    def __unicast__(self):
        return "MulticastReceiver [{0}] {1}:{2} @ {3}".format (self._socket, self.multicast_address, self.port, self.local_address)


def DatagramReceiver (destination_address, destination_port, source_interface=None, ttl=32):
    multicast = ord(socket.inet_aton(destination_address)[0]) in range(224, 240)
    
    if multicast:
        receiver = Multicast (source_interface, destination_address, destination_port, ttl=ttl)
    else:
        receiver = Datagram (destination_address, destination_port)

    return receiver


    
