#!/bin/env python

import socket
import struct
import platform
try:
    import fcntl
    NO_FCNTL = False
except ImportError:
    NO_FCNTL = True
import array

SIOCGIFCONF = 0x8912
SIOCGIFFLAGS = 0x8913

IFF_MULTICAST = 0x1000   

class IfConfigNotSupported(Exception): pass

def ifconfig():
    """
    Fetch network stack configuration.
    """
    if NO_FCNTL:
        raise IfConfigNotSupported ( "No fcntl")
    
    class _interface:
        def __init__(self, name):
            self.name = name
            self.addresses = []
            self.up = False
            self.multicast = False

        def _first_ip(self):
            try:
                return self.addresses[0]
            except IndexError:
                return None
        ip = property(_first_ip)

    #An ugly hack to account for different ifreq sizes on
    #different architectures
    arch = platform.architecture()[0]
    if arch == "32bit": offsets = (32, 32)
    elif arch == "64bit": offsets = (16, 40)
    else: raise OSError ( "Unsupported architecture: %s" % (arch) )
        
    #Get the list of all network interfaces
    _socket = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
    buffer = array.array ( 'B', '\0' * 128 * offsets[1] )
    reply_length = struct.unpack ( 'iL', fcntl.ioctl(_socket.fileno(), SIOCGIFCONF, struct.pack ('iL', 4096, buffer.buffer_info()[0])))[0]
    if_list = buffer.tostring()    
    if_list = filter(lambda x: len(x[0]) > 0, [ (if_list[i:i+offsets[0]].split('\0', 1)[0], socket.inet_ntoa(if_list[i+20:i+24])) for i in range(0, 4096, offsets[1])])
    
    iff = {}
    
    #Get ip addresses for each interface
    for (ifname, addr) in if_list:
        iff[ifname] = iff.get (ifname, _interface(ifname) );
        flags, = struct.unpack ( 'H', fcntl.ioctl(_socket.fileno(), SIOCGIFFLAGS, ifname + '\0'*256)[16:18])
        iff[ifname].addresses.append ( addr )
        iff[ifname].up = bool(flags & 1)
        iff[ifname].multicast = bool(flags & IFF_MULTICAST)
        
    _socket.close()
    return iff


