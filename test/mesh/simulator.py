#!/usr/bin/env python
#
# Simulate network of Bluetooth Controllers
#
# Each simulated controller has an HCI H4 interface
# Network configuration will be stored in a YAML file or similar
#
# Copyright 2017 BlueKitchen GmbH
#


import os
import pty
import select
import subprocess
import sys

class H4Parser:
    state = "H4_W4_PACKET_TYPE"
    # H4_W4_PACKET_TYPE,
    # H4_W4_EVENT_HEADER,
    # H4_W4_ACL_HEADER,
    # H4_W4_SCO_HEADER,
    # H4_W4_PAYLOAD,
    packet_type = "NONE"
    bytes_to_read = 1
    buffer = []

    def set_packet_handler(self, handler):
        self.handler = handler

    def reset(self):
        self.bytes_to_read = 1
        self.buffer = []
        self.state = "H4_W4_PACKET_TYPE"

    def parse(self, data):
        self.buffer.append(data)
        self.bytes_to_read -= 1
        print(self.buffer, self.bytes_to_read, self.state)
        if self.bytes_to_read == 0:
            if self.state == "H4_W4_PACKET_TYPE":
                if data == chr(1):
                    # cmd
                    self.packet_type = "CMD"
                    self.state = "W4_CMD_HEADER"
                    self.bytes_to_read = 3
                    self.buffer = []
                if data == chr(2):
                    # acl 
                    self.packet_type = "ACL"
                    self.state = "W4_ACL_HEADER"
                    self.bytes_to_read = 4
                    self.buffer = []
                return
            if self.state == "W4_CMD_HEADER":
                self.bytes_to_read = ord(self.buffer[2])
                self.state = "H4_W4_PAYLOAD"
                if self.bytes_to_read > 0:
                    return
                # fall through
            if self.state == "W4_ACL_HEADER":
                self.bytes_to_read = ord(self.buffer[2]) + ord(self.buffer[3]) << 8
                self.state = "H4_W4_PAYLOAD"
                if self.bytes_to_read > 0:
                    return
                # fall through
            if self.state == "H4_W4_PAYLOAD":
                self.handler(self.packet_type, self.buffer)
                self.reset()
                return

def packet_handler(packet_type, packet):
    print (packet_type, packet)

parser = H4Parser()
parser.set_packet_handler(packet_handler)
# parser.parse(chr(0x01))
# parser.parse(chr(0x03))
# parser.parse(chr(0x0c))
# parser.parse(chr(0x00))
# sys.exit(0)


# parse configuration file passed in via cmd line args
# TODO

# create ptys
# TODO as requested
(master, slave) = pty.openpty()
slave_ttyname = os.ttyname(slave)
print('slave  %u %s' % (slave, slave_ttyname))

# start up nodes 
subprocess.Popen(['./le_counter', slave_ttyname])

# get response from more than one
while True:
    (read_ready, write_ready, exception_ready) = select.select([master],[],[])
    print(read_ready, write_ready, exception_ready)    
    c = os.read(master, 1)
    parser.parse(c)


