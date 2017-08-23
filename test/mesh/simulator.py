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
    packet_type = "NONE"
    bytes_to_read = 1
    buffer = []

    def __init__(self):
        self.reset()

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

class HCIController:

    parser = H4Parser()
    fd = -1

    def __init__(self):
        print('HCI Controller()')
        self.parser.set_packet_handler(self.packet_handler)

    def parse(self, data):
        self.parser.parse(data)

    def packet_handler(self, packet_type, packet):
        print (packet_type, packet)
        print ('Received HCI Reset, sending Command Complete')
        # TODO: check command, send appropriate response
        os.write(self.fd, "\x04\x0e\x04\x01\x03\x0c\x00")

    def set_fd(self,fd):
        self.fd = fd

# parser = H4Parser()
# parser.set_packet_handler(packet_handler)

# parse configuration file passed in via cmd line args
# TODO

# create ptys
# TODO as requested
(master, slave) = pty.openpty()
slave_ttyname = os.ttyname(slave)
print('slave  %u %s' % (slave, slave_ttyname))

controller = HCIController()
controller.set_fd(master)

# start up nodes 
subprocess.Popen(['./le_counter', '-u', slave_ttyname])

# get response from more than one
while True:
    (read_ready, write_ready, exception_ready) = select.select([master],[],[])
    print(read_ready, write_ready, exception_ready)    
    c = os.read(master, 1)
    controller.parse(c)


