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

from Crypto.Cipher import AES
from Crypto import Random

def little_endian_read_16(buffer, pos):
    return ord(buffer[pos]) + (ord(buffer[pos+1]) << 8)

class H4Parser:
    state = "H4_W4_PACKET_TYPE"
    packet_type = "NONE"
    bytes_to_read = 1
    buffer = ''

    def __init__(self):
        self.reset()

    def set_packet_handler(self, handler):
        self.handler = handler

    def reset(self):
        self.bytes_to_read = 1
        self.buffer = ''
        self.state = "H4_W4_PACKET_TYPE"

    def parse(self, data):
        self.buffer += data
        self.bytes_to_read -= 1
        # print(self.buffer, self.bytes_to_read, self.state)
        if self.bytes_to_read == 0:
            if self.state == "H4_W4_PACKET_TYPE":
                self.buffer = ''
                if data == chr(1):
                    # cmd
                    self.packet_type = "CMD"
                    self.state = "W4_CMD_HEADER"
                    self.bytes_to_read = 3
                if data == chr(2):
                    # acl 
                    self.packet_type = "ACL"
                    self.state = "W4_ACL_HEADER"
                    self.bytes_to_read = 4
                return
            if self.state == "W4_CMD_HEADER":
                self.bytes_to_read = ord(self.buffer[2])
                self.state = "H4_W4_PAYLOAD"
                if self.bytes_to_read > 0:
                    return
                # fall through
            if self.state == "W4_ACL_HEADER":
                self.bytes_to_read = little_endian_read_16(buffer, 2)
                self.state = "H4_W4_PAYLOAD"
                if self.bytes_to_read > 0:
                    return
                # fall through
            if self.state == "H4_W4_PAYLOAD":
                self.handler(self.packet_type, self.buffer)
                self.reset()
                return

class HCIController:

    # fields
    parser = H4Parser()
    fd = -1
    random = Random.new()
    name = 'BTstack Mesh Simulator'
    bd_addr = '\x11\x22\x33\x44\x55\x66'

    def __init__(self):
        print('HCI Controller()')
        self.parser.set_packet_handler(self.packet_handler)

    def parse(self, data):
        self.parser.parse(data)

    def set_fd(self,fd):
        self.fd = fd

    def set_bd_addr(self, bd_addr):
        self.bd_addr = bd_addr

    def set_name(self, bd_addr):
        self.name = name

    def emit_command_complete(self, opcode, result):
        # type, event, len, num commands, opcode, result
        os.write(self.fd, '\x04\x0e' + chr(3 + len(result)) + chr(1) + chr(opcode & 255)  + chr(opcode >> 8) + result)

    def packet_handler(self, packet_type, packet):
        # print (packet_type, packet)
        opcode = little_endian_read_16(packet, 0)
        if opcode == 0x0c03:
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x1001:
            self.emit_command_complete(opcode, '\x00\x10\x00\x06\x86\x1d\x06\x0a\x00\x86\x1d')
            return
        if opcode == 0x0c14:
            self.emit_command_complete(opcode, '\x00' + self.name)
            return
        if opcode == 0x1002:
            self.emit_command_complete(opcode, '\x00\xff\xff\xff\x03\xfe\xff\xff\xff\xff\xff\xff\xff\xf3\x0f\xe8\xfe\x3f\xf7\x83\xff\x1c\x00\x00\x00\x61\xf7\xff\xff\x7f\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
            return
        if opcode == 0x1009:
            # read bd_addr
            self.emit_command_complete(opcode, '\x00' + self.bd_addr[::-1])            
            return
        if opcode == 0x1005:
            # read buffer size
            self.emit_command_complete(opcode, '\x00\x36\x01\x40\x0a\x00\x08\x00')
            return
        if opcode == 0x1003:
            # read local supported features
            self.emit_command_complete(opcode, '\x00\xff\xff\x8f\xfe\xf8\xff\x5b\x87')
            return
        if opcode == 0x0c01:
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x2002:
            # le read buffer size
            self.emit_command_complete(opcode, '\x00\x00\x00\x00')
            return
        if opcode == 0x200f:
            # read whitelist size
            self.emit_command_complete(opcode, '\x00\x19')
            return
        if opcode == 0x200b:
            # set scan parameters
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x0c6d:
            # write le host supported
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x2017:
            # LE Encrypt - key 16, data 16
            key = packet[18:2:-1]
            data = packet[35:18:-1]
            cipher = AES.new(key)
            result = cipher.encrypt(data)
            self.emit_command_complete(opcode, result[::-1])
            return
        if opcode == 0x2018:
            # LE Rand
            self.emit_command_complete(opcode, '\x00' + self.random.read(8))
            return
        if opcode == 0x2006:
            # Set Adv Params
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x2008:
            # Set Adv Data
            self.emit_command_complete(opcode, '\x00')
            return
        if opcode == 0x200a:
            # Set Adv Enable
            self.emit_command_complete(opcode, '\x00')
            return
        print("Opcode 0x%0x not handled!" % opcode)

# parse configuration file passed in via cmd line args
# TODO

# create ptys
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
    # print(read_ready, write_ready, exception_ready)    
    c = os.read(master, 1)
    controller.parse(c)


