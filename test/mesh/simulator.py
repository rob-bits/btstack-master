#!/usr/bin/env python
#
# Simulate network of Bluetooth Controllers
#
# Each simulated controller has an HCI H4 interface
# Network configuration will be stored in a YAML file or similar
#
# Copyright 2017 BlueKitchen GmbH
#


import select
import pty
import os
import subprocess

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
    print('%02x' % ord(c))


