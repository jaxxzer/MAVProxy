#!/usr/bin/env python
'''
support for outputting depth and temperature over UDP
'''

import socket, errno
import json
import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class DepthOutputModule(mp_module.MPModule):

    BUFFER_SIZE = 65536

    def __init__(self, mpstate):
        super(DepthOutputModule, self).__init__(mpstate, "DepthOutput", "Depth output support")
        self.add_command('DepthOutput.port', self.cmd_port, 'Port selection', ['<25400|25450|25500>'])
        self.add_command('DepthOutput.depthSource', self.cmd_depth_source, 'Depth source selection', ['<bar30|filtered>'])
        self.add_command('DepthOutput.tempSource', self.cmd_temp_source, 'Temperature source selection', ['<sp2|sp3>'])
        
        self.last_update = 0
        self.depth_source = 'filtered'
        self.temp_source = 'sp2'
        
        self.data = {
            'depth' : 0, # Depth in meters
            'temp' : 0   # Water temperature in degrees Celsius
        }
        self.portnum = 25400
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.port.bind(("127.0.0.1", self.portnum))
        #self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print "Outputting depth on UDP://%s:%s" % ("127.0.0.1", self.portnum)

    'Handle mavlink packets, get data'
    def mavlink_packet(self, m):
        if m.get_type() == 'SCALED_PRESSURE2':
            if self.temp_source == 'sp2':
                self.data['temp'] = m.temperature / 100.0 # m.temperature is centi-C
            if self.depth_source == 'bar30':
                self.data['depth'] = self.get_depth(m.press_diff * 100) # press_diff is hPa
        elif m.get_type() == 'SCALED_PRESSURE3':
            if self.temp_source == 'sp3':
                self.data['temp'] = m.temperature / 100.0 # m.temperature is centi-C
        elif m.get_type() == 'GLOBAL_POSITION_INT':
            if self.depth_source == 'filtered':
                self.data['depth'] = -m.relative_alt / 1000.0 # m.relative alt is mm

    def send_data(self):
        if time.time() < self.last_update + 0.25:
            return
        self.last_update = time.time()
        datagram = json.dumps(self.data)
        self.port.sendto(datagram, ('127.0.0.1', self.portnum))

    def idle_task(self):
        '''called in idle time'''
        self.send_data()

    def cmd_port(self, args):
        'handle port selection'
        if len(args) != 1:
            print("Usage: port <number>")
            return
        
        self.port.close()
        self.portnum = int(args[0])
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.port.bind(("127.0.0.1", self.portnum))
        #self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print "Outputting depth on UDP://%s:%s" % ("127.0.0.1", self.portnum)
        
    def cmd_temp_source(self, args):
        'handle temperature source selection'
        if len(args) != 1:
            print("Usage: tempSource <source>")
            return
        source = args[0]
        if source == 'sp2':
            self.temp_source = source
        elif source == 'sp3':
            self.temp_source = source
        else:
            print ('Unknown temperature source %s' % source)
        
    def cmd_depth_source(self, args):
        'handle depth source selection'
        if len(args) != 1:
            print("Usage: depthSource <source>")
            return
        source = args[0]
        if source == 'bar30':
            self.depth_source = source
        elif source == 'filtered':
            self.depth_source = source
        else:
            print ('Unknown depth source %s' % source)
            
    def get_depth(self, pressure_pa):
        specific_gravity = self.get_mav_param("GND_SPEC_GRAV", 1.0)
        return pressure_pa / 9800.0 / specific_gravity

def init(mpstate):
    '''initialise module'''
    return DepthOutputModule(mpstate)
