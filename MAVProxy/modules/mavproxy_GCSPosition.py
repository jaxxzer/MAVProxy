#!/usr/bin/env python
'''
support for GCS position reporting
'''

import socket, errno
import json
import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module

class GCSPositionModule(mp_module.MPModule):

    BUFFER_SIZE = 65536

    def __init__(self, mpstate):
        super(GCSPositionModule, self).__init__(mpstate, "GCSPosition", "Positioning information for GCS (moving GCS)")
        self.add_command('GCSPosition.port', self.cmd_port, 'Port selection', ['<25101>'])
        self.gcs = mavutil.mavlink_connection('udpout:0.0.0.0:14550', source_system=2)
        self.last_heartbeat = 0
        self.data = {
            'time_usec' : 0,          # Timestamp (microseconds since UNIX epoch or microseconds since system boot) (uint64_t)
            'fix_type' : 3,           # See the GPS_FIX_TYPE enum. (uint8_t)
            'lat' : 0,                # Latitude (WGS84), in degrees * 1E7 (int32_t)
            'lon' : 0,                # Longitude (WGS84), in degrees * 1E7 (int32_t)
            'alt' : 100,              # Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude. (int32_t)
            'eph' : 1,                # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (uint16_t)
            'epv' : 1,                # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (uint16_t)
            'vel' : 0,                # GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX (uint16_t)
            'cog' : 0,                # Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX (uint16_t)
            'satellites_visible' : 6, # Number of satellites visible. If unknown, set to 255 (uint8_t)
            
            'time_boot_ms' : 0, # Timestamp (milliseconds since system boot) (uint32_t)
            'roll' : 0,         # Roll angle (rad, -pi..+pi) (float)
            'pitch' : 0,        # Pitch angle (rad, -pi..+pi) (float)
            'yaw' : 0,          # Yaw angle (rad, -pi..+pi) (float)
            'rollspeed' : 0,    # Roll angular speed (rad/s) (float)
            'pitchspeed' : 0,   # Pitch angular speed (rad/s) (float)
            'yawspeed' : 0      # Yaw angular speed (rad/s) (float)
        }
        
        self.ip=""
        self.portnum = 25101
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind((self.ip, self.portnum))
        self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print "Listening for GCS position packets on UDP://%s:%s" % (self.ip, self.portnum)

    def heartbeat(self):
        if time.time() < self.last_heartbeat + 1:
            return
        self.last_heartbeat = time.time()
        self.gcs.mav.heartbeat_send(
            1,
            1,
            1,
            1,
            1
    )

    def idle_task(self):
        '''called in idle time'''
        
        self.heartbeat()
        
        try:
            datagram = self.port.recvfrom(self.BUFFER_SIZE)
            data = json.loads(datagram[0])
            
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                return
            raise
        
        for key in data.keys():
            self.data[key] = data[key]
        
        try:
            self.gcs.mav.gps_raw_int_send(
                self.data['time_usec'],
                self.data['fix_type'],
                self.data['lat']*1E7,
                self.data['lon']*1E7,
                self.data['alt'],
                self.data['eph'],
                self.data['epv'],
                self.data['vel'],
                self.data['cog'],
                self.data['satellites_visible']
            )
            
            self.gcs.mav.attitude_send(
                self.data['time_boot_ms'], 
                self.data['roll'],
                self.data['pitch'],
                self.data['yaw'],
                self.data['rollspeed'],
                self.data['pitchspeed'],
                self.data['yawspeed']
            )
            
        except Exception,e:
            print "GCS Position Failed:", e

    def cmd_port(self, args):
        'handle port selection'
        if len(args) != 1:
            print("Usage: port <number>")
            return
        
        self.port.close()
        self.portnum = int(args[0])
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.port.bind((self.ip, self.portnum))
        self.port.setblocking(0)
        mavutil.set_close_on_exec(self.port.fileno())
        print "Listening for GCS position packets on UDP://%s:%s" % (self.ip, self.portnum)

def init(mpstate):
    '''initialise module'''
    return GCSPositionModule(mpstate)
