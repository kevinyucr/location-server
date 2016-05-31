#!/usr/bin/env python

'''
Requests position and attitude info from a Pixhawk and serves it.
'''

import sys, struct, time, os, math, random, thread
import serial
import zmq
import proto.location_pb2 

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate",
                    type=int,
                    default=115200,
                    help="master port baud rate")
parser.add_argument("--device",
                    default="/dev/ttyUSB0",
                    help="serial device")
parser.add_argument("--rate",
                    type=int,
                    default=10,help="requested stream rate")
parser.add_argument("--source-system",
                    type=int,
                    default=255,
                    help='MAVLink source system for this GCS')
parser.add_argument("--showmessages",
                    default=False,
                    help="show incoming messages")
args = parser.parse_args()

from pymavlink import mavutil

#class geotag_state:
#	def __init__(self):
#		self.alt_agl_meters = 0
#		self.roll_degrees = 0
#		self.pitch_degrees = 0
#		self.yaw_degrees = 0
#		self.lat_degrees = 0
#		self.lon_degrees = 0

#state = geotag_state()

def server_setup():
	context = zmq.Context()
	socket = context.socket(zmq.PUB)
	socket.bind("tcp://*:7777")
	return socket

def server_send_msg(socket, msg):
	print(msg)
	blob = msg.SerializeToString()
	socket.send(blob)
	
def wait_heartbeat(m):
	'''wait for a heartbeat so we know the target system IDs'''
	print("Waiting for APM heartbeat")
	m.wait_heartbeat()
	print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

def request_data_streams(master, rate):
	print("Requesting data s" % rate)
	for i in range(0, 3):
		master.mav.request_data_stream_send(master.target_system, master.target_component,
											mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 0)
	for i in range(0, 3):
	   master.mav.request_data_stream_send(master.target_system, master.target_component,
										   mavutil.mavlink.MAV_DATA_STREAM_POSITION, rate, 1)
	for i in range(0, 3):
	   master.mav.request_data_stream_send(master.target_system, master.target_component,
										   mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, rate, 1)

def mavlink_message_loop(master, socket, location_msg):
	# wait for the heartbeat msg to find the system ID
	wait_heartbeat(master)
	request_data_streams(master, args.rate)
	'''show incoming mavlink messages'''
	while True:
		msg = m.recv_match(blocking=True)
		if not msg:
			return
		if msg.get_type() == "BAD_DATA":
			if mavutil.all_printable(msg.data):
				sys.stdout.write(msg.data)
				sys.stdout.flush()

		elif msg.get_type() == "GLOBAL_POSITION_INT":
			state.alt_asl_meters = msg.alt / 1000.0
			state.alt_agl_meters = msg.relative_alt / 1000.0
			state.lat_degrees = msg.lat
			state.lon_degrees = msg.lon
 
		elif msg.get_type() == "ATTITUDE":
			state.roll_degrees = msg.roll * 180.0 / math.pi
			state.pitch_degrees = msg.pitch * 180.0 / math.pi
			state.yaw_degrees = msg.yaw * 180.0 / math.pi

		if msg:
			print("agl: {0}, asl: {1}, roll: {2}, pitch: {3}, yaw: {4}, lat: {5}, lon: {6}".format(
				state.alt_agl_meters,
				state.alt_asl_meters,
				state.roll_degrees,
				state.pitch_degrees,
				state.yaw_degrees,
				state.lat_degrees,
				state.lon_degrees))

	
def demo_message_loop(socket, plane_location):
	while True:
		plane_location.alt_agl_meters = random.randrange(0,100000)/1000.0;
		plane_location.alt_asl_meters = plane_location.alt_agl_meters + 25.0   # pretend we're 25 m above sea level
		plane_location.roll_degrees   = random.randrange(-45000,45000)/1000.0
		plane_location.pitch_degrees  = random.randrange(-25000,25000)/1000.0
		plane_location.yaw_degrees    = random.randrange(0,360000)/1000.0
		plane_location.lat_degrees    = random.randrange(-90000,90000)/1000.0
		plane_location.lon_degrees    = random.randrange(-180000,180000)/1000.0
		
		server_send_msg(socket, plane_location)
		time.sleep(1.0/args.rate);
										   
def main():
	try:
		# try to create a mavlink serial instance
		print("Trying to open serial device %s with baud rate %s..." % (args.device, args.baudrate))
		master = mavutil.mavlink_connection(args.device, baud=args.baudrate)
		print("Serial device opened successfully")
		demo_mode = False
	except serial.serialutil.SerialException:
		print("WARNING: Serial device failed to open. Continuing in demo mode and generating random messages.")
		demo_mode = True;
	
	socket = server_setup()
	plane_location = proto.location_pb2.PlaneLocation()
	
	if (demo_mode):
		demo_message_loop(socket, plane_location)
	else:
		mavlink_message_loop(master, socket, plane_location)
	
	#thread.start_new_thread(responder_loop, () )

#message_loop(master)


if __name__ == "__main__":
	main()