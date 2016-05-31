#!/usr/bin/env python

import zmq
import proto.location_pb2
import time

print(zmq.pyzmq_version())

context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Connecting to server")
socket.connect("tcp://localhost:7777")
print("Connected")

# Subscribe to all messages
socket.setsockopt(zmq.SUBSCRIBE, "")

while True:
	blob = socket.recv()
	message = proto.location_pb2.PlaneLocation()	
	message.ParseFromString(blob)
	print ("Recieved reply: %s, %s, %s" % (message.alt_agl_meters, message.lon_degrees, message.lat_degrees))

