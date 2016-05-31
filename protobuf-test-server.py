#!/usr/bin/env python

import zmq
import proto.location_pb2 
import time

print(zmq.pyzmq_version())


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:7777")

while True:
	message = proto.location_pb2.PlaneLocation()
	message.alt_agl_meters = 150.5
	message.lon_degrees = 135.342353234
	print message
	blob = message.SerializeToString()
	print message.ByteSize(), len(blob)
	socket.send(blob)
	time.sleep(1)


