#!/usr/bin/env python

import zmq
import time

print(zmq.pyzmq_version())


context = zmq.Context()
socket = context.socket(zmq.REQ)

print("Connecting to server")
socket.connect("tcp://localhost:7777")

while True:
	time.sleep(1)
	print ("Sending request")
	socket.send("Ping")
	message = socket.recv()
	print ("Recieved reply ", message);

