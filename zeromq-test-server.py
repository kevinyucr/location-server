#!/usr/bin/env python

import zmq
import time

print(zmq.pyzmq_version())


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:7777")

while True:
	message = socket.recv()
	print("Recieved request: ", message)
#/	time.sleep(1)
	socket.send("Hello Pong");

