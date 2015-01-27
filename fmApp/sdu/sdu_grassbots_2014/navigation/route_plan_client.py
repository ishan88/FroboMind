#!/usr/bin/env python
#/****************************************************************************
# FroboMind route plan client
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name of the copyright holder nor the names of its
#	  contributors may be used to endorse or promote products derived from
#	  this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""

2014-09-17 KJ First version
"""
from navigation_globals import *
from sys import argv
import signal
import time
from socket import *

SOCKET_SERVER = 'localhost'
SOCKET_PORT = 8080

class socket_client():
	def __init__(self, sock=None):
		if sock is None:
			self.sock = socket(AF_INET, SOCK_STREAM)
		else:
			self.sock = sock

	def connect(self, host, port):
		self.sock.connect((host, port))

	def send (self, msg):
		totalsent = 0
		msglen = len(msg)
		while totalsent < msglen:
			sent = self.sock.send(msg[totalsent:])
			if sent == 0:
				raise RuntimeError("socket connection broken")
			totalsent = totalsent + sent

	def receive(self):
		chunks = []
		bytes_recd = 0
		while True:
			chunk = self.sock.recv(1)
			if chunk == '':
				raise RuntimeError("socket connection broken")
			chunks.append(chunk)
			bytes_recd = bytes_recd + len(chunk)
			if chunk.find('\n') != -1:
				return ''.join(chunks)

class route_plan():
	def __init__(self):
		self.list = []
		self.seperator = ','

	def load_from_csv (self, filename):
		lines = [line.rstrip() for line in open(filename)] # read the file and strip \n
		wpt_num = 0
		for i in xrange(len(lines)): # for all lines
			if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
				data = lines[i].split (self.seperator) # split into comma separated list
				if len(data) >= CSV_N and data[CSV_E] != '' and data[CSV_N] != '':
					wpt_num += 1
					e = float (data[CSV_E])
					n = float (data[CSV_N])
					if (len(data) > CSV_HEADING) and data[CSV_HEADING] != '':
						heading = float (data[CSV_HEADING])
					else:
						heading = ROUTEPT_INVALID_DATA
					if (len(data) > CSV_ID) and data[CSV_ID] != '':
						name = data[CSV_ID]
					else:
						name = ''
					if (len(data) > CSV_NAV_MODE) and data[CSV_NAV_MODE] != '':
						if data[CSV_NAV_MODE] == 'PP':
							nav_mode = 'PP'
						elif data[CSV_NAV_MODE] == 'AB':
							nav_mode = 'AB'
						else:
							nav_mode = ''
					else:
						nav_mode  = ''
					if (len(data) > CSV_LIN_VEL) and data[CSV_LIN_VEL] != '':
						lin_vel = float (data[CSV_LIN_VEL])
					else:
						lin_vel = ROUTEPT_INVALID_DATA
					if (len(data) > CSV_ANG_VEL) and data[CSV_ANG_VEL] != '':
						ang_vel = float (data[CSV_ANG_VEL])
					else:
						ang_vel = ROUTEPT_INVALID_DATA
					if (len(data) > CSV_PAUSE) and data[CSV_PAUSE] != '':
						pause = float (data[CSV_PAUSE])
					else:
						pause = ROUTEPT_INVALID_DATA
					if (len(data) > CSV_TASK) and data[CSV_TASK] != '':
						task = int (data[CSV_TASK])
					else:
						task = ROUTEPT_INVALID_DATA

					self.list.append([e, n, heading, name, nav_mode, lin_vel, ang_vel, pause, task])
				else:
					print 'Erroneous waypoint: %s' % lines[i]
		print 'Total %d waypoints loaded.' % wpt_num


class socket_functions():
	def __init__(self):
		self.ack_timeout = 1.0
		self.socket = False

	def connect_to_socket_server(self):
		ack_ok = False
		# connect to socket server
		print 'Connecting to FroboMind route plan server %s port %s' % (socket_server, socket_port)
		self.socket = socket_client()
		try:
			self.socket.connect (SOCKET_SERVER, SOCKET_PORT)
		except:
			self.socket = False
		if self.socket != False:
			ack_tout = time.time() + self.ack_timeout
			while ack_ok == False and ack_tout > time.time():
				d = self.socket.receive()
				if d[0]=='$' and d[1]=='P' and d[2]=='F' and d[3]=='M' and d[4]=='R'and d[5]=='H':
					print 'Connected'
					ack_ok = True
				else:
					print 'Unknown response: %s' % answer
		if ack_ok == False:
			print 'Unable to connect'

	def disconnect_from_socket_server(self):
		print 'Disconnecting from server'
		if self.socket != False:
			self.socket.send ('$PFMRQ\r\n')

		else:
			print 'Already disconnected!'

	def delete_route_plan(self):
		self.connect_to_socket_server()
		if self.socket != False:
			print 'Deleting route plan'
			self.socket.send ('$PFMRD\r\n')
			ack_ok = False
			ack_tout = time.time() + self.ack_timeout
			while ack_ok == False and ack_tout > time.time():
				answer = self.socket.receive()
				if answer=='$PFMRD,ok\r\n':
					print 'Delete ok'
					ack_ok = True
				else:
					print 'Unknown response: %s' % answer
			self.disconnect_from_socket_server()
		else:
			print 'Unable to delete route plan'

	def subscribe_status(self):
		self.connect_to_socket_server()
		if self.socket != False:
			print 'Subscribing to the status (each 2 seconds)'
			self.socket.send ('$PFMRS,2.0\r\n')
			ack_ok = False
			ack_tout = time.time() + self.ack_timeout
			while ack_ok == False and ack_tout > time.time():
				answer = self.socket.receive()
				if answer=='$PFMRS,ok\r\n':
					print 'Subscription ok'
					ack_ok = True
					msg_tout = time.time() + self.ack_timeout
					while msg_tout > time.time():
						answer = self.socket.receive()
						if answer[0] == '$':
							print answer
							msg_tout = time.time() + self.ack_timeout
				else:
					print 'Unknown response: %s' % answer
			self.disconnect_from_socket_server()
		else:
			print 'Unable to suscribe to the status'

	def upload_route_plan(self, filename):
		self.connect_to_socket_server()

		# load route plan from file
		print 'Loading route plan from file'
		plan = route_plan()
		plan.load_from_csv (route_file_name)

		list_len = len(plan.list)
		i = 0
		send_err = False		
		while i < list_len and send_err == False:
			# send waypoint
			if plan.list[i][W_HEADING] != ROUTEPT_INVALID_DATA:
				heading_str = '%.3f' % plan.list[i][W_HEADING]
			else:
				heading_str = ''
			if plan.list[i][W_LIN_VEL] != ROUTEPT_INVALID_DATA:
				lin_vel_str = '%.3f' % plan.list[i][W_LIN_VEL]
			else:
				lin_vel_str = ''
			if plan.list[i][W_ANG_VEL] != ROUTEPT_INVALID_DATA:
				ang_vel_str = '%.3f' % plan.list[i][W_ANG_VEL]
			else:
				ang_vel_str = ''
			if plan.list[i][W_PAUSE] != ROUTEPT_INVALID_DATA:
				pause_str = '%.2f' % plan.list[i][W_PAUSE]
			else:
				pause_str = ''
			if plan.list[i][W_TASK] != ROUTEPT_INVALID_DATA:
				task_str = '%.2f' % plan.list[i][W_TASK]
			else:
				task_str = ''

			wpt_msg = '$PFMRE,%.3f,%.3f,%s,%s,%s,%s,%s,%s,%s\r\n' % (plan.list[i][W_E],plan.list[i][W_N], heading_str, plan.list[i][W_ID], plan.list[i][W_NAV_MODE],lin_vel_str, ang_vel_str, pause_str, task_str)
			self.socket.send (wpt_msg)

			# wait for acknowledge
			ack_ok = False
			ack_tout = time.time() + 1.0
			while ack_ok == False and ack_tout > time.time():
				answer = self.socket.receive()
				if answer=='$PFMRE,ok\r\n':
					print 'Route point %s sent ok' % plan.list[i]
					ack_ok = True
				elif answer=='$PFMRE,passwd\r\n':
					print 'Password error'
					ack_ok = True
					send_err = True
				else:
					print 'Unknown response: %s' % answer

			# go to next waypoint
			i += 1

		if send_err == False:
			print 'Route upload completed succesfully\n'
		else:
			print 'Route upload error\n'

	def mode_auto(self):
		self.connect_to_socket_server()
		if self.socket != False:
			print 'Switching to autonomous mode'
			self.socket.send ('$PFMHM,1\r\n')
			ack_ok = False
			ack_tout = time.time() + self.ack_timeout
			while ack_ok == False and ack_tout > time.time():
				answer = self.socket.receive()
				if answer=='$PFMHM,ok\r\n':
					print 'Switch ok'
					ack_ok = True
				else:
					print 'Unknown response: %s' % answer
			self.disconnect_from_socket_server()
		else:
			print 'Unable to switch to autonomous mode'

	def mode_manual(self):
		self.connect_to_socket_server()
		if self.socket != False:
			print 'Switching to manual mode'
			self.socket.send ('$PFMHM,0\r\n')
			ack_ok = False
			ack_tout = time.time() + self.ack_timeout
			while ack_ok == False and ack_tout > time.time():
				answer = self.socket.receive()
				if answer=='$PFMHM,ok\r\n':
					print 'Switch ok'
					ack_ok = True
				else:
					print 'Unknown response: %s' % answer
			self.disconnect_from_socket_server()
		else:
			print 'Unable to switch to manual mode'



# main function
print 'FroboMind route plan server utility v2014-09-17'
argc = len(argv)
if argc < 5:
	print ''
	print 'Usage: route_plan_client.py server port password command'
	print ''
	print 'Commands:'
	print '  delete'
	print '  upload filename'
	print '  auto'
	print '  manual'
	print '  status'
	print ''

else:
	# define and install ctrl-c handler
	ctrl_c = False
	def signal_handler(signal, frame):
		global ctrl_c
		ctrl_c = True
		print 'Ctrl-C pressed'
		print 'Quit'
	signal.signal(signal.SIGINT, signal_handler)

	socket_server = argv[1:][0]
	socket_port = argv[1:][1]
	socket_password = argv[1:][2]
	socket_cmd = argv[1:][3]

	sf = socket_functions()

	if socket_cmd == 'delete':
		sf.delete_route_plan()
	elif socket_cmd == 'upload':
		route_file_name =  argv[1:][4]
		sf.upload_route_plan (route_file_name)
	elif socket_cmd == 'status':
		sf.subscribe_status()
	elif socket_cmd == 'auto':
		sf.mode_auto()
	elif socket_cmd == 'manual':
		sf.mode_manual()
	else:
		print 'Unknown command'
	print ''


