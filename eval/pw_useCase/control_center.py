#channon@iit.edu
import zmq
import time
import threading
import os
import thread
import sys
import pipe
import logging
#import gtod

cc_ID = sys.argv[1] 
myIP = sys.argv[2]
ListenPort = sys.argv[3]
es_IP = sys.argv[4]
es_port = sys.argv[5]

logging.basicConfig(filename='%s.log'%cc_ID,level=logging.DEBUG)

#start
logging.debug('new run')

pipeout=pipe.setup_pipe_l(cc_ID)
pipin = pipe.setup_pipe_w()

com_lock = thread.allocate_lock()

# listen to Loads/Generator
contextIn = zmq.Context()
serverIn = contextIn.socket(zmq.REP)
print("tcp://%s:%s" % (myIP,ListenPort))
serverIn.bind("tcp://%s:%s" % (myIP,ListenPort))

# sent to Energy Storage Device
contextOut = zmq.Context()
clientOut = contextOut.socket(zmq.REQ)
print("tcp://%s:%s" % (es_IP,es_port))
clientOut.connect("tcp://%s:%s" % (es_IP,es_port))

def send_es(msg):
    print('sending %s' % msg)
    msg_bytes=msg.encode('utf-8')
    clientOut.send_string(msg)
    result=clientOut.recv()
    logging.debug('send: %s at time %s' % (msg,time.time()))

def listen():
    print('in listen')
    requestIn_bytes = serverIn.recv()
    print('requestIn_bytes')
    requestIn=requestIn_bytes.decode('utf-8')
    print('recv')
    ok='ok'.encode('utf-8')
    serverIn.send(ok)
    send_es(requestIn)

while 1:
    listen()
