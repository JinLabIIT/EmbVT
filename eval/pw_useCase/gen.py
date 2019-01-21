
# every 100ms
  # fuction: 
    # get value from opendss
    # send value of load to control center
import pipe
import sys
import time
import threading
import zmq
import os
import logging
#import gtod
import thread 

TIME_INT = 0.1

Gen_ID = sys.argv[1]
server_IP = sys.argv[2]
server_Port = sys.argv[3]


com_lock=thread.allocate_lock()


logging.basicConfig(filename='%s.log'%Gen_ID,level=logging.DEBUG)

contextOut = zmq.Context()
clientOut = contextOut.socket(zmq.REQ)
clientOut.connect("tcp://%s:%s" % (server_IP,server_Port))
print('talking on %s:%s'%(server_IP,server_Port))
# open pipe to communicate to opendss

pipeout=pipe.setup_pipe_l(Gen_ID)
pipin = pipe.setup_pipe_w()

# send to control center
 
def send_cc(val):

    
    global gen_val1
    global gen_val2
    global gen_val3

    gens = val.split()

    print gens

    gen_val1 = float(gens[0])
    gen_val2 = float(gens[1])
    gen_val3 = float(gens[2])

    dif1 = -1.0 * (gen_val1 - 1292.0)
    dif2 = -1.0 * (gen_val2 - 1039.0)#(573 + load_1_val + load_2_val/2)                                                   
    dif3 = -1.0 * (gen_val3 - 1252.0)#(880 + load_3_val + load_2_val/2)                                                   

    print('difs')
    print(dif1)
    print(dif2)
    print(dif3)

    # create thread ()
    inputs = '%s %s %s %s %s'% (Gen_ID, gen_val1, gen_val2, gen_val3, time.time())
    thread.start_new_thread(send_es,(inputs,None))
    
def send_es(vall,boo):
    with com_lock:
        val = vall.encode('utf-8')
        clientOut.send(val, zmq.NOBLOCK)
        print 'sent but waiting for response'
        status=clientOut.recv()
        logging.debug('sent message to cc: %s '%val)
        print 'sent'
    thread.exit()

def get_val():
    update = 'update b p monitor_1 post_monitor_1 %s %s 1 mon_wind_gen\n' %(time.time(),Gen_ID)
    pipe.send_sync_event(update.encode('UTF-8'), pipin)

def t():
    print(time.time())

#time.sleep(2)# for sync to start properly

while 1:
    time.sleep(TIME_INT)
    get_val()
    
    #listen to response and send to cc
    x = pipe.listen(pipeout)
    if x:
        print x
        send_cc(x)
    
