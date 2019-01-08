#!/usr/bin/python

import socket
import sys
import time

UDP_IP = "192.168.4.1"
UDP_PORT = 3333
#MESSAGE = "02810500".decode("hex")

regs = {}

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind(("0.0.0.0", UDP_PORT))

def regdump():
    print "\n\n\n\n\n\n\n\n\n\n"
    for key in regs:
        print key.encode("hex"), regs[key].encode("hex")

if len(sys.argv) > 1:
    MESSAGE = sys.argv[1].decode("hex")
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    time.sleep(0.5)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    addr = data[0:2]
    payl = data[2:]
    if addr in regs and regs[addr] != payl:
        regs[addr] = payl
        #regdump()
        if addr not in [ '\x01\x51', '\x06\x23' ]:
            print addr.encode("hex"), payl.encode("hex")
    regs[addr] = payl

    #print addr.encode("hex"), payl.encode("hex")

