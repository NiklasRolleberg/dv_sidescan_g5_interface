import socket
import time
import sys
import struct

serverAddressPort   = ("127.0.0.1",65025)
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


i = 0
while True:
    i+=1
    if(i>255):
        i=0

    print(i)

    right = b""
    left = b""

    right += struct.pack('>B',0xFE)
    right += struct.pack('>B',0x01)
    right += struct.pack('>B',i)
    right += struct.pack('>B',0)

    left += struct.pack('>B',0xFE)
    left += struct.pack('>B',0x01)
    left += struct.pack('>B',i)
    left += struct.pack('>B',1)

    for j in range(1000):
        right+=struct.pack('>B', (i + j) % 255)
        left+=struct.pack('>B', (i + j) % 255)


    UDPClientSocket.sendto(right, serverAddressPort)
    UDPClientSocket.sendto(left, serverAddressPort)
    time.sleep(0.1)
