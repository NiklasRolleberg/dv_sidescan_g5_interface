#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import numpy as np
from smarc_msgs.msg import Sidescan
import pygame
from pygame.locals import *
import pygame.gfxdraw
import serial
import numpy as np
from scipy import interpolate
import sys, time
import math
from PIL import Image
from matplotlib import cm

pygame.init()
#ScreenSize = 1000
width = 1000
height = 500
Surface = pygame.display.set_mode((width,height),pygame.RESIZABLE)

echo_res  = 1000;
history_size = 1000; #number of samples kept in view
history = np.zeros((history_size,echo_res*2,3),dtype=np.uint8)

red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
darkBlue = (0,0,128)
white = (255,255,255)
black = (0,0,0)
pink = (255,200,200)

'''
def colorMapJet(value,maxValue):
    n = 4 * (float(value)/float(maxValue))
    r = 255 * min(max(min(n - 1.5, -n + 4.5), 0), 1)
    g = 255 * min(max(min(n - 0.5, -n + 3.5), 0), 1)
    b = 255 * min(max(min(n + 0.5, -n + 2.5), 0), 1)
    return (r ,g ,b)
'''

def draw(msg):
    print("Draw called")

    #move history
    for i in range(history_size-1,0,-1):
        history[i,:] = history[i-1,:]

    #Resize data for left & right
    echo_length = len(msg.starboard_channel)

    t = np.linspace(0,echo_length-1,echo_length)
    e_l = np.array([ c for c in msg.port_channel ])
    e_r = np.array([ c for c in msg.starboard_channel ])

    f_l = interpolate.interp1d(t, e_l)
    f_r = interpolate.interp1d(t, e_r)

    tnew = np.linspace(0,echo_length-1,echo_res)

    newEcho_left = f_l(tnew)
    newEcho_right = f_r(tnew)

    newEcho_left = np.log(newEcho_left)
    newEcho_right = np.log(newEcho_right)

    #Rescale a bit
    newEcho_left*= 255/6
    newEcho_right*= 255/6


    newEcho_left[newEcho_left < 1] = 1
    newEcho_right[newEcho_right < 1] = 1

    print("left: " + str(min(newEcho_left)) + " | " + str(max(newEcho_left)) + " right: " + str(min(newEcho_right)) + " | " + str(max(newEcho_right))) 
    
    newEcho_left[newEcho_left < 0] = 0
    newEcho_right[newEcho_right < 0] = 0
    newEcho_left[newEcho_left > 255] = 255
    newEcho_right[newEcho_right > 255] = 255
    



    for i in range(echo_res):

        #color_left = cm.jet(int(newEcho_left[i]))
        #color_right = cm.jet(int(newEcho_right[i]))

        color_left = cm.copper(int(newEcho_left[i]))
        color_right = cm.copper(int(newEcho_right[i]))

        color_left = np.array(color_left[0:3])*255
        color_right = np.array(color_right[0:3])*255

        history[0,echo_res - i] = color_left
        history[0,echo_res + i] = color_right

    img = Image.fromarray(history, 'RGB')

    mode = img.mode
    size = img.size
    data = img.tobytes()
    py_image = pygame.image.fromstring(data, size, mode)

    scaled_image = pygame.transform.scale(py_image,Surface.get_size())
    Surface.blit(scaled_image, (0, 0))
    pygame.display.update()

def main(args=None, namespace=None):
    rclpy.init(args=args)
    _node = Node('sidescan_visualizer_log')

    _node.declare_parameter('topic', "payload/sidescan")
    topic = _node.get_parameter('topic').value

    subsciber = _node.create_subscription(Sidescan, topic, draw, 10)
    subsciber

    rate = _node.create_rate(10)
    while rclpy.ok():
        rclpy.spin_once(_node)
        #pygame.display.flip()
        time.sleep(0.05)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit();
                sys.exit()
            elif event.type == pygame.VIDEORESIZE:
                scrsize = event.size
                screen = pygame.display.set_mode(scrsize,RESIZABLE)

if __name__ == "__main__":
    main()
