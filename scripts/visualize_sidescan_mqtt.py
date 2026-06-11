#!/usr/bin/python3

import paho.mqtt.client as mqtt
import json
import numpy as np
import pygame
import pygame.gfxdraw
import numpy as np
import sys, time

from pygame.locals import *
from scipy import interpolate

from PIL import Image
from matplotlib import cm

pygame.init()
#ScreenSize = 1000
width = 1000
height = 500
Surface = pygame.display.set_mode((width,height),pygame.RESIZABLE)

echo_res  = 1000
history_size = 1000; #number of samples kept in view
history = np.zeros((history_size,echo_res*2,3),dtype=np.uint8);

visualization_settings = {'high_left': 255, 'low_left': 0, 'high_right': 255, 'low_right': 0}

new_frame = False

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

def draw(client, userdata, mqtt_msg):
    global new_frame

    data = json.loads(mqtt_msg.payload.decode())
    sidescan = data['sidescan']
    starboard = sidescan['starboard_channel']
    port = sidescan['port_channel']

    #move history
    for i in range(history_size-1,0,-1):
        history[i,:] = history[i-1,:];

    #Resize data for left & right
    echo_length = len(starboard)

    t = np.linspace(0, echo_length-1, echo_length)
    e_l = np.array(starboard)
    e_r = np.array(port)

    f_l = interpolate.interp1d(t, e_l)
    f_r = interpolate.interp1d(t, e_r)

    tnew = np.linspace(0,echo_length-1,echo_res)

    newEcho_left = f_l(tnew)
    newEcho_right = f_r(tnew)

    
    skip_begin = 0 #150
    skip_end = 1
    min_left = np.min(newEcho_left[skip_begin:-skip_end])
    #translate and scale the values

    newEcho_left[0:skip_begin] = min_left
    newEcho_left[-skip_end:] = min_left
    low_left = min(newEcho_left);
    high_left = max(newEcho_left);

    visualization_settings['low_left'] = 0.9*visualization_settings['low_left'] + 0.1*low_left
    visualization_settings['high_left'] = 0.9*visualization_settings['high_left'] + 0.1*high_left

    newEcho_left = (newEcho_left - visualization_settings['low_left']) * 255 / (visualization_settings['high_left']-visualization_settings['low_left']);

    min_right = np.min(newEcho_right[skip_begin:-skip_end])

    newEcho_right[0:skip_begin] = min_right
    newEcho_right[-skip_end:] = min_right
    low_right = min(newEcho_right);
    high_right = max(newEcho_right);

    visualization_settings['low_right'] = 0.9*visualization_settings['low_right'] + 0.1*low_right
    visualization_settings['high_right'] = 0.9*visualization_settings['high_right'] + 0.1*high_right

    newEcho_right = (newEcho_right - visualization_settings['low_right']) * 255 / (visualization_settings['high_right']-visualization_settings['low_right']);
    
    # print("left: " + str(low_left) + " | " + str(high_left) + " right: " + str(low_right) + " | " + str(high_right)) 
    

    
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

    """
    img = Image.fromarray(history, 'RGB')
    mode = img.mode
    size = img.size
    img_bytes = img.tobytes()
    py_image = pygame.image.fromstring(img_bytes, size, mode)
    scaled_image = pygame.transform.scale(py_image,Surface.get_size())
    Surface.blit(scaled_image, (0, 0))
    pygame.display.update()
    """
    new_frame = True

def main():
    global new_frame, Surface
    client = mqtt.Client()
    client.on_message = draw

    client.username_pw_set("", "") # adding here username, password 
    client.connect("", )       # adding here host, port
    client.subscribe("evolo/unit/surface/simulation/smarc_evolo/sensor/sidescan")      
    client.loop_start()                        

    while True:
        time.sleep(0.05)
        for event in pygame.event.get():
            if event.type == QUIT:
                client.loop_stop()
                pygame.quit()
                sys.exit()
            elif event.type == pygame.VIDEORESIZE:
                Surface = pygame.display.set_mode(event.size, pygame.RESIZABLE)

        if new_frame:
            img = Image.fromarray(history, 'RGB')
            py_image = pygame.image.fromstring(img.tobytes(), img.size, img.mode)
            scaled = pygame.transform.scale(py_image, Surface.get_size())
            Surface.blit(scaled, (0, 0))
            pygame.display.update()
            new_frame = False


if __name__ == "__main__":
    main()

