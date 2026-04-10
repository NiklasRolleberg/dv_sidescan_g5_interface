#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import socket
import sys
import struct
import threading
from smarc_msgs.msg import Sidescan
from datetime import datetime
import struct
import signal


class DVSFileWriter:

    class V1_FileHeader:

        def __init__(self, range):
            self.nSamples = 1000  #Number of samples per side (int)
            self.sampleRes = range / self.nSamples  #[m] (float)
            self.left = True  #true if left/port side active (bool)
            self.right = True  #true if right/starboard side active (bool)
            self.lineRate = 750.0 / (float(self.nSamples) * self.sampleRes
                                     )  #[ ping/s ] (float)

        def pack(self):
            s = b""
            s += struct.pack('<I', 0x00000001)  #Version
            s += struct.pack('<f', self.sampleRes)
            s += struct.pack('<f', self.lineRate)
            s += struct.pack('<i', self.nSamples)
            s += struct.pack('<?', self.left)
            s += struct.pack('<?', self.right)
            return s

    class V1_Position:

        def __init__(self, lat, lon, speed, heading):
            self.lat = lat  #[rad] WGS84 (double)
            self.lon = lon  #[rad] WGS84 (double)
            self.speed = speed  #[m/s] (float)
            self.heading = heading  #[rad (float)

        def pack(self):
            s = b""
            s += struct.pack('<d', self.lat)
            s += struct.pack('<d', self.lon)
            s += struct.pack('<f', self.speed)
            s += struct.pack('<f', self.heading)
            return s

    class V1_PingReturn:

        def __init__(self, rightChannel, leftChannel):
            self.rightChannel = rightChannel
            self.leftChannel = leftChannel

        def pack(self):
            s = b""
            for val in self.rightChannel:
                s += struct.pack('<B', val)
            for val in self.leftChannel:
                s += struct.pack('<B', val)
            return s

    def __init__(self, log_dir, range):
        self.log_dir = log_dir
        self.range = range
        now = datetime.now()  # current date and time
        date_time = now.strftime("%Y%m%d_%H%M%S")
        self.filename = log_dir + "/" + date_time + '.dvs'
        self.OK = False

        self.C_PI = 3.1415926535897932384626433832795
        self.C_ER = 6353000

        self.fake_lat = 56.0 / 180.0 * self.C_PI  # approx 56 deg north
        self.fake_lon = 16.0 / 180.0 * self.C_PI  # approx 16 deg east, in the Baltic Sea
        self.dx = 1.0 * 50.0 / 750.0

        try:
            f = open(self.filename, 'wb')
            fileHeader = self.V1_FileHeader(range)
            f.write(fileHeader.pack())
            f.close()
            self.OK = True
        except Exception as e:
            print("Failed to open file" + str(e))

    def write(self, echo_right, echo_left):

        self.fake_lat -= self.dx / self.C_ER  #Move fake position

        pos = self.V1_Position(self.fake_lat, self.fake_lon, 1, 0)
        ping = self.V1_PingReturn(echo_right, echo_left)
        try:
            f = open(self.filename, 'ab')
            f.write(pos.pack())
            f.write(ping.pack())
            f.close()
        except Exception as e:
            print("Failed to open file" + str(e))


class UDP_listener(threading.Thread):

    def __init__(self, _socket, _callback, _shutdown_callback):
        threading.Thread.__init__(self)
        self.udp_socket = _socket
        self.callback = _callback
        self.shutdown_callback = _shutdown_callback
        self.stopListening = False

    def run(self):
        #while(not self.stop and not rclpy.ok()):
        #while(not self.stopListening):
        while (rclpy.ok()):
            print("Listening")
            try:
                data, address = self.udp_socket.recvfrom(4096)
                self.callback(data)
            except Exception as e:
                print("exception: " + str(e))
                #self.stopListening = True
        print("listener stopped")
        self.shutdown_callback()
        print("Sonar turned off")


class sss_decoder:

    def __init__(self, _sock, _sonar_ip, _sonar_port, _sonar_pub, log_dir):
        self.sock = _sock
        self.sonar_ip = _sonar_ip
        self.sonar_port = _sonar_port
        self.sonar_pub = _sonar_pub
        self.log_dir = log_dir

        self.sss_message = Sidescan()
        self.current_ping_number = 0
        self.ch0_received = False
        self.ch1_received = False

        self.file_writer = None

    def _set_regG5(self, reg, value):
        send_data = b""
        send_data += struct.pack('>B', 0xFE)  #header
        send_data += struct.pack('>B', reg)  #reg
        send_data += struct.pack('>H', value)  #value
        self.sock.sendto(send_data, (self.sonar_ip, self.sonar_port))

    def start_sonar(self, range, frequency=None, chirp=None):
        self._set_regG5(0x02, 0)  #200kHz, Short pulse
        self._set_regG5(0x01, range)  # xm range
        self.sss_message.range = range
        #self.file_writer = DVSFileWriter(log_dir="/tmp", range=range)

    def stop_sonar(self):
        self._set_regG5(0x01, 0x0)  #0m range
        self.file_writer = None

    def datagram_callback(self, data):
        #This funciton is called every time a new udp packet is received
        print("New UDP packet!!" + str(len(data)))
        if (len(data) == 1004):  #New SSS data received
            #Sanity checks
            assert (data[0] == 0xFE)
            assert (data[1] == 0x01)
            ping_nr = data[2]
            channel = data[3]
            echo = data[4:]

            if (channel == 0x00):
                self.sss_message.port_channel = echo
                self.ch0_received = True
            if (channel == 0x01):
                self.sss_message.starboard_channel = echo
                self.ch1_received = True

            if (ping_nr == self.current_ping_number
                ):  #We should have received both channels
                if (self.ch0_received and self.ch1_received):
                    self.sonar_pub.publish(self.sss_message)
                    print("published sss message")
                    if (self.file_writer):
                        self.file_writer.write(
                            self.sss_message.starboard_channel,
                            self.sss_message.port_channel)
                        print("Wrote to DVS file")
                self.ch0_received = False
                self.ch1_received = False
            self.current_ping_number = ping_nr

        else:
            print("Unknown UDP packet")
 

def rosparam_callback(params):
    return SetParametersResult(successful=True)
    
def main(args=None, namespace=None):
    rclpy.init(args=args)

    _node = Node('deepvisionG5_sidescan')

    _node.declare_parameter('output_topic', "payload/sidescan")
    topic = _node.get_parameter('output_topic').value

    _node.declare_parameter('sidescan_ip', "192.168.2.70")
    sonar_ip = _node.get_parameter('sidescan_ip').value

    _node.declare_parameter('sidescan_port', 65025)
    sonar_port = _node.get_parameter('sidescan_port').value

    _node.declare_parameter('range', 100)
    sonar_range = _node.get_parameter('range').value

    _node.declare_parameter('sonar_on', True)

    sonar_pub = _node.create_publisher(Sidescan, topic, 10)

    # Callback for sonar on/off rosparam.
    _node.add_on_set_parameters_callback(rosparam_callback)

    # Create a UDP socket>
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to the port
    local_address = ("0.0.0.0", sonar_port)
    s.bind(local_address)
    s.settimeout(1)

    decoder = sss_decoder(s, sonar_ip, sonar_port, sonar_pub, log_dir="/tmp")

    listener = UDP_listener(s, decoder.datagram_callback, decoder.stop_sonar)
    listener.start()

    #set up sonars
    decoder.start_sonar(range=sonar_range)
    print("Sonar started")

    while rclpy.ok():
        rclpy.spin_once(_node)
        sonar_on = _node.get_parameter('sonar_on').value
        if not sonar_on:
            _node.get_logger().info("Stopping DeepVision G5 sidescan sonar!")
            decoder.stop_sonar()
        else:
            sonar_range = _node.get_parameter('range').value
            _node.get_logger().info(
                "Starting DeepVision G5 sidescan sonar with range {0}".format(
                    sonar_range))
            decoder.start_sonar(range=sonar_range)


    decoder.stop_sonar()
    print("Sonar sopped")
    listener.stopListening = True
    listener.join()


if __name__ == "__main__":
    main()
