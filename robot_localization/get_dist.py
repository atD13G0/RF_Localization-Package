#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

class publishDist(Node):

    def __init__(self): #constructor
        super().__init__("get_dist")
        self.serial_A = self.create_publisher(Float32,"/Head/Anchor_A",10)
        self.serial_B = self.create_publisher(Float32,"/Head/Anchor_B",10)
        self.serial_C = self.create_publisher(Float32,"/Head/Anchor_C",10)
        self.get_logger().info("publish dist has been started")
        self.ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200)

        self.printSerial()

    
    def sendPacket(self, packet):
        msg = Float32()
        dist = packet[1]
        # msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.data = dist
        
        if(packet[0] == "A"):
            self.serial_A.publish(msg)
        elif(packet[0] == "B"):
            self.serial_B.publish(msg)
        elif(packet[0] == "C"):
            self.serial_C.publish(msg)

    def printSerial(self):
        while True:  
            valueRaw = self.ser.readline()
            valueInString = str(valueRaw,'UTF-8')
            packet = valueInString.split(',')
            packet[1] = float(packet[1])

            self.sendPacket(packet)
            self.get_logger().info(valueInString)

def main(args=None):
    rclpy.init(args=args)

    node = publishDist()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()