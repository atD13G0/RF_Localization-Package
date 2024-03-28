#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

from numpy.linalg import inv
from time import time
from geometry_msgs.msg import TransformStamped

def trilaterate_based_on_berm(berm_w, berm_h, r1, r2, r3):
    half_w = berm_w / 2
    half_h = berm_h / 2
    r1_coord = (- half_w, half_h)
    r2_coord = (half_w, half_h)
    r3_coord = (- half_w, - half_h)
    return trilaterate(r1_coord, r1, r2_coord, r2, r3_coord, r3)

    
def trilaterate(r1_coord, r1, r2_coord, r2, r3_coord, r3):
    x1, y1 = r1_coord
    x2, y2 = r2_coord
    x3, y3 = r3_coord
    A = 2*x2 - 2*x1
    B = 2*y2 - 2*y1
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2*x3 - 2*x2
    E = 2*y3 - 2*y2
    F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)
    return x,y

#QUESTION: What does this do?
# Angle code portion is currently unused
def normalize_angle(angle):
    return np.mod(angle, 2 * np.pi)

def calc_orientation(head_pos, tail_pos, robot_width, robot_length):
    # diagonal angle offset: the diff. in angle between angle of diagonal of robot
    # calculated from the actual orientation of the robot
    dia_angle_offset = np.arctan(robot_width / robot_length)
    
    #QUESTION: What does vectorize mean and why are we turning it into an array?
    # vectorize
    head_vec = np.array(head_pos)
    tail_vec = np.array(tail_pos)
    # vector from tail to head
    t_h_vec = head_vec - tail_vec
    # note that for arctan2 the position of y and x are swapped
    # return the angle from x axis in radian
    diagonal_orientation = np.arctan2(t_h_vec[1], t_h_vec[0])

    return normalize_angle(diagonal_orientation - dia_angle_offset)


class publishCoords(Node):

    def __init__(self): #constructor
        super().__init__("get_coords")
        
        self.berm_width = 1
        self.berm_height = 0.5

        self.coordPublisher = self.create_publisher(Float32MultiArray,"/Head/Coord",10)

        #QUESTION: How come you are creating subrcribers seperately?
        self.radio_sub1 = Subscriber(self, Float32, "/Head/Anchor_A") # Replace with correct topic names
        self.radio_sub2 = Subscriber(self, Float32, "/Head/Anchor_B")
        self.radio_sub3 = Subscriber(self, Float32, "/Head/Anchor_C")

        #QUESTION: What does this do and what does its parameters mean?
        self.ts = ApproximateTimeSynchronizer([self.radio_sub1, self.radio_sub2, self.radio_sub3], 10, 1, allow_headerless=True) # Time should be a tighter margin
        self.ts.registerCallback(self.radio_dist_callback)

    def radio_dist_callback(self, radio1, radio2, radio3):
        x, y = trilaterate_based_on_berm(self.berm_width, self.berm_height, radio1.data, radio2.data, radio3.data)
        dist = math.sqrt(x ** 2 + y ** 2)

        msg = Float32MultiArray()
        coords = [x,y]
        msg.data = coords

        print("x: " + str(x) + "\n" + "y: " + str(y) + "\n" + "distance: " + str(dist))   
        self.coordPublisher.publish(msg)     

def main(args=None):
    rclpy.init(args=args)

    node = publishCoords()
    rclpy.spin(node)
    #QUESTION: Why do we need to destory the node afterwards?
    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__": #QUESTION: Is this neccessary and what is this used for?
    main()