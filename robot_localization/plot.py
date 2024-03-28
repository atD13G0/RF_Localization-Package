#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray
import multiprocessing 

class plot(Node):
    def __init__(self): #constructor
        super().__init__("plot")
        self.get_logger().info("Testing plot node once more!!")
        self.coordsSub = self.create_subscription(Float32MultiArray, "/Head/Coord", self.updateCoords, 10)
        self.q = multiprocessing.Queue()
        self.p1 = multiprocessing.Process(target= self.showPlot, args=(self.q,)) 
        self.p1.start()
        # self.showPlot()

    def updateCoords(self, msg):
        self.get_logger().info("Coords: [" + str(msg.data[0]) + ", "+ str(msg.data[1]) +"] \n")
        self.q.put([msg.data[0], msg.data[1]])

    def showPlot(i,q):
        #define Matplotlib figure and axis
        fig = plt.figure()
        limit = 5
        axis = plt.axes(xlim = (-limit,limit), ylim = (-limit,limit))
        axis.set_title("Robot Localization")

        #draw axis
        axis.axhline(y = 0, color = 'r', linestyle = '--') 
        axis.axvline(x = 0, color = 'r', linestyle = '--')

        # #add rectangle to plot
        axis.add_patch(Rectangle((-0.5, -0.25), 1, 0.5, fill=False))
        axis.scatter(0,0, c="r")

        #create scatter
        plt.ion()
        x, y = 0, 0
        scat = axis.scatter(x, y, lw = 0.5)

        plt.draw()
        while True:
            if(q.empty() is False):
                newCoords = q.get()
                x = newCoords[0]
                y = newCoords[1]
                axis.set_xlabel("(" + str(round(x,4)) + ", " + str(round(y,4)) + ")")

            scat.set_offsets(np.c_[x,y])
            fig.canvas.draw_idle()
            plt.pause(0.1)
            plt.waitforbuttonpress

def main(args=None):
    rclpy.init(args=args)

    node = plot()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()