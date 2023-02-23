import math
import random
import sys

import matplotlib
import rclpy
from matplotlib import colors
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from qt_gui.main_window import MainWindow
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from std_msgs.msg import String
from threading import Thread
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pandas as pd
from avai_messages.msg import Map
from avai_messages.msg import MapEntry
from nav_msgs.msg import Odometry

class MapDisplayNode(Node):
    current_pos = (0, 0)
    print("s")
    main_window = None
    def __init__(self):
        super().__init__('map_display_node')
        self.map_subscriber = self.create_subscription(Map, '/map', self.callback_map, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.current_angle = 0
        self.current_pos = (0,0)
        self.get_logger().info("Map Display Node started!")
        #testing purpose
        #self.timer = self.create_timer(1 , self.debug)

    def callback_odom(self, msg):
        self.get_logger().info("Odometry Data received!")
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def callback_map(self, msg):
        self.get_logger().info("Map Data received!")
        map = np.empty((len(msg.map_objects), 3))
        for i, map_object in enumerate(msg.map_objects):
            map_object_extracted = np.empty(3)
            map_object_extracted[:2] = map_object.coordinates
            map_object_extracted[2] = map_object.cls
            map[i] = map_object_extracted
        self.main_window.update_plot(map)

    def debug(self):
        pub = self.create_publisher(Map, '/map', 10)
        test_entry = MapEntry()
        test_entry.coordinates = [float(random.randint(-3,3)), float(random.randint(-3,3))]
        test_entry.cls = random.randint(0,2)
        data = Map()
        data.map_objects = [test_entry]
        pub.publish(data)

class GUI(QWidget):
    node: MapDisplayNode = None

    def __init__(self):
        super(GUI, self).__init__()

        self.VBL = QVBoxLayout()
        self.figure = plt.figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.VBL.addWidget(self.canvas)
        self.setLayout(self.VBL)
        self.graph_lim = 3
        self.colors = ['#0000FF', '#FF7800', '#FFFF00']

    def update_plot(self, data):
        if len(data) != 0 :
            print("Updating Map Display!")
            self.figure.clear()
            ax = self.figure.add_subplot()
            dataX = data[:, 0]
            dataY = data[:, 1]
            dataCls = data[:, 2]
            colors_points = []
            for cls in dataCls:
                colors_points.append(self.colors[int(cls)])
            circle = plt.Circle(MapDisplayNode.current_pos, 0.1, color='purple')

            if odom_msg.pose.pose.orientation.x > 0:
                MapDisplayNode.current_angle = round(math.acos(odom_msg.pose.pose.orientation.w) * 180 / math.pi * 2, 1)
            elif odom_msg.pose.pose.orientation.x < 0:
                MapDisplayNode.current_angle = round(-math.acos(odom_msg.pose.pose.orientation.w) * 180 / math.pi * 2, 1)

            arrow = plt.arrow(MapDisplayNode.current_pos[0], MapDisplayNode.current_pos[1],   math.cos(MapDisplayNode.current_angle), math.sin(MapDisplayNode.current_angle))
            ax.add_patch(circle)
            ax.scatter(dataX, dataY, c=colors_points)
            ax.grid()
            ax.set_xlim([-self.graph_lim + MapDisplayNode.current_pos[0], self.graph_lim + MapDisplayNode.current_pos[0]])
            ax.set_ylim([-self.graph_lim + MapDisplayNode.current_pos[1], self.graph_lim + MapDisplayNode.current_pos[1]])
            self.canvas.draw()


def main(args=None):
    rclpy.init(args=args)
    node = MapDisplayNode()
    app = QApplication(sys.argv)
    main_window = GUI()

    node.main_window = main_window
    main_window.node = node

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    main_window.show()
    thread = Thread(target=executor.spin)
    thread.start()
    try:
        sys.exit(app.exec())
    finally:
        node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
