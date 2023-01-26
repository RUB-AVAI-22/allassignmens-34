import random
import sys

import matplotlib
import rclpy
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

class MapDisplayNode(Node):
    main_window = None
    def __init__(self):
        super().__init__('map_display_node')
        self.map_subscriber = self.create_subscription(Map, '/map', self.callback_map, 10)
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [], []
        self.map = np.zeros((1,3))
        self.get_logger().info("Map Display Node started!")

    def callback_map(self, msg):
        self.get_logger().info("Map Data received!")
        map = np.empty((len(msg.map_objects), 3))
        for i, map_object in enumerate(msg.map_objects):
            map_object_extracted = np.empty(3)
            map_object_extracted[:2] = map_object.coordinates
            map_object_extracted[2] = map_object.cls
            map[i] = map_object_extracted
        self.main_window.update_plot(map)

class GUI(QWidget):
    node: MapDisplayNode = None

    def __init__(self):
        super(GUI, self).__init__()

        self.VBL = QVBoxLayout()
        self.figure = plt.figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.VBL.addWidget(self.canvas)
        self.setLayout(self.VBL)

    def update_plot(self, data):
        if len(data) != 0 :
            print("Updating Map Display!")
            self.figure.clear()
            ax = self.figure.add_subplot()

            dataX = data[:, 0]
            dataY = data[:, 1]
            circle = plt.Circle((0,0), 0.1, color='g')
            ax.add_patch(circle)
            ax.scatter(dataX, dataY)
            ax.grid()
            ax.set_xlim([-3,3])
            ax.set_ylim([-3,3])
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
