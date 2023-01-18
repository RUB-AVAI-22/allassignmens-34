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
from avai_messages.msg import BoundingBoxWithRealCoordinates
from avai_messages.msg import BoundingBoxesWithRealCoordinates

class MapDisplayNode(Node):
    main_window = None
    def __init__(self):
        super().__init__('map_display_node')
        print("test")
        self.detectedCones_subscriber = self.create_subscription(String, '/BoundingBoxesWithRealCoordinates', self.callback_detectedConesForMap, 10)
        #self.timer = self.create_timer(1 / 2, self.callback_test)
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [], []

    def callback_detectedConesForMap(self, msg):

        map_values = []

        for bbox in msg.bboxes:
            map_values.append(bbox.real_coords)
        self.main_window.update_plot(map_values)

    def callback_test(self):
        values = []
        values.append([random.randint(-3000,3000)/1000, random.randint(-3000,3000)/1000])
        self.main_window.update_plot(values)

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
            self.figure.clear()
            ax = self.figure.add_subplot()

            df = pd.DataFrame(data)
            dfX = df.iloc[:, 0]
            dfY = df.iloc[:, 1]
            circle = plt.Circle((0,0), 0.1, color='g')
            ax.add_patch(circle)
            ax.scatter(dfX.to_numpy(), dfY.to_numpy())
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
