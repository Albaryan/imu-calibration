from demo_visualizer import DemoVisualizer
from data_collector import DataCollector
from ahrs import Quaternion,DCM,RAD2DEG
import numpy as np
import imufusion
import serial
import json
import time

port="/dev/ttyACM1"
data_collector=DataCollector(port)
demo_visualizer=DemoVisualizer()
    
while True:
    roll,pitch,yaw=data_collector.get_rpy()

    # print(data_collector.data["GYR"])
    demo_visualizer.visualize_rpy(roll,pitch,yaw)
