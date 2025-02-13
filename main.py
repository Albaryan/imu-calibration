from demo_visualizer import DemoVisualizer
from data_collector import DataCollector
from ahrs import Quaternion,DCM,RAD2DEG
import numpy as np
import imufusion
import serial
import json
import time

port="/dev/ttyACM0"
data_collector=DataCollector(port)
demo_visualizer=DemoVisualizer()
    
while True:
    roll,pitch,yaw=data_collector.get_rpy()
    demo_visualizer.visualize_rpy(roll,pitch,yaw)
