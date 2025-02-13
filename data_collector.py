from ahrs import Quaternion,DCM,RAD2DEG
import numpy as np
import imufusion
import serial
import json
import time
import threading

class DataCollector:

    def __init__(self, port):
        self.ahrs=imufusion.Ahrs()
        self.offset = imufusion.Offset(100)

        self.time_now=time.time()

        self.is_data_ready=False

        self.gy_offset=np.zeros(3)

        collect_serial_data_thread=threading.Thread(target=self.collect_serial_data, args=(port,)) # Define serial data collection thread
        collect_serial_data_thread.start()  

        while not self.is_data_ready:                               # Wait until data gets ready on thread
            print("--- WAITING DATA TO GET READY ---", end='\r')

        self.calibrate_gyro()

    def collect_serial_data(self, port):
        while True:
            with serial.Serial(port,115200) as ser:
                try:
                    self.data=json.loads(ser.readline())
                    self.data["GYR"]=self.offset.update(np.array(self.data["GYR"]-self.gy_offset))
                    self.ahrs.update_no_magnetometer(np.array(self.data["GYR"]),
                                                     np.array(self.data["ACC"]),
                                                     time.time()-self.time_now)
                    self.time_now=time.time()
                    self.w,self.x,self.y,self.z=self.ahrs.quaternion.wxyz
                    self.Q=Quaternion([self.w,self.x,self.y,self.z])
                    self.roll,self.pitch,self.yaw=DCM(self.Q.to_DCM()).to_rpy()*RAD2DEG
                    self.is_data_ready=True
                except Exception as e:
                    print(e)

    def get_rpy(self):
        return self.roll, self.pitch, self.yaw
    
    def calibrate_gyro(self):
        gy_x,gy_y,gy_z=0,0,0
        counter=0

        while counter<1000:
            gy_x+=self.data["GYR"][0]
            gy_y+=self.data["GYR"][1]
            gy_z+=self.data["GYR"][2]
            counter+=1
            time.sleep(0.01)
            print("Doing calibration\t%",counter/10)
        
        gy_x/=1000
        gy_y/=1000
        gy_z/=1000

        self.gy_offset[0]=gy_x
        self.gy_offset[1]=gy_y
        self.gy_offset[2]=gy_z