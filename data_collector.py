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

        self.hi_offset=np.zeros((3))

        self.time_diff=0

        self.gy_calibrated=False

        collect_serial_data_thread=threading.Thread(target=self.collect_serial_data, args=(port,)) # Define serial data collection thread
        collect_serial_data_thread.start()  

        while not self.is_data_ready:                               # Wait until data gets ready on thread
            print("--- WAITING DATA TO GET READY ---", end='\r')

        self.calibrate_gyro()

        time.sleep(1)

        #self.calibrate_mag()

    def collect_serial_data(self, port):
        with serial.Serial(port,115200) as ser:
            while True:

                if not ser.is_open: # If serial port is closed, open
                    ser.open()
                try:
                    data=json.loads(ser.readline())
                    if self.gy_calibrated:
                        data["GYR"]=self.offset.update(np.array(data["GYR"]-self.gy_offset))
                        self.data=data
                        self.ahrs.update(np.array(self.data["GYR"]),
                                                    np.array(self.data["ACC"]),
                                                    np.array(self.data["MAG"])-self.hi_offset,
                                                    time.time()-self.time_now)
                        self.time_now=time.time()
                    
                        self.w,self.x,self.y,self.z=self.ahrs.quaternion.wxyz
                        self.Q=Quaternion([self.w,self.x,self.y,self.z])
                        self.roll,self.pitch,self.yaw=DCM(self.Q.to_DCM()).to_rpy()*RAD2DEG
                        self.is_data_ready=True
                    else:
                        self.data=data
                        self.is_data_ready=True
                except Exception as e:
                    print(e)
                    self.is_data_ready=False

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
            print("Gyro calibration, keep steady\t%",counter/10)
            time.sleep(0.005)
        
        gy_x/=1000
        gy_y/=1000
        gy_z/=1000

        self.gy_offset[0]=gy_x
        self.gy_offset[1]=gy_y
        self.gy_offset[2]=gy_z

        self.gy_calibrated=True

    def calibrate_mag(self):
        rls_theta=np.zeros((4,1))
        rls_p=np.eye(4) * 1000
        rls_in=np.zeros((4,1))
        rls_out=0
        rls_gain=np.zeros((4,1))
        rls_lambda=0.999

        covar=np.dot( np.diag(rls_p).T , np.diag(rls_p) )

        while not covar<1e-4:


            mag_x,mag_y,mag_z=self.data["MAG"]
            gyr_x, gyr_y, gyr_z= self.data["GYR"]
            print(round(gyr_x),'\t',round(gyr_y),'\t',round(gyr_z),end='\t')
            if (round(gyr_x)!=0 or round(gyr_y)!=0 or round(gyr_z)!=0):

                rls_in[0][0] = mag_x
                rls_in[1][0] = mag_y
                rls_in[2][0] = mag_z
                rls_in[3][0] = 1

                rls_out = (mag_x**2) + (mag_y**2) + (mag_z**2)

                err = (rls_out - (rls_in.T @ rls_theta))[0][0]

                rls_gain = (rls_p @ rls_in) / (rls_lambda + (rls_in.T @ rls_p @ rls_in))


                rls_p = (rls_p - ((rls_gain @ rls_in.T) * rls_p)) / rls_lambda

                rls_theta = rls_theta + err * rls_gain

                covar = np.diag(rls_p).T @ np.diag(rls_p) 

                print(covar, "\t", rls_theta.T[0][:3],end='')

                time.sleep(0.1)
            # else:
            #     print(gyr_x, gyr_y, gyr_z)

            print()

        self.hi_offset=(rls_theta.T[0][:3]/2)

             
