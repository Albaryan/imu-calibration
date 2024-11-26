import numpy as np
import logging
import serial
import time
import threading
from scipy.optimize import curve_fit 
from scipy import linalg
from calibration_functions import CalibrationFunctions
import keyboard

"""

This code is for collecting data from ESP32 that connected to both IMU
and PMW3360 modules. It also has calibration process for IMU module. 
Data collection and calibration process is on the same code to reduce 
user complexity. It basically searches for calibration parameters file,
gets calibration params, does calibration and collects data.

"""


class DataCollector:
    
    def __init__(self, port, magnetometer):
        
        """
        First, check if magnetometer will be used
        """

        self.magnetometer=magnetometer

        """
        
        print() functions doesn't work on threading.
        So, we need to use logging to print info on 
        terminal screen
        
        """
        
        self.format = "%(asctime)s: %(message)s"   # Define logging info format              
        logging.basicConfig(format=self.format,  
                            level=logging.INFO,
                            datefmt="%H:%M:%S")
        
        """
        
        Code fails if data is not ready to process. We need to make
        sure that it is ready.
        
        """
        
        self.is_data_ready=False # Define a variable to check if data is ready
        
        """
        
        Collected data must be stored in an array and for ease of use,
        it can also be stored in seperate variables.
        
        """
        
        self.IMU_data=np.zeros((9),dtype=np.float16)    # Define IMU data array
        self.PMW3360_data=np.zeros((4),dtype=np.int16)      # Define PMW3360 data array
        
        self.data_debug=None    # Define a variable to serial data debugging if needed 
        
        self.accel_raw=[]     # Define accelerometer raw data array
        
        self.ax_raw=None      # Define accelerometer raw x axis data variable
        self.ay_raw=None      # Define accelerometer raw y axis data variable
        self.az_raw=None      # Define accelerometer raw z axis data variable
        
        self.gyro_raw=[]      # Define gyroscope raw data array
        
        self.gx_raw=None      # Define gyroscope raw x axis data variable 
        self.gy_raw=None      # Define gyroscope raw y axis data variable
        self.gz_raw=None      # Define gyroscope raw z axis data variable
        
        self.mag_raw=[]       # Define magnetometer raw data array

        if self.magnetometer:
        
            self.mx_raw=None      # Define magnetometer raw x axis data variable
            self.my_raw=None      # Define magnetometer raw y axis data variable
            self.mz_raw=None      # Define magnetometer raw z axis data variable
        
        """
        
        Raw IMU needs to be calibrated. This calibration has parameters
        to process data. We need to define every calibration parameter in 
        variables.
        
        """
        
        self.GYRO_OFFSET_MAT=np.array([     # Define gyro offset calibration parameters variable 
            0.0,                            #   GYRO_X_OFFSET
            0.0,                            #   GYRO_Y_OFFSET
            0.0,                            #   GYRO_Z_OFFSET
        ],dtype=np.float16)
        
        self.ACCEL_FIT_MAT=np.array([       # Define accelerometer lane fitting calibration parameters variable
            [1.0, 0.0],                     #   ACCEL_X_FIT_COEFF
            [1.0, 0.0],                     #   ACCEL_Y_FIT_COEFF
            [1.0, 0.0]                      #   ACCEL_Z_FIT_COEFF
        ])
        

        if self.magnetometer:

            self.MAG_HI_OFFSET=np.array([       # Define magnetometer hard iron calibration parameters variable 
                0.0,                            #   MAG_X_HI_OFFSET
                0.0,                            #   MAG_Y_HI_OFFSET
                0.0                             #   MAG_Z_HI_OFFSET
            ])
            
            self.MAG_SI_OFFSET=np.array([       # Define magnetometer soft iron calibration parameters variable
                [1.0, 0.0, 0.0],                #   MAG_X_SI_OFFSET
                [0.0, 1.0, 0.0],                #   MAG_Y_SI_OFFSET
                [0.0, 0.0, 1.0]                 #   MAG_Z_SI_OFFSET
            ])


        """
        
        Calibrated data must be stored in an array and for ease of use,
        it can also be stored in seperate variables.
        
        """


        self.accel_cal=None     # Define calibrated accelerometer data variable
        
        self.gyro_cal=None      # Define calibrated gyroscope data variable
        
        
        
        self.ax_cal=None        # Define calibrated accelerometer x axis data variable
        self.ay_cal=None        # Define calibrated accelerometer y axis data variable
        self.az_cal=None        # Define calibrated accelerometer z axis data variable
        
        self.gx_cal=None        # Define calibrated gyroscope x axis data variable
        self.gy_cal=None        # Define calibrated gyroscope y axis data variable
        self.gz_cal=None        # Define calibrated gyroscope z axis data variable

        if self.magnetometer:

            self.mag_cal=None       # Define calibrated magnetometer data variable
            
            self.mx_hi_cal=None     # Define hard iron calibrated magnetometer x axis data variable
            self.my_hi_cal=None     # Define hard iron calibrated magnetometer y axis data variable
            self.mz_hi_cal=None     # Define hard iron calibrated magnetometer z axis data variable
            
            self.mx_cal=None        # Define hard + soft iron (fully) calibrated magnetometer x axis data variable
            self.my_cal=None        # Define hard + soft iron (fully) calibrated magnetometer y axis data variable
            self.mz_cal=None        # Define hard + soft iron (fully) calibrated magnetometer z axis data variable
        
        collect_serial_data_thread=threading.Thread(target=self.collect_serial_data, args=(port,)) # Define serial data collection thread
        collect_serial_data_thread.start()                                           # Start serial data collection thread
        
        while not self.is_data_ready:                               # Wait until data gets ready on thread
            print("--- WAITING DATA TO GET READY ---", end='\r')
    
    def read_calibration_params(self, path="calibration_params.txt", reference_mag_value=None):

        self.reference_mag_value=reference_mag_value
        
        """
        
        If there is any calibration done befoe, there must be a calibrations
        file in project folder. Instead of doing a new calibration, params can be
        read from this file.
        
        """
        
        try:
            with open(path,"r") as file:                            # Open calibration file
                calibration_params=(file.read().split('\n'))        # Split data lines

                #### Split all data

                self.ACCEL_FIT_MAT[0]=calibration_params[0:2]
                self.ACCEL_FIT_MAT[1]=calibration_params[2:4]
                self.ACCEL_FIT_MAT[2]=calibration_params[4:6]

                self.GYRO_OFFSET_MAT[:]=calibration_params[6:9]

                if self.magnetometer:

                    self.MAG_HI_OFFSET[:]=calibration_params[9:12]

                    self.MAG_SI_OFFSET[0]=calibration_params[12:15]
                    self.MAG_SI_OFFSET[1]=calibration_params[15:18]
                    self.MAG_SI_OFFSET[2]=calibration_params[18:21]
                
                ####
                
            do_calibration=input("--- CALIBRATION PARAMS FILE FOUND, DO CALIBRATION INSTED? --- Y/n")   # Get input if user needs new calibration 
            
            if do_calibration=="Y": # Do a new calibration if Y is sent on input
                self.calibrate(path=path)    # Do calibration
            
        except Exception as e:  # If exception has occured, do calibration
            self.calibrate(path=path)
        
    def collect_serial_data(self, port):
        
        """
        
        Serial data collection module. Must be called as thread to
        prevent data loss on serial port.
        
        """
        
          
        with serial.Serial(port=port,baudrate=115200) as ser:   # Connect to serial port
            while True:
            
                """
                
                Sometimes serial port closes itself.
                It must be open for communications
                
                """
            
                if not ser.is_open: # If serial port is closed, open
                    ser.open()
    
                try:
                    data=ser.readline().decode()[:-2].split(':')        # Split serial data by ':' character
                    
                    self.is_data_ready=False # Define if serial data is ready, if code fails it is still be False

                    if data[0]=="IMU":  # Check if coming data is IMU data
                        raw_data=data[1].split(' ')
                        self.ax_raw,self.ay_raw,self.az_raw,self.gx_raw,self.gy_raw,self.gz_raw=raw_data[:6] # Split data to raw string values

                        if self.magnetometer:
                            self.mx_raw,self.my_raw,self.mz_raw=raw_data[6:]

                        ### Define acceleration, gyroscope and magnetometer raw value variables

                        self.accel_raw=[self.ax_raw,self.ay_raw,self.az_raw]
                        self.gyro_raw=[self.gx_raw, self.gy_raw, self.gz_raw]

                        if self.magnetometer:
                            self.mag_raw=[self.mx_raw, self.my_raw, self.mz_raw]

                        ###
                        
                        ### Calibrate accelerometer's raw data with calibration parameters on lane fitting and save as an array

                        self.ax_cal=CalibrationFunctions.accel_fit(x_input=float(self.ax_raw),
                                                                   m_x=self.ACCEL_FIT_MAT[0,0],
                                                                   b=self.ACCEL_FIT_MAT[0,1])
                        
                        self.ay_cal=CalibrationFunctions.accel_fit(x_input=float(self.ay_raw),
                                                                   m_x=self.ACCEL_FIT_MAT[1,0],
                                                                   b=self.ACCEL_FIT_MAT[1,1])
                        
                        self.az_cal=CalibrationFunctions.accel_fit(x_input=float(self.az_raw),
                                                                   m_x=self.ACCEL_FIT_MAT[2,0],
                                                                   b=self.ACCEL_FIT_MAT[2,1])
                        
                        self.accel_cal=[self.ax_cal, self.ay_cal, self.az_cal]
                        
                        ###
                        
                        
                        ### Calibrate gyroscope's raw data with offset parameters and save as an array

                        self.gx_cal=float(self.gx_raw)-self.GYRO_OFFSET_MAT[0]
                        self.gy_cal=float(self.gy_raw)-self.GYRO_OFFSET_MAT[1]
                        self.gz_cal=float(self.gz_raw)-self.GYRO_OFFSET_MAT[2]
                        
                        self.gyro_cal=[self.gx_cal, self.gy_cal, self.gz_cal]
                        
                        ###
                        
                        if self.magnetometer:
                            ### Do hard iron and soft iron calibration on magnetometer's raw data with hi and si parameters and save as an array

                            self.mx_hi_cal=float(self.mx_raw)-self.MAG_HI_OFFSET[0]
                            self.my_hi_cal=float(self.my_raw)-self.MAG_HI_OFFSET[1]
                            self.mz_hi_cal=float(self.mz_raw)-self.MAG_HI_OFFSET[2]
                            
                            self.mx_cal=self.mx_hi_cal*self.MAG_SI_OFFSET[0,0] + self.my_hi_cal*self.MAG_SI_OFFSET[0,1] + self.mz_hi_cal*self.MAG_SI_OFFSET[0,2]
                            self.my_cal=self.mx_hi_cal*self.MAG_SI_OFFSET[1,0] + self.my_hi_cal*self.MAG_SI_OFFSET[1,1] + self.mz_hi_cal*self.MAG_SI_OFFSET[1,2]
                            self.mz_cal=self.mx_hi_cal*self.MAG_SI_OFFSET[2,0] + self.my_hi_cal*self.MAG_SI_OFFSET[2,1] + self.mz_hi_cal*self.MAG_SI_OFFSET[2,2]

                            ###NOT Yukarıda matris çarpımı yapılabilecek şekilde düzenleme yapılabilir 
                            
                            self.mag_cal=[self.mx_cal*0.001,self.my_cal*0.001,self.mz_cal*0.001]
                            
                            ###

                        # Save all calibrated data in one array
                        self.IMU_data[:6]=[self.ax_cal,self.ay_cal,self.az_cal,self.gx_cal,self.gy_cal,self.gz_cal]

                        if self.magnetometer:
                            self.IMU_data[6:]=[self.mx_cal,self.my_cal,self.mz_cal]

                        self.is_data_ready=True # Change data's ready state
                    
                except Exception as e:
                    logging.info(f"Exception: {e}") # If exception occured, print
                    
                    self.is_data_ready=False    # Change data state

                time.sleep(0)   # For interrupt to kill while loop on threading if needed
                
    def calibrate(self, path="calibration_params.txt"): 
        
        """
        
        IMU needs to be calibrated carefullly. 
        
            -> Gyroscope is calibrated by collection 1000 values once
            and getting mean of the value
            
            -> Accelerometer is calibrated by rotating every axis upward
            and downward and collecting 1000 data in every axis. Then, calibration
            is done by lane fitting using those values
            
            -> Magnetometer is calibrated by rotating in all direction regardless
            and collecting 1000 unique data in total. Then, calibration is done by
            elipsoid fitting using those values
        
        """        
        
        DATA_COUNT=1000     # Total value to collect
        
        ## Define offsets for every axis' up and down rotation 
        acc_offset={'Z_UP':     [0, 0, 0], 
                    'Z_DOWN':   [0, 0, 0], 
                    'Y_UP':     [0, 0, 0],
                    'Y_DOWN':   [0, 0, 0],
                    'X_UP':     [0, 0, 0],
                    'X_DOWN':   [0, 0, 0]}
        ##
        
        gyro_raw_data=np.empty((0,3))   # Define empty gyroscope data array
        mag_raw_data=[]                 # Define empty magnetometer data array
        
        
        ## Define values to collect accelerometer's every axis 
        acc_x_sum=0
        acc_y_sum=0
        acc_z_sum=0
        ##
        
        
        print("Leave the product steady")
        while not keyboard.is_pressed('e'):
            pass

        i=0
        while i<DATA_COUNT:
            gyro_raw_data=np.append(gyro_raw_data,[[float(self.gx_raw),float(self.gy_raw),float(self.gz_raw)]],axis=0)
            time.sleep(0.005)
            i+=1
            print(i)
        ## Calculate gyroscope offset value
        self.GYRO_OFFSET_MAT[:]=np.mean(gyro_raw_data,axis=0)

        for position in acc_offset.keys():          # Get rotations to calibrate
            while not keyboard.is_pressed('e'):     # Wait for user to get to right rotation
                
                current_pos=None    # Define a variable that holds current rotation value
                
                ## Calculate axis states by rounding accelerometer values
                x_state=round(float(self.ax_raw))
                y_state=round(float(self.ay_raw))
                z_state=round(float(self.az_raw))
                
                if x_state==1 and y_state==0 and z_state==0:
                    current_pos="X_DOWN"
                    
                elif x_state==-1 and y_state==0 and z_state==0:
                    current_pos="X_UP"
                    
                elif x_state==0 and y_state==1 and z_state==0:
                    current_pos="Y_DOWN"
                    
                elif x_state==0 and y_state==-1 and z_state==0:
                    current_pos="Y_UP"
                    
                elif x_state==0 and y_state==0 and z_state==1:
                    current_pos="Z_DOWN"
                    
                elif x_state==0 and y_state==0 and z_state==-1:
                    current_pos="Z_UP"
                    
                else:
                    current_pos="ROTATING"
                
                print(f"ROTATE MODULE TO GET {position}, CURRENT POSITION = {current_pos} :",
                      x_state,
                      y_state,
                      z_state,
                      round(self.gx_cal,2),
                      round(self.gy_cal,2),
                      round(self.gz_cal,2),
                      " "*10,
                      end='\r')
                
                ##
            
            ## Collect raw data until collected data reaches to data_count value
            i=0
            while i<DATA_COUNT:
                if self.IMU_data[0] != 0 and self.IMU_data[1] != 0 and self.IMU_data[2]!=0 and round(self.gx_cal,2)==0 and round(self.gy_cal,2)==0 and round(self.gz_cal,2)==0:
                    acc_x_sum+=float(self.ax_raw)
                    acc_y_sum+=float(self.ay_raw)
                    acc_z_sum+=float(self.az_raw)
                    i+=1

                print("COLLECTING RAW DATA :",
                      f"%{round(i/DATA_COUNT*100)}",
                      float(self.ax_raw),
                      float(self.ay_raw),
                      float(self.az_raw),
                      self.gx_cal,
                      self.gy_cal,
                      self.gz_cal,
                      " "*100,
                      end='\r')
                time.sleep(0.005)  # time.sleep() is to reduce collection speed of raw values
            ##

            ## Print offset data to console
            print(" "*100)
            print(f"--- OFFSETS FOR {position} ---")
            acc_offset[position]=[acc_x_sum/DATA_COUNT,acc_y_sum/DATA_COUNT,acc_z_sum/DATA_COUNT]
            print("OFFSETS: ",*acc_offset[position])
            acc_x_sum,acc_y_sum,acc_z_sum=0,0,0
            ###

        ## Calculate accelerometer lane fitting params
        self.ACCEL_FIT_MAT[0],_ = curve_fit(CalibrationFunctions.accel_fit,np.append(np.append(acc_offset["X_UP"][0],
                                 acc_offset["X_DOWN"][0]),acc_offset["Z_UP"][0]),
                   np.append(np.append(1.0*np.ones(np.shape(acc_offset["X_UP"][0])),
                    -1.0*np.ones(np.shape(acc_offset["X_DOWN"][0]))),
                        0.0*np.ones(np.shape(acc_offset["Z_UP"][0]))),
                            maxfev=10000)

        self.ACCEL_FIT_MAT[1],_ = curve_fit(CalibrationFunctions.accel_fit,np.append(np.append(acc_offset["Y_UP"][1],
                                 acc_offset["Y_DOWN"][1]),acc_offset["Z_UP"][1]),
                   np.append(np.append(1.0*np.ones(np.shape(acc_offset["Y_UP"][1])),
                    -1.0*np.ones(np.shape(acc_offset["Y_DOWN"][1]))),
                        0.0*np.ones(np.shape(acc_offset["Z_UP"][1]))),
                            maxfev=10000)

        self.ACCEL_FIT_MAT[2],_ = curve_fit(CalibrationFunctions.accel_fit,np.append(np.append(acc_offset["Z_UP"][2],
                                 acc_offset["Z_DOWN"][2]),acc_offset["X_UP"][2]),
                   np.append(np.append(1.0*np.ones(np.shape(acc_offset["Z_UP"][2])),
                    -1.0*np.ones(np.shape(acc_offset["Z_DOWN"][2]))),
                        0.0*np.ones(np.shape(acc_offset["X_UP"][2]))),
                            maxfev=10000)
        ##
        ##

        if self.magnetometer:
        
            ## Collect every unique magnetometer data 
            while len(mag_raw_data)<DATA_COUNT:
                print(f"COLLECTING MAG DATA : %{round(len(mag_raw_data)/DATA_COUNT*100)}", '\t', len(mag_raw_data))
                if [float(self.mx_raw),float(self.my_raw),float(self.mz_raw)] not in mag_raw_data and (round(self.gx_cal,2)!=0 and round(self.gy_cal,2)!=0 and round(self.gz_cal,2)!=0):
                    mag_raw_data.append([float(self.mx_raw),float(self.my_raw),float(self.mz_raw)]) # COLLECT RAW MAGNETOMETER DATA WHILE WAITING
            ##

            ## Apply elipsoid fitting on magnetometer data
            s = np.array(mag_raw_data).T
            M, n, d = CalibrationFunctions.ellipsoid_fit(s)

            M_1 = linalg.inv(M)
            
            # rize 82 mpu9250
            # trabzon 7891.504 deneyap

            HI_OFFSET = -np.dot(M_1, n)
            SI_OFFSET = np.real(self.reference_mag_value / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

            self.MAG_HI_OFFSET[:]=HI_OFFSET.T[0]
            self.MAG_SI_OFFSET=SI_OFFSET
            ##

        with open(path,'a+') as file:                   # Open calibration parameters file
            file.truncate(0)                            # Clear data inside calibration parameters file

            
            print("Calibration is done!")
            
            ## Write calibration params to file
            for i in self.ACCEL_FIT_MAT:
                for j in i:
                    file.write(f"{j}\n")
                    
            for i in self.GYRO_OFFSET_MAT:
                file.write(f"{i}\n")

            if self.magnetometer:
                
                for i in self.MAG_HI_OFFSET:
                    file.write(f"{i}\n")
                    
                for i in self.MAG_SI_OFFSET:
                    for j in i:
                        file.write(f"{j}\n")
            ##
                    
    def calibrate_mag(self, reference_mag_value, path="calibration_params.txt"):
        
        """
            Accelerometer and gyroscope calibration can be done once unless
            there is drift. Magnetometer calibration is very sensitive to 
            any ferromagnetic material around. If there is not sensor drift
            on accelerometer and gyroscope values, magnetometer could be 
            calibrated
        """
        
        mag_raw_data=[]
        
        DATA_COUNT=1000
        
        with open(path,'r+') as file: 
            params=file.read().split("\n")[:9]
            
        ## Collect every unique magnetometer data 
        while len(mag_raw_data)<DATA_COUNT:
            print(f"COLLECTING MAG DATA : %{round(len(mag_raw_data)/DATA_COUNT*100)}", '\t', len(mag_raw_data),                      
                      round(self.gx_cal,2),
                      round(self.gy_cal,2),
                      round(self.gz_cal,2),
                      " "*100,
                      end='\r')
            if [float(self.mx_raw),float(self.my_raw),float(self.mz_raw)] not in mag_raw_data and (round(self.gx_cal,2)!=0 and round(self.gy_cal,2)!=0 and round(self.gz_cal,2)!=0):
                mag_raw_data.append([float(self.mx_raw),float(self.my_raw),float(self.mz_raw)]) # COLLECT RAW MAGNETOMETER DATA WHILE WAITING
                time.sleep(0.05)
        ##
        s = np.array(mag_raw_data).T
        M, n, d = CalibrationFunctions.ellipsoid_fit(s)
        M_1 = linalg.inv(M)
        HI_OFFSET = -np.dot(M_1, n)
        SI_OFFSET = np.real(reference_mag_value / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        self.MAG_HI_OFFSET[:]=HI_OFFSET.T[0]
        self.MAG_SI_OFFSET=SI_OFFSET
        
        with open(path,"r+") as file:
            file.truncate(0)
            
            for param in params:
                file.write(f"{param}\n")
                
            for i in self.MAG_HI_OFFSET:
                file.write(f"{i}\n")
                
            for i in self.MAG_SI_OFFSET:
                for j in i:
                    file.write(f"{j}\n")
                    
                    
    