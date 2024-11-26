from ahrs.filters import Madgwick, SAAM
from ahrs.common.orientation import acc2q
from ahrs.common.quaternion import Quaternion
from ahrs import RAD2DEG
from ahrs import DCM
import time

class OrientationFilters:
    def __init__(self, magnetometer, accel=[], mag=[]):
        
        """
            Orientation values of an IMU is obtained by
            filtering every axis and calculating orientation.
            In this code Madgwick filter is used to calculate
            orientation from IMU sensor values.
            
        """
        
        self.madgwick_filter=Madgwick() # Define madgwick filter
        self.magnetometer=magnetometer # Define if magnetometer is used
        if magnetometer: 
            self.saam=SAAM() # Define a SAAM filter to calculate initial orientation using magnetometer and accelerometer
            self.Q=Quaternion(self.saam.estimate( # Calculate initial orientation using magnetometer and accelerometer
                acc=accel,
                mag=mag
            ))
        else:
            self.Q = Quaternion(acc2q(accel))  # Calculate initial orientation using accelerometer
        
        self.YAW_OFFSET=0      # Define yaw offset value 
        self.pTime=time.time() # Define a time value
        
    def set_yaw_offset(self):
        self.YAW_OFFSET=((DCM(self.Q.to_DCM()).to_rpy())*RAD2DEG)[2]
        
    def get_yaw_offset(self):
        return self.YAW_OFFSET
    
    def set_mag_usage(self,state):
        self.magnetometer=state
    
    def get_mag_usage(self):
        return self.magnetometer   
    
    def get_rpy(self):
        return self.roll, self.pitch, self.yaw
    
    def get_angles(self):
        
        """

            In ultrasound project, every 90 degree rotation should
            be detected
        
        """
        
        ## Detect every values on 90 degre rotation
        self.angle_roll=None
        self.angle_pitch=None
        self.angle_yaw=None
        
        if self.roll<100 and self.roll>80:
            self.angle_roll=90
        elif self.roll>-100 and self.roll<-80:
            self.angle_roll=270
        elif self.roll<10 and self.roll>-10:
            self.angle_roll=0
        elif self.roll>170 or self.roll<-170:
            self.angle_roll=180
            
        if self.pitch<90 and self.pitch>80:
            self.angle_pitch=90
        elif self.pitch>-90 and self.pitch<-80:
            self.angle_pitch=270
        elif self.pitch<10 and self.pitch>-10:
            self.angle_pitch=0
        elif self.pitch>170 or self.pitch<-170:
            self.angle_pitch=180
            
        if self.yaw<90 and self.yaw>80:
            self.angle_yaw=90
        elif self.yaw>-90 and self.yaw<-80:
            self.angle_yaw=270
        elif self.yaw<10 and self.yaw>-10:
            self.angle_yaw=0
        elif self.yaw>170 or self.yaw<-170:
            self.angle_yaw=180
            
        ##
            
        return self.angle_roll,self.angle_pitch,self.angle_yaw
        
    def calculate_rpy(self, gyro=[], accel=[], mag=[]):
        
        """
            Calculation of roll, pitch and yaw 
        """
        
        try:
            self.cTime=time.time()
            self.madgwick_filter.Dt=(self.cTime-self.pTime) # Set elapsed time on Madgwick Filter
            self.pTime=self.cTime
            if self.magnetometer:   # Use calculation with magnetometer
                self.Q=self.madgwick_filter.updateMARG(self.Q,
                                                  gyro,
                                                  accel,
                                                  mag
                                                  )

            else: # Use calculation without magnetometer
                self.Q=self.madgwick_filter.updateIMU(self.Q,
                                                      gyro,
                                                      accel
                                                      )
            self.roll,self.pitch,self.yaw=(DCM(self.Q.to_DCM()).to_rpy())*RAD2DEG
            self.yaw-=self.YAW_OFFSET
            
            return True
        except Exception as e:
            print(e)
            
            return False
        
    
    
    