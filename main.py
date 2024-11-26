from data_collector import DataCollector
from orientation_filters import OrientationFilters
from demo_visualizer import DemoVisualizer
import keyboard


port='/dev/ttyACM0'
magnetometer=True
path='deneyap_calibration_params.txt'
reference_mag_value=49.3354

data_collector=DataCollector(port=port, magnetometer=magnetometer)

data_collector.read_calibration_params(path=path, reference_mag_value=reference_mag_value)

#data_collector.calibrate_mag(reference_mag_value=49.3354,path=path)

orientation_filters=OrientationFilters(magnetometer=magnetometer, accel=data_collector.accel_cal, 
                                       mag=data_collector.mag_cal
                                       )



demo_visualizer=DemoVisualizer()


while True:
    if orientation_filters.calculate_rpy(data_collector.gyro_cal, data_collector.accel_cal, 
                                         data_collector.mag_cal
                                         ):
        roll,pitch,yaw=orientation_filters.get_rpy()
        demo_visualizer.visualize_rpy(float(roll),float(pitch),float(yaw))



