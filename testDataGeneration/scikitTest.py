# Set the in-file, initial sensor orientation 
in_file = r'testData0.txt'

from skinematics.sensors.manual import MyOwnSensor

# q_type None is for just reading in sensor data, will default to analytical (what we want) if not included
my_sensor_data = MyOwnSensor(in_file, q_type=None)
# my_sensor = MyOwnSensor(in_file)

# Use sampling rate of 400
# Display this IMU's path in a coordinate plane to verify test data works before trying anything crazy