from skinematics.imus import IMU_Base
from skinematics.view import Orientation_OGL
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Set the in-file, initial sensor orientation 
input = r'testData6.txt'

class RealSenseIMU(IMU_Base):
    def get_data(self, in_data):
        self._set_data(in_data)

if __name__ == '__main__':
    data = pd.read_csv(input, sep=',', index_col=False)
    acc_array = data.filter(regex="Acc").values
    omega_array = data.filter(regex="Gyr").values
    quat_array = []
    index = 0

    print(acc_array)
    print(omega_array)
    
    # Read the data
    data = {
        'rate': 400,
        'acc': acc_array,
        'omega': omega_array
    }

    head_sensor = RealSenseIMU(in_data=data)
    quat_array.append(head_sensor.quat)
    '''
        index += 1
        # acc_array.__len__
        while(index < 10):
            data = {
                'acc': acc_array[index],
                'omega': omega_array[index]
            }

            index += 1
            head_sensor._set_data(data)
            print(head_sensor.quat)
            quat_array.append(head_sensor.quat)
    '''
    print(quat_array)
    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=1000)
    
    print('Done')