from skinematics.imus import IMU_Base, analytical
from skinematics.quat import convert as convertQuatTo
from skinematics.view import Orientation_OGL
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Set the in-file, initial sensor orientation
input = r'testData0.txt'

class RealSenseIMU(IMU_Base):
    def get_data(self, in_data):
        self._set_data(in_data)

if __name__ == '__main__':
    csv_data = pd.read_csv(input, sep=',', index_col=False)
    acc_array = csv_data.filter(regex="Acc").values
    omega_array = csv_data.filter(regex="Gyr").values

    data = {
        'rate': 400,
        'acc': acc_array[:len(acc_array)//2],
        'omega': omega_array[:len(omega_array)//2]
    }

    head_sensor = RealSenseIMU(in_data=data)
    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=5000)

    data = {
        'rate': 400,
        'acc': acc_array[len(acc_array)//2:],
        'omega': omega_array[len(omega_array)//2:]
    }

    q, pos, vel = analytical(convertQuatTo(head_sensor.quat, 'rotmat'), data['omega'], head_sensor.pos, data['acc'], data['rate'])
    head_sensor.quat = q
    head_sensor.pos = pos

    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=5000)