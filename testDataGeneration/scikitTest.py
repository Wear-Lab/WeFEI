from skinematics.imus import IMU_Base
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Set the in-file, initial sensor orientation 
input = r'testData1.txt'

class RealSenseIMU(IMU_Base):
    def get_data(self, in_file, q_type='analytical'):
        # Read the data
        data = pd.read_csv(in_file, sep=',', index_col=False)
        in_data = {
            'rate': 400,
            'acc': data.filter(regex="Acc").values,
            'omega': data.filter(regex="Gyr").values
        }
        self._set_data(in_data)

if __name__ == '__main__':
    head_sensor = RealSenseIMU(in_file=input)
    plt.plot(head_sensor.quat[:,1:])
    plt.show()
    print('Done')