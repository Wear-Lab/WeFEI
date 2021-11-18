import math
from redefinedMethods import calcFrameAnalytical, getInitials
import pandas as pd
import numpy as np
from skinematics.imus import IMU_Base
from skinematics.view import Orientation_OGL

# Set the in-file, initial sensor orientation
input = r'testData0.txt'

class RealSenseIMU():
    def __init__(self):
        self.qref = getInitials()
        self.q = self.qref
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        
class SKIMU(IMU_Base):
    def get_data(self, in_data):
        self._set_data(in_data)

if __name__ == '__main__':

    # Testing new version for accuracy
    headimu = RealSenseIMU()

    csv_data = pd.read_csv(input, sep=',', index_col=False)
    accelArray = csv_data.filter(regex="Acc").values
    omegaArray = csv_data.filter(regex="Gyr").values

    qAdjustedList = []

    for i in range (0, len(accelArray)):
        (nextq, adjustedq, pos, vel) = calcFrameAnalytical(headimu.qref, omegaArray[i], accelArray[i], headimu.q, headimu.pos, headimu.vel)
        headimu.q = nextq
        headimu.pos = pos
        headimu.vel = vel
        qAdjustedList.append(adjustedq)

    # Old version
    data = pd.read_csv(input, sep=',', index_col=False)
    acc_array = data.filter(regex="Acc").values
    omega_array = data.filter(regex="Gyr").values

    # Read the data
    data = {
        'rate': 400,
        'acc': acc_array,
        'omega': omega_array
    }

    head_sensor = SKIMU(in_data=data)
    
    
    '''
    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=1000)
    '''


    print("Lengths: ", len(qAdjustedList), " for the new one, ", len(head_sensor.quat), " for the old one.")
    
    print("Differences:")
    i = 0
    rangeMax = max(len(qAdjustedList), len(head_sensor.quat))
    while i < rangeMax:
        if i < 50 or i > 2100:
            print("i = ", i, ": ", qAdjustedList[i], " vs ", head_sensor.quat[i])
        elif i == 51:
            print("...\n...\n...")
        i+=1
        

    print('Done')

    ''' 
    Test qAdjustedList and grab the code that ran the entire thing in the IMU and compare
    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=5000)
    '''