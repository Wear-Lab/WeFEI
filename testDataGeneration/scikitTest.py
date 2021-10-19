from skinematics.quat import convert as convertQuatTo
from redefinedMethods import calcFrameAnalytical, getInitials
from skinematics.view import Orientation_OGL
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Set the in-file, initial sensor orientation
input = r'testData0.txt'

class RealSenseIMU():
    def __init__(self):
        self.qref = getInitials()
        self.q = self.qref
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        
    

if __name__ == '__main__':
    headimu = RealSenseIMU()

    csv_data = pd.read_csv(input, sep=',', index_col=False)
    accelArray = csv_data.filter(regex="Acc").values
    omegaArray = csv_data.filter(regex="Gyr").values

    qAdjustedList = []

    for i in range (0, accelArray.__len__):
        (nextq, adjustedq, pos, vel) = calcFrameAnalytical(headimu.qref, omegaArray[i], accelArray[i], headimu.q, headimu.pos, headimu.vel)
        headimu.q = nextq
        headimu.pos = pos
        headimu.vel = vel
        qAdjustedList.append(adjustedq)

    ''' 
    Test qAdjustedList and grab the code that ran the entire thing in the IMU and compare
    viewer = Orientation_OGL(quat_in=head_sensor.quat)
    viewer.run(looping=False, rate=5000)
    '''