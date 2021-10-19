import numpy as np
import matplotlib.pyplot as plt
from scipy import constants     # for "g"
from scipy.integrate import cumulative_trapezoid
from skinematics import quat, vector, rotmat

# TODO: Can we just pass in one instance of each data instead of a list for omega and accMeasured (rate can be constant or 
# we can measure a new rate every frame depending on how long since the last frame)
def calcFrameAnalytical(
               qref,
               currOmega,
               currAccel,
               prevq,
               prevPosition,
               prevVelo,
               rate=400):
    # Calculation ingores whatever the last omega list is in the 2D array,
    # so we make a dummy omega to make sure our current frame omega is used right
    omega = np.array([currOmega, [0, 0, 0]])

    # Our camera orients itself such that the positive z axis faces out the front of the lens, 
    # the positive y axis points down, and the positive x axis points right

    # Therefore, our standard gravity vector according to this frame is +y
    g_v = np.r_[0, constants.g, 0]
    
    # Calculate orientation q by "integrating" omega -----------------
    nextq = quat.calc_quat(omega, prevq, rate, 'bf')[1]
    # Return of calc_quat is: [prevq_unit, q1] where q is a np array of length 4
    # We get the new q value from q1

    # Acceleration, velocity, and position ----------------------------
    # From q and the measured acceleration, get the {d^2x}/{dt^2}
    accReSensor = currAccel - vector.rotate_vector(g_v, quat.q_inv(nextq))
    accReSpace = vector.rotate_vector(accReSensor, nextq)

    # Make the first position the reference position for this quat
    # TODO: How are we going to keep the calculations using the values before they are aligned to the reference here?
    adjustedq = quat.q_mult(nextq, quat.q_inv(qref))

    # Position and Velocity through integration, assuming 0-velocity at t=0
    vel = np.nan*np.ones_like(accReSpace)
    pos = np.nan*np.ones_like(accReSpace)

    # This is the part more than anything else that I don't think is going to work
    for ii in range(accReSpace.shape[1]):
        vel[ii] = cumulative_trapezoid(accReSpace[ii], dx=1./rate, initial=prevVelo)
        pos[ii] = cumulative_trapezoid(vel[ii], dx=1./rate, initial=prevPosition[ii])

    # Next q used for calculating things
    # adjustdq is what gets stored and used as our output quaternion in the display
        # I'm still not convinced we need it for anything though?
    # Pos and vel are next readings of each
    return (nextq, adjustedq, pos, vel)


    # TODO: Find a way to validate if position and orientation work as expected 
    # (orientation in viewer probably, position just by simple eval)
    # Use a list to load up a preset path, then pass things in one  by one and log the quats and pos/vel into separate lists
    # User viewer to check quat accuracy
    # Check vel and pos directly to see if they make sense



def getInitials(R_initialOrientation=np.eye(3),
               initialAcc=np.r_[0, constants.g, 0]):
    # Transform recordings to angVel/acceleration in space --------------

    # Orientation of \vec{g} with the sensor in the "R_initialOrientation"
    g = constants.g
    g_v = np.r_[0, 0, g]

    # .r_ just makes an array for the dot product to work
    # .inv gets multiplicative inverse of the array
    g0 = np.linalg.inv(R_initialOrientation).dot(g_v)
    # What this is doing is orienting the gravity vector with the initial orientation of the IMU.
    # Either the camera needs to start perfectly horizontal when loading this up,
    # Or we would need a magnatometer to help determine the original orientation, which it doesn't appear is available?

    # For the remaining deviation, assume the shortest rotation to there
    q0 = vector.q_shortest_rotation(initialAcc, g0)
    q_initial = rotmat.convert(R_initialOrientation, to='quat')

    # Combine the two, to form a reference orientation. Note that the sequence
    # is very important!
    q_ref = quat.q_mult(q_initial, q0)

    return q_ref