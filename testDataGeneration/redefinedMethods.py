import numpy as np
import matplotlib.pyplot as plt
from scipy import constants     # for "g"
from scipy.integrate import cumtrapz
from skinematics import quat, vector, rotmat

def calcOrientation(imu):
    # TODO: Remove and use the below method directly once fixed. This is just to demo what gets passed in.
    initialPosition = np.r_[0,0,0]
    (quaternion, position, velocity) = calcFrameAnalytical(imu.R_init, imu.omega, initialPosition, imu.acc, imu.rate)
    imu.quat = quaternion

# TODO: Can we just pass in one instance of each data instead of a list for omega and accMeasured (rate can be constant or 
# we can measure a new rate every frame depending on how long since the last frame)
def calcFrameAnalytical(R_initialOrientation=np.eye(3),
               omega=np.zeros((5,3)),
               initialPosition=np.zeros(3),
               accMeasured=np.column_stack((np.zeros((5,2)), 9.81*np.ones(5))),
               rate=100):
    ''' Reconstruct position and orientation with an analytical solution,
    from angular velocity and linear acceleration.
    Assumes a start in a stationary position. No compensation for drift.
    Parameters
    ----------
    R_initialOrientation: ndarray(3,3)
        Rotation matrix describing the initial orientation of the sensor,
        except a mis-orienation with respect to gravity
    omega : ndarray(N,3)
        Angular velocity, in [rad/s]
    initialPosition : ndarray(3,)
        initial Position, in [m]
    accMeasured : ndarray(N,3)
        Linear acceleration, in [m/s^2]
    rate : float
        sampling rate, in [Hz]
    Returns
    -------
    q : ndarray(N,3)
        Orientation, expressed as a quaternion vector
    pos : ndarray(N,3)
        Position in space [m]
    vel : ndarray(N,3)
        Velocity in space [m/s]
    Example
    -------
     
    >>> q1, pos1 = analytical(R_initialOrientation, omega, initialPosition, acc, rate)
    '''

    # Checks to make sure that this is an orientation matrix
    if omega.ndim == 1:
        raise ValueError('The input to "analytical" requires matrix inputs.')
    
    # Transform recordings to angVel/acceleration in space --------------

    # Orientation of \vec{g} with the sensor in the "R_initialOrientation"
    g = constants.g
    # .r_ just makes an array for the dot product to work
    # .inv gets multiplicative inverse of the array
    g0 = np.linalg.inv(R_initialOrientation).dot(np.r_[0,0,g])
    # What this is doing is orienting the gravity vector with the initial orientation of the IMU.
    # Either the camera needs to start perfectly horizontal when loading this up,
    # Or we would need a magnatometer to help determine the original orientation, which it doesn't appear is available?

    # For the remaining deviation, assume the shortest rotation to there
    q0 = vector.q_shortest_rotation(accMeasured[0], g0)    
    q_initial = rotmat.convert(R_initialOrientation, to='quat')
    
    # Combine the two, to form a reference orientation. Note that the sequence
    # is very important!
    q_ref = quat.q_mult(q_initial, q0)
    
    # Calculate orientation q by "integrating" omega -----------------
    q = quat.calc_quat(omega, q_ref, rate, 'bf')

    # Acceleration, velocity, and position ----------------------------
    # From q and the measured acceleration, get the {d^2x}/{dt^2}
    g_v = np.r_[0, 0, g]
    # This should be safe as well, given that it's rotating wrt the q orientation quaternion
    accReSensor = accMeasured - vector.rotate_vector(g_v, quat.q_inv(q))
    accReSpace = vector.rotate_vector(accReSensor, q)

    # Make the first position the reference position
    q = quat.q_mult(q, quat.q_inv(q[0]))

    # compensate for drift - commented out in original source
    # TODO: Try this once we think the code is working
    # drift = np.mean(accReSpace, 0)
    # accReSpace -= drift*0.7

    # Position and Velocity through integration, assuming 0-velocity at t=0
    # TODO: How do we convert this to one frame at a time instead of a list?
    vel = np.nan*np.ones_like(accReSpace)
    pos = np.nan*np.ones_like(accReSpace)

    for ii in range(accReSpace.shape[1]):
        vel[:,ii] = cumtrapz(accReSpace[:,ii], dx=1./rate, initial=0)
        pos[:,ii] = cumtrapz(vel[:,ii], dx=1./rate, initial=initialPosition[ii])

    return (q, pos, vel)
    # TODO: Find a way to validate if position and orientation work as expected 
    # (orientation in viewer probably, position just by simple eval)

    # TODO: Can we do initial orientation and drift compenstation with some of the code from:
    # https://dev.intelrealsense.com/docs/rs-motion