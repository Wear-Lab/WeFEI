import numpy as np
from skinematics.quat import unit_q, q_mult, q_inv
import time

def calc_quat(omega, q0, rate, CStype):
    '''
    Take an angular velocity (in rad/s), and convert it into the
    corresponding orientation quaternion.
    Parameters
    ----------
    omega : array, shape (3,) or (N,3)
        angular velocity [rad/s].
    q0 : array (3,)
        vector-part of quaternion (!!)
    rate : float
        sampling rate (in [Hz])
    CStype:  string
        coordinate_system, space-fixed ("sf") or body_fixed ("bf")
    Returns
    -------
    quats : array, shape (N,4)
        unit quaternion vectors.
    Notes
    -----
    1) The output has the same length as the input. As a consequence, the last velocity vector is ignored.
    2) For angular velocity with respect to space ("sf"), the orientation is given by
      .. math::
          q(t) = \\Delta q(t_n) \\circ \\Delta q(t_{n-1}) \\circ ... \\circ \\Delta q(t_2) \\circ \\Delta q(t_1) \\circ q(t_0)
      .. math::
        \\Delta \\vec{q_i} = \\vec{n(t)}\\sin (\\frac{\\Delta \\phi (t_i)}{2}) = \\frac{\\vec \\omega (t_i)}{\\left| {\\vec \\omega (t_i)} \\right|}\\sin \\left( \\frac{\\left| {\\vec \\omega ({t_i})} \\right|\\Delta t}{2} \\right)
    3) For angular velocity with respect to the body ("bf"), the sequence of quaternions is inverted.
    4) Take care that you choose a high enough sampling rate!
    Examples
    --------
    >>> v0 = np.r_[0., 0., 100.] * np.pi/180.
    >>> omega = np.tile(v0, (1000,1))
    >>> rate = 100
    >>> out = quat.calc_quat(omega, [0., 0., 0.], rate, 'sf')
    array([[ 1.        ,  0.        ,  0.        ,  0.        ],
       [ 0.99996192,  0.        ,  0.        ,  0.00872654],
       [ 0.9998477 ,  0.        ,  0.        ,  0.01745241],
       ..., 
       [-0.74895572,  0.        ,  0.        ,  0.66262005],
       [-0.75470958,  0.        ,  0.        ,  0.65605903],
       [-0.76040597,  0.        ,  0.        ,  0.64944805]])
    '''
    
    start = time.time()

    # If omega isn't 2D (more than one reading) adds an outer array to make it @D ([[omega0]])
    omega_05 = np.atleast_2d(omega).copy()

    print("omega_05\n", omega_05)
    
    # The following is (approximately) the quaternion-equivalent of the trapezoidal integration (cumtrapz)
    # [:-t] excludes the last element
    if omega_05.shape[1]>1:
        omega_05[:-1] = 0.5*(omega_05[:-1] + omega_05[1:])

    print("->", omega_05)

    omega_t = np.sqrt(np.sum(omega_05**2, 1))
    print("omega_t\n", omega_t)
    omega_nonZero = omega_t>0 # Each value in omega_t (1D) is nonZero -> 1D array with True or False
    print("omega_nonZero\n", omega_nonZero)

    # initialize the quaternion
    q_delta = np.zeros(omega_05.shape) # Make a 2D array with same shape as omega_05 but all zeros
    q_pos = np.zeros((len(omega_05),4)) # 2D array with len(omega_05) rows and each row having 4 zero elements
    q_pos[0,:] = unit_q(q0) # Set the first q value to the unit quat of the initial quaternion q0

    print("q_delta\n", q_delta)
    print("q_pos\n", q_pos)

    # magnitude of position steps
    dq_total = np.sin(omega_t[omega_nonZero]/(2.*rate))

    # Operates on all rows where omega_nonZero is true
    q_delta[omega_nonZero,:] = omega_05[omega_nonZero,:] * np.tile(dq_total/omega_t[omega_nonZero], (3,1)).T
    print("q_delta new\n", q_delta)

    for ii in range(len(omega_05)-1):
        q1 = unit_q(q_delta[ii,:])
        q2 = q_pos[ii,:]
        if CStype == 'sf':
            qm = q_mult(q1,q2)
        elif CStype == 'bf':
            qm = q_mult(q2,q1)
        else:
            print('I don\'t know this type of coordinate system!')
        q_pos[ii+1,:] = qm # stick the new quaternion into i+1, where i starts at 0 which is where q0 was inserted

    end = time.time()

    print("Duration: ", end - start)

    return q_pos

# NOTE: The reason this takes so long to execute is that this device takes a while to start the program
# Execution time without only the duration print is ~1 ms, which is plenty fast for us

# The output q list has N dimensions, but it includes q0 as the first value
# It really doesn't use the last omega, so might just have to pass some arbitrary array that it ignores
# Then the result we can get the first omega value

# omega, q0, rate, CStype
q_first = calc_quat([[1, 2, 3], [3, 2, 1], [4, 5, 6], [6, 5, 4]], [.5, .5, .5], 400, "bf")
q_next = q_mult(q_first, q_inv(q_first[0]))
print("Output\n", q_first)
print("Output 2\n", q_next)

'''

test = np.array([[1,2,3],[4,5,6]])
print(test[1,:]) # => [4, 5, 6] # Get the array at index 1, get all of it
print(test[1:]) # => [[4, 5, 6]] (from index 1 onwards)
print(test[:1]) # => [[1, 2, 3]] (up to and not including index 1)



print("Output\n", calc_quat([[1, 2, 3], [3, 2, 1]], [.5, .5, .5], 400, "bf"))

 [[0.5        0.5        0.5        0.5       ]
 [0.49765423 0.50077922 0.50015422 0.50140422]
 [0.49389373 0.50201553 0.50139523 0.50264522]]
'''