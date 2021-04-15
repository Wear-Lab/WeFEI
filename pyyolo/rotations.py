import math
import numpy as np

def bodyToInertialFrame(rotationAngles, point):
	theta = rotationAngles[0] # θ - pitch
	phi = rotationAngles[1] # φ - yaw
	psi = rotationAngles[2] # ψ - roll

	cosTheta = math.cos(theta)
	sinTheta = math.sin(theta)
	cosPhi = math.cos(phi)
	sinPhi = math.sin(phi)
	cosPsi = math.cos(psi)
	sinPsi = math.sin(psi)

	rotationMatrix = np.array([[cosTheta * cosPhi, cosTheta * sinPhi, -1 * sinTheta],
					[-1 * cosPsi * sinPhi + sinPsi * sinTheta * cosPhi, cosPsi * cosPhi + sinPsi * sinTheta * sinPhi, sinPsi * cosTheta],
					[sinPsi * sinPhi + cosPsi * sinTheta * cosPhi, -1 * sinPsi * cosPhi + cosPsi * sinTheta * sinPhi, cosPsi * cosTheta]]).transpose()

	vector = np.asarray([point.x, point.y, point.z])

	return rotationMatrix.dot(vector)

def inertialToBodyFrame(rotationAngles, point):
	theta = rotationAngles[0] # θ - pitch
	phi = rotationAngles[1] # φ - yaw
	psi = rotationAngles[2] # ψ - roll
	
	cosTheta = math.cos(theta)
	sinTheta = math.sin(theta)
	cosPhi = math.cos(phi)
	sinPhi = math.sin(phi)
	cosPsi = math.cos(psi)
	sinPsi = math.sin(psi)

	rotationMatrix = rotationMatrix = np.array([[cosTheta * cosPhi, cosTheta * sinPhi, -1 * sinTheta],
					[-1 * cosPsi * sinPhi + sinPsi * sinTheta * cosPhi, cosPsi * cosPhi + sinPsi * sinTheta * sinPhi, sinPsi * cosTheta],
					[sinPsi * sinPhi + cosPsi * sinTheta * cosPhi, -1 * sinPsi * cosPhi + cosPsi * sinTheta * sinPhi, cosPsi * cosTheta]])

	vector = np.asarray([0, 9.81, 0])

	return rotationMatrix.dot(vector)
