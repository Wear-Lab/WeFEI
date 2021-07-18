import math
import numpy as np

# rotationAngles: [pitch, yaw, roll]

def bodyToInertialFrame(rotationAngles, point):
	phi = rotationAngles[0] # φ - x
	theta = rotationAngles[1] # θ - y
	psi = rotationAngles[2] # ψ - z

	cosPhi = math.cos(phi)
	sinPhi = math.sin(phi)
	cosTheta = math.cos(theta)
	sinTheta = math.sin(theta)
	cosPsi = math.cos(psi)
	sinPsi = math.sin(psi)

	rotationMatrix = np.array([[cosTheta * cosPsi, cosTheta * sinPsi, -1 * sinTheta],
				[sinPhi * sinTheta * cosPsi - cosPhi * sinPsi, sinPhi * sinTheta * sinPsi + cosPhi * cosPsi, sinPhi * cosTheta],
				[cosPhi * sinTheta * cosPsi + sinPhi * sinPsi, cosPhi * sinTheta * sinPsi - sinPhi * cosPsi, cosPhi * cosTheta]]).transpose()

	vector = np.array([point.x, point.y, point.z]).transpose()

	return np.matmul(rotationMatrix, vector)

def inertialToBodyFrame(rotationAngles, point):
	phi = rotationAngles[0] # φ - x
	theta = rotationAngles[1] # θ - y
	psi = rotationAngles[2] # ψ - z

	cosPhi = math.cos(phi)
	sinPhi = math.sin(phi)
	cosTheta = math.cos(theta)
	sinTheta = math.sin(theta)
	cosPsi = math.cos(psi)
	sinPsi = math.sin(psi)



	rotationMatrix = np.array([[cosTheta * cosPhi, cosTheta * sinPhi, -1 * sinTheta],
				[sinPsi * sinTheta * cosPhi - cosPsi * sinPhi, sinPsi * sinTheta * sinPhi + cosPsi * cosPhi, cosTheta * sinPsi],
				[cosPsi * sinTheta * cosPhi + sinPsi * sinPhi, cosPsi * sinTheta * sinPhi - sinPsi * cosPhi, cosTheta * cosPsi]])



	'''rotationMatrix = np.array([[cosTheta * cosPsi, cosTheta * sinPsi, -1 * sinTheta],
				[sinPhi * sinTheta * cosPsi - cosPhi * sinPsi, sinPhi * sinTheta * sinPsi + cosPhi * cosPsi, sinPhi * cosTheta],
				[cosPhi * sinTheta * cosPsi + sinPhi * sinPsi, cosPhi * sinTheta * sinPsi - sinPhi * cosPsi, cosPhi * cosTheta]])'''

	vector = np.array(point).transpose()

	return np.matmul(rotationMatrix, vector)
