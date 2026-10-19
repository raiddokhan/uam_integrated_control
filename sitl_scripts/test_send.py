import numpy as np
from arm_state_sender import ArmStateSender

q = [1, 2, 3, 4]
qd = [0, 0, 0, 0]
qdd = [0, 0, 0, 0]
tau = [2, 9, 1]

ARM_STATE_CONN = "udpout:127.0.0.1:14580"

arm_sender = ArmStateSender(ARM_STATE_CONN)

RATE=50
STREAM_RATE=250

def calc_feedforward(q) -> float:
	# Calculate Feedforward Torque in Y direction

	theta = [-1.77639886, -1.96584182, -1.05718374, 1.29266102,-0.18348597]

	sin_term = np.sin(q[1:3])
	cos_term = np.cos(q[1:3])
	Phi = np.concatenate([sin_term, cos_term, [1]])

	ff_torque = np.dot(Phi, theta)

	return ff_torque


#while True:
	#if arm_sender is not None:
	#	arm_sender.send(q, qd, qdd, tau)

ff_torq = calc_feedforward([1, 2, 3, 4])

print(ff_torq)
