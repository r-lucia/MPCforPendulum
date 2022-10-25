#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import cvxpy

# parametri fisici del sistema
l = 1.5  # length of bar
M = 1 # [kg]
m = 0.1  # [kg]
g = 9.8  # [m/s^2]
Ia = 0.019
k2 = 1 / (M + m)
k1 = (-m * g * l) / Ia



# State indexes
X = 0
THETA = 1
DX = 2
DTHETA = 3
XMAX = 4  # number of states
UMAX = 1  # number of inputs

Q = np.diag([10, 7, 0.0, 0.0])
R = np.diag([0.0000])
T = 30  # Horizon length
delta_t = 0.1 # time tick
max_F = 70000.0  # Max and minimum forces
min_F = -70000.0

# matrici di stato:
A = np.array([
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, k1, 0.0, 0.0]
])
B = np.array([[0.0], [0.0], [k2], [0.0]])


def callback(msg):
    global teta_pendolo
    global pos_carrellino
    global vel_carrellino
    global vel_pendolo
    global x0
    teta_pendolo = msg.position[1]
    pos_carrellino = msg.position[0]
    vel_carrellino = msg.velocity[0]
    vel_pendolo = msg.velocity[1]
    x0 = [pos_carrellino, teta_pendolo, vel_carrellino, vel_pendolo]
    # rospy.loginfo("angolo del pendolo %f", teta_pendolo)
    # rospy.loginfo("posizione del carrellino %f", pos_carrellino)
    # rospy.loginfo("velocità del carrellino %f", vel_carrellino)
    # rospy.loginfo("velocità del pendolo %f", vel_pendolo)


def talker():
    x_r, Tmax = build_trajectory()
    pub = rospy.Publisher('pend_carr_controller/command', Float64, queue_size=10)
    rospy.Subscriber("joint_states", JointState,
                     callback)  # questa funzione viene chiamata ogni volta che arriva un nuovo messaggio sul topic
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        u, x = mpc_control(x0, x_r)
        f_desiderata = u[0, 0]
        rospy.logwarn("forza desiderata %f", f_desiderata)
        pub.publish(f_desiderata)
        # rospy.logwarn("teta ottenuto %f",teta_pendolo)
        rate.sleep()


def mpc_control(x0, x_r):
    # Perform MPC (linearized about equilbrium)
    x = cvxpy.Variable((XMAX, T + 1))
    u = cvxpy.Variable((UMAX, T))

    cost = 0.0
    constr = []
    for t in range(T):
        cost += cvxpy.quad_form(x[:, t + 1] - x_r[:, t + 1], Q)
        cost += cvxpy.quad_form(u[:, t], R)
        constr.append(x[:, t + 1] == x[:, t] + delta_t * (A @ x[:, t] + B @ u[:, t]))  # tiene memoria degli stati
        constr += [u[:, t] <= max_F]
        constr += [u[:, t] >= min_F]
    constr += [x[:, 0] == x0]
    start = time.time()
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constr)
    prob.solve(verbose=True)
    end = time.time()
    print("solution took", end - start, "seconds")

    # Extract the predicted trajectory and inputs
    x = np.array(x.value)
    u = np.array(u.value)

    return u, x


def build_trajectory():
    # Create a trajectory to follow - specify the velocity and the time to follow that
    # velocity
    # m/s,  time

    v_desiderata = 0
    p_desiderata = [-8, 0, 2, -4]
    Tmax = 6
    x_r = []
    for i in range(9):
        x_r.append([-8, 0, v_desiderata, 0])
    for i in range(5):
        x_r.append([0, 0, v_desiderata, 0])
    for i in range(9):
        x_r.append([8, 0, v_desiderata, 0])
    for i in range(8):
        x_r.append([-4, 0, v_desiderata, 0])

    x_r = np.array(x_r).T

    return x_r, Tmax


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
