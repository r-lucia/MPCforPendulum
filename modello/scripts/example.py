#!/usr/bin/env python3
import gekko
from gekko import GEKKO
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import math
import time


# Parametrers: m.Const(value)
# questo ci permette di definere variabili statiche che non vengono modificate dall'ottimizzatore

def callback(msg):
    global teta_pendolo
    global pos_carrellino
    global vel_carrellino
    global vel_pendolo
    global x0

    pos_carrellino = msg.position[0]
    vel_carrellino = msg.velocity[0]

    x0 = [pos_carrellino, vel_carrellino]


def talker():
    pub = rospy.Publisher('pend_carr_controller/command', Float64, queue_size=1)
    rospy.Subscriber("joint_states", JointState,
                     callback,
                     queue_size=1)  # questa funzione viene chiamata ogni volta che arriva un nuovo messaggio sul topic
    rospy.init_node('talker', anonymous=True)
    # 10hz

    m = GEKKO(remote=False)
    m.time = np.linspace(0, 1, 21)

    l = m.Param(0.25)  # lunchezza di metà asta

    M = m.Param(1)  # [kg]
    mp = m.Param(0.1)  # [kg]
    g = m.Param(9.8)  # [m/s^2]


    m.options.CV_TYPE = 2

    # Manipulated variable
    u = m.MV(name='u')
    u.STATUS = 1  # questo mi dice può esser modificato dall'ottimizzatore
    u.DCOST = 0.1 # quanto influeisce sulla funzione obbiettivo la variazione di u
    u.FSTATUS = 0

    # Controlled Variable
    s = m.CV(value=pos_carrellino, name='s')
    s.STATUS = 1
    s.FSTATUS = 1
    s.TAU = 0.1
    s.SP = -8
    # s.SPHI= s_des + 0.1
    # s.SPLO= s_des - 0.1
    s.TR_INIT = 2  # !! devo inserire il valore dato dal nodo
    s.WSP = 10


    ds = m.CV(value=vel_carrellino, name='ds')
    ds.STATUS = 1
    ds.SP = 0
    ds.FSTATUS = 1
    ds.TAU = 0.1
    ds.TR_INIT = 2


    m.Equation(s.dt() == ds)
    m.Equation(ds.dt() == (1/M) * u)
    # m.Equation(ds.SP == 0)
    m.options.IMODE = 6  # control
    m.options.SOLVER = 3
    # while not rospy.is_shutdown():
    s.MEAS = pos_carrellino

    ds.MEAS = vel_carrellino
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        s.MEAS = pos_carrellino
        ds.MEAS = vel_carrellino
        try:
            m.solve(disp=True)
            f_desiderata = u[1]
        except:
            f_desiderata = 0

        pub.publish(f_desiderata)

        print(s.PRED)
        print(ds.PRED)
        print(u.PRED)
        rospy.logwarn("forza desiderata %f", f_desiderata)
        rate.sleep()






if __name__ == '__main__':
    talker()
