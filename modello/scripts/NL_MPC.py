#!/usr/bin/env python3

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
    teta_pendolo = msg.position[1]
    pos_carrellino = msg.position[0]
    vel_carrellino = msg.velocity[0]
    vel_pendolo = msg.velocity[1]
    x0 = [pos_carrellino, teta_pendolo, vel_carrellino, vel_pendolo]

def talker():
    pub = rospy.Publisher('pend_carr_controller/command', Float64, queue_size=10)
    rospy.Subscriber("joint_states", JointState,
                     callback)  # questa funzione viene chiamata ogni volta che arriva un nuovo messaggio sul topic
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        u = NL_mpc()
        f_desiderata = u[1]
        rospy.logwarn("forza desiderata %f", f_desiderata)
        pub.publish(f_desiderata)
        # rospy.logwarn("teta ottenuto %f",teta_pendolo)
        rate.sleep()
        #rospy.logwarn("posizione attuale %f", pos_carrellino)



def NL_mpc():

    m = GEKKO(remote=False)
    m.time = np.linspace(0, 2, 20)

    l = m.Const(0.75)  # lunchezza di metà asta

    M = m.Const(1)  # [kg]
    mp = m.Const(0.1)  # [kg]
    g = m.Const(9.8)  # [m/s^2]
    k2 = (mp * g) / 2
    k1 = mp * l
    k4 = M * g
    s_des = m.Param(value=8)
    teta_des = m.Param(value=0)

    # Manipulated variable
    u = m.MV(name='u')
    u.STATUS = 1  # questo mi dice può esser modificato dall'ottimizzatore
    u.DCOST = 10
    # u.DMAX = 200
    # u.FSTATUS = 0

    # Controlled Variable
    s = m.CV(value=pos_carrellino, name='s')
    s.STATUS = 1
    s.TAU = 1.5
    s.SP = s_des
    # s.TR_INIT = 2        # !! devo inserire il valore dato dal nodo

    teta = m.CV(value=teta_pendolo, name='teta')
    teta.STATUS = 1
    teta.SP = teta_des
    # teta.TR_INIT = 2

    ds = m.CV(value=vel_carrellino, name='ds')
    ds.STATUS = 1
    ds.SP = 0
    # ds.TR_INIT = 0

    dteta = m.CV(value=vel_pendolo, name='dteta')
    dteta.STATUS = 1
    dteta.SP = 0
    # dteta.TR_INIT = 0




    m.Minimize(50 * (s - s_des) ** 2)
    m.Minimize(100 * (teta - teta_des) ** 2)


    sin_teta = m.sin(teta)
    sin_2teta = m.sin(teta+teta)
    cos_teta = m.cos(teta)
    k3= M + mp * (1 - cos_teta**2)

    m.Equation(s.dt() == ds)
    m.Equation(teta.dt() == dteta)
    m.Equation(ds.dt() == ((-k2 * sin_2teta) - (k1 * sin_teta * dteta.dt()**2) + u) / k3)
    m.Equation(dteta.dt() == ((k4 * sin_teta) + (k2 * sin_teta) + (0.5 * k1 * sin_2teta * dteta.dt()**2) + (cos_teta * u)) / (l * k3))
    m.options.IMODE = 6 # control
   # m.open_folder()
    m.solve(disp=True)
    print(s.value)
    print(teta.value)
    print(u.value)

    #x = np.array(x.value)
    #u = np.array(u.value)


    return u

    #with open(m.path + '/results.json') as f:
     #   results = json.load(f)


if __name__ == '__main__':
    talker()
