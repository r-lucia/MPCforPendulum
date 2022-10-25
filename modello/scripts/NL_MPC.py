#!/usr/bin/env python3

from gekko import GEKKO
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import json

m = GEKKO()
m.time = np.linspace(0, 1, 501)
# Parametrers: m.Const(value)
# questo ci permette di definere variabili statiche che non vengono modificate dall'ottimizzatore

l = m.Const(0.75)  # lunchezza di metà asta
l1= 1/l
M = m.Const(1)  # [kg]
mp = m.Const(0.1)  # [kg]
g = m.Const(9.8)  # [m/s^2]
Ia = m.Const(0.019)

k2 = 1/(M+mp)
k1 = m.Const((-mp * g * l) / Ia)


def NL_mpc():
    s_des = m.Param(value=15)
    teta_des = m.Param(value=0)

    # Manipulated variable
    u = m.MV(value=0, lb=-10000, ub=10000)
    u.STATUS = 1  # questo mi dice può esser modificato dall'ottimizzatore
   # u.DCOST = 100000000
    #u.DMAX = 200
    #u.FSTATUS = 0

    # Controlled Variable
    s = m.CV(value=0)
    s.STATUS = 1
    s.TAU = 1.5
    s.SP = s_des
    s.TR_INIT = 2 # !! devo inserire il valore dato dal nodo

    m.Minimize(10 * (s - s_des) ** 2)

    teta = m.CV(0)
    teta.STATUS = 1
    teta.SP = teta_des
    teta.TR_INIT = 2

    s.TAU = 1.5
    m.Minimize(7 * (teta - teta_des) ** 2)

    ds = m.CV(0)
    ds.STATUS = 1
    ds.SP = 0
    ds.TR_INIT = 0

    dteta = m.CV(0)
    dteta.STATUS = 1
    dteta.SP = 0
    dteta.TR_INIT = 0

    sin_teta = m.sin(teta)
    cos_teta = m.cos(teta)
    m.Equation(s.dt() == ds)
    m.Equation(teta.dt() == dteta)
    m.Equation(ds.dt() == k2 * u)
    m.Equation(dteta.dt() == k1 * sin_teta)
    m.options.IMODE = 6# control
    m.solve(disp=True)
    print(s.value)
    print(teta.value)
    print(u.value)


    #with open(m.path + '/results.json') as f:
     #   results = json.load(f)


if __name__ == '__main__':
    NL_mpc()
