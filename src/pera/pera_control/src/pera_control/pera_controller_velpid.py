#!/usr/bin/env python

"""
   Controllers for pera
"""
from __future__ import division
import rospy

import numpy as np
import math

from std_msgs.msg import Float64
from diagnostic_msgs.msg import *

from urdf_parser_py.urdf import URDF
from boards import *
from myexception import *

from rtm_usb import *

from initencoders import *

class parallel():
    """
        test PID control
    """
    def __init__(self, K, D, M, KF, Q, F):
        self.Km = np.identity(3)
        self.Km[0][0] = K[0][0]
        self.Km[1][1] = K[1][0]
        self.Km[2][2] = K[2][0]
        self.Dm = np.identity(3)
        self.Dm[0][0] = D[0][0]
        self.Dm[1][1] = D[1][0]
        self.Dm[2][2] = D[2][0]
        self.Md = np.identity(3)
        self.Md[0][0] = M[0][0]
        self.Md[1][1] = M[1][0]
        self.Md[2][2] = M[2][0]
        self.Kf = np.identity(3)
        self.Kf[0][0] = KF[0][0]
        self.Kf[1][1] = KF[1][0]
        self.Kf[2][2] = KF[2][0]
        self.qd = Q
        self.al2 = 0.22
        self.qbard = np.array([[self.al2*np.cos(self.qd[0])*np.cos(self.qd[1])],
            [self.al2*np.cos(self.qd[1])*np.sin(self.qd[0])],
            [-self.al2*np.sin(self.qd[1])]])
        self.Fd = F 

        self.F_prev = np.zeros((3,1))
        self.zfPH = np.zeros((3,1))
        self.zfEL = np.zeros((3,1))

        # For moving average filter
        self.window_size = 10
        self.values = []
        self.sum = 0

    def parallelControlPH(self,posWP,velWP,posWY,velWY,Fe,delta):
        # System parameters
        I1 = np.zeros((3,3))
        I2 = np.diag([3.467e-4, 3.467e-4, 3.467e-4])
        m1, m2 = 0, 0.2+0.035+0.035
        al2 = self.al2
        acl2 = 0.5*al2
        g = 9.80665

        # Current wrist position
        q = np.array([[posWP],[posWY]])
        # Current wrist velocity
        qdot = np.array([[velWP],[velWY]])

        # Mass matrix
        M = np.array([[I1[2][2] + I2[1][1] + m2*np.cos(posWP)**2*np.cos(posWY)**2*(acl2 - al2)**2 + m2*np.cos(posWY)**2*np.sin(posWP)**2*(acl2-al2)**2, -I2[1][2]],
            [-I2[2][1], m2*acl2**2 - 2*m2*acl2*al2 + m2*al2**2 + I2[2][2]]])
        Minv = np.linalg.inv(M)
        # Current wrist momentum
        p = np.matmul(M, qdot)
        # Mass matrix derivative
        a111 = 2*np.cos(posWP)*(-np.sin(posWP))*velWP*np.cos(posWY)**2 + np.cos(posWP)**2*2*np.cos(posWY)*(-np.sin(posWY))*velWY
        a112 = 2*np.cos(posWY)*(-np.sin(posWY))*velWY*np.sin(posWP)**2 + np.cos(posWY)**2*2*np.sin(posWP)*np.cos(posWP)*velWP
        Mdot = np.array([[m2*(acl2 - al2)**2*a111 + m2*(acl2 - al2)**2*a112, 0],[0, 0]])
        # Gravity
        g1 = -g*m2*np.cos(posWY)*np.sin(posWP)*(acl2 - al2)
        g2 = -g*m2*np.cos(posWP)*np.sin(posWY)*(acl2 - al2)
        G = np.array([[g1], [g2]])
        # Centrifugal and Coriolis
        c1 =  -velWP*velWY*m2*np.sin(2*posWY)*(acl2 - al2)**2
        c2 = (velWP**2*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2
        C = np.array([[c1],[c2]])
        # Linear Jacobian
        JL = np.array([[-al2*np.cos(posWY)*np.sin(posWP), -al2*np.cos(posWP)*np.sin(posWY)],
            [al2*np.cos(posWP)*np.cos(posWY), -al2*np.sin(posWP)*np.sin(posWY)],
            [0,             -al2*np.cos(posWY)]])
        pJL = np.linalg.pinv(JL)
        # Transformed variables
        qbar = np.array([[al2*np.cos(posWP)*np.cos(posWY)],
            [al2*np.cos(posWY)*np.sin(posWP)],
            [-al2*np.sin(posWY)]])
        qbardot = np.matmul(JL, qdot)
        pbar = np.matmul(pJL.T, p)
        Mbar = pJL.T.dot(M).dot(JL)
        Mbarinv = JL.dot(Minv).dot(JL.T)

        # Contact force time derivative
        Fedot = (self.F_prev - Fe)/delta.to_sec()
        self.F_prev = Fe

        # Force error integrator
        self.zfPH = self.zfPH + Mbar.dot(self.Kf).dot(self.Fd - Fe)

        phat = pbar + 2*self.Km.dot(qbar - self.qbard + self.zfPH) + self.Kf.dot(self.Fd - Fe)

        # Control Law
        # 1. Feedback linearization
        ul = C - Mdot.dot(qdot) + G 
        # 2. Impedance control law
        ui = self.Md.dot(Mbarinv).dot(self.Km).dot(qbar - self.qbard) - (self.Md.dot(Mbarinv).dot(self.Kf) - np.identity(3)).dot(Fe)
        # 3. Force integral control
        uf = -self.Md.dot(Mbarinv).dot(self.Kf).dot(self.Fd) - self.Md.dot(Mbarinv).dot(self.Km).dot(self.zfPH) - 2*self.Km.dot(qbardot + Mbar.dot(self.Kf).dot(self.Fd - Fe)) + self.Kf.dot(Fedot) - self.Dm.dot(np.linalg.inv(self.Md)).dot(phat)
        # 4. Combine
        u = ul - JL.T.dot(ui) + JL.T.dot(uf)

        tau_WP = float(u[0])
        tau_WY = float(u[1])

        return tau_WP, tau_WY

    def parallelControlEL(self,posWP,velWP,posWY,velWY,Fe,KI,delta):
        # System parameters
        I1 = np.zeros((3,3))
        I2 = np.diag([3.467e-4, 3.467e-4, 3.467e-4])
        m1, m2 = 0, 0.2+0.035+0.035
        al2 = self.al2
        acl2 = 0.5*al2
        g = 9.80665

        # Current wrist position
        q = np.array([[posWP],[posWY]])
        # Current wrist velocity
        qdot = np.array([[velWP],[velWY]])

        # Mass matrix
        M = np.array([[I1[2][2] + I2[1][1] + m2*np.cos(posWP)**2*np.cos(posWY)**2*(acl2 - al2)**2 + m2*np.cos(posWY)**2*np.sin(posWP)**2*(acl2-al2)**2, -I2[1][2]],
            [-I2[2][1], m2*acl2**2 - 2*m2*acl2*al2 + m2*al2**2 + I2[2][2]]])
        # Gravity
        g1 = -g*m2*np.cos(posWY)*np.sin(posWP)*(acl2 - al2)
        g2 = -g*m2*np.cos(posWP)*np.sin(posWY)*(acl2 - al2)
        G = np.array([[g1], [g2]])
        # Centrifugal and Coriolis
        c1 =  -velWP*velWY*m2*np.sin(2*posWY)*(acl2 - al2)**2
        c2 = (velWP**2*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2
        C = np.array([[c1],[c2]])
        CC = np.array([[-(velWY*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2, -(velWP*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2],
        [(velWP*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2, 0]])
        # Linear Jacobian
        JL = np.array([[-al2*np.cos(posWY)*np.sin(posWP), -al2*np.cos(posWP)*np.sin(posWY)],
            [al2*np.cos(posWP)*np.cos(posWY), -al2*np.sin(posWP)*np.sin(posWY)],
            [0,             -al2*np.cos(posWY)]])
        pJL = np.linalg.pinv(JL)
        JLdot = np.array([[-al2*(-np.sin(posWY)*velWY*np.sin(posWP) + np.cos(posWY)*np.cos(posWP)*velWP), -al2*(-np.sin(posWP)*velWP*np.sin(posWY) + np.cos(posWP)*np.cos(posWY)*velWY)],
            [al2*(-np.sin(posWP)*velWP*np.cos(velWY) + np.cos(posWP)*(-np.sin(posWY)*velWY)), -al2*(np.cos(posWP)*velWP*np.sin(posWY) + np.sin(posWP)*np.cos(posWY)*velWY)],
            [0, al2*np.sin(posWY)*velWY]]])

        # Transformed variables
        qbar = np.array([[al2*np.cos(posWP)*np.cos(posWY)],
            [al2*np.cos(posWY)*np.sin(posWP)],
            [-al2*np.sin(posWY)]])
        qbardot = np.matmul(JL, qdot)

        # Force error integrator
        self.zfEL = self.zfEL + self.Fd - Fe

        # Control Law
        uParallel = np.linalg.inv(self.Md).dot((self.Dm.dot(-qbardot) + self.Km.dot(self.qbard - qbar)) - self.K_F.dot(self.Fd - Fe) - KI.dot(self.zfEL))
        u = M.dot(pJL).dot(uParallel - JLdot.dot(qdot)) + CC.dot(qdot) + G - JL.T.dot(Fe)

        tau_WP = float(u[0])
        tau_WY = float(u[1])

        return tau_WP, tau_WY

    def ImpedancePH(self,posWP,velWP,posWY,velWY,Fe):
        # System parameters
        I1 = np.zeros((3,3))
        I2 = np.diag([3.467e-4, 3.467e-4, 3.467e-4])
        m1, m2 = 0, 0.2+0.035+0.035
        al2 = self.al2
        acl2 = 0.5*al2
        g = 9.80665

        # Current wrist position
        q = np.array([[posWP],[posWY]])
        # Current wrist velocity
        qdot = np.array([[velWP],[velWY]])

        # Mass matrix
        M = np.array([[I1[2][2] + I2[1][1] + m2*np.cos(posWP)**2*np.cos(posWY)**2*(acl2 - al2)**2 + m2*np.cos(posWY)**2*np.sin(posWP)**2*(acl2-al2)**2, -I2[1][2]],
            [-I2[2][1], m2*acl2**2 - 2*m2*acl2*al2 + m2*al2**2 + I2[2][2]]])
        Minv = np.linalg.inv(M)
        # Current wrist momentum
        p = np.matmul(M, qdot)
        # Mass matrix derivative
        a111 = 2*np.cos(posWP)*(-np.sin(posWP))*velWP*np.cos(posWY)**2 + np.cos(posWP)**2*2*np.cos(posWY)*(-np.sin(posWY))*velWY
        a112 = 2*np.cos(posWY)*(-np.sin(posWY))*velWY*np.sin(posWP)**2 + np.cos(posWY)**2*2*np.sin(posWP)*np.cos(posWP)*velWP
        Mdot = np.array([[m2*(acl2 - al2)**2*a111 + m2*(acl2 - al2)**2*a112, 0],[0, 0]])
        # Gravity
        g1 = -g*m2*np.cos(posWY)*np.sin(posWP)*(acl2 - al2)
        g2 = -g*m2*np.cos(posWP)*np.sin(posWY)*(acl2 - al2)
        G = np.array([[g1], [g2]])
        # Centrifugal and Coriolis
        c1 =  -velWP*velWY*m2*np.sin(2*posWY)*(acl2 - al2)**2
        c2 = (velWP**2*m2*np.sin(2*posWY)*(acl2 - al2)**2)/2
        C = np.array([[c1],[c2]])
        # Linear Jacobian
        JL = np.array([[-al2*np.cos(posWY)*np.sin(posWP), -al2*np.cos(posWP)*np.sin(posWY)],
            [al2*np.cos(posWP)*np.cos(posWY), -al2*np.sin(posWP)*np.sin(posWY)],
            [0,             -al2*np.cos(posWY)]])
        pJL = np.linalg.pinv(JL)
        # Transformed variables
        qbar = np.array([[al2*np.cos(posWP)*np.cos(posWY)],
            [al2*np.cos(posWY)*np.sin(posWP)],
            [-al2*np.sin(posWY)]])
        qbardot = np.matmul(JL, qdot)
        pbar = np.matmul(pJL.T, p)
        Mbar = pJL.T.dot(M).dot(JL)
        Mbarinv = JL.dot(Minv).dot(JL.T)

        # Control Law
        # 1. Feedback linearization
        ul = C - Mdot.dot(qdot) + G 
        # 2. Impedance control law
        ui = self.Md.dot(Mbarinv).dot(self.Km).dot(qbar - self.qbard) - (self.Md.dot(Mbarinv) - np.identity(3)).dot(Fe) + self.Dm.dot(np.linalg.inv(self.Md)).dot(pbar)
        # 3. Combine
        u = ul - JL.T.dot(ui)

        tau_WP = float(u[0])
        tau_WY = float(u[1])

        return tau_WP, tau_WY

    def moving_average(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values)>self.window_size:
            self.sum -= self.values.pop(0)
        return float(self.sum)/len(self.values)

