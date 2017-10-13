#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: kinematic_constraint.py
Author: Stanislas Brossette
Email: stanislas.brossette@gmail.com
Github: https://github.com/stanislas-brossette
Description: In this file we try to figure out a linear cstr that represents conservatively a kinematic feasibility nonlinear constraint. We apply that on a toy problem with a kinematic arm that has 2 revolute joints, the entry variable of the problem is the location (x,z) of the hip, and the constraint is on the knee's location (which is a nonlinear function of the hip location computed via IK) that has to remain inside a given area (Square)
"""

import numpy as np
import collections
import matplotlib.pyplot as plt
from scipy.optimize import minimize

l1 = 0.5
l2 = 0.5

def main():
    xHip = np.array([0.7, 0.1])
    print('xHip target:', xHip)
    #thetaZero = np.array([0.1, 0.2])
    #test_finite_diff(thetaZero[0], thetaZero[1])
    #resIK = inverse_kinematics(xHip, thetaZero)
    #print('resIK:', resIK)
    #foundXHip = direct_kinematics(resIK[0], resIK[1])
    #print('xHip result:', foundXHip)
    #plot(resIK,xHip)
    #plot()
    kneeFromHip(xHip)

#This function returns the nonlinear constraint that we want to approximate
def kin_cstr(xHip, xKneeMin=np.array([0.3,0.1]), xKneeMax=np.array([0.5,0.4])):
    knee = kneeFromHip(xHip)
    res = np.zeros(4)
    res[0] = xKneeMax[0]-knee[0]
    res[1] = xKneeMax[1]-knee[1]
    res[2] = -xKneeMin[0]+knee[0]
    res[3] = -xKneeMin[1]+knee[1]
    return res

#This function is used to check the derivatives by finite difference
def test_finite_diff(theta1, theta2):
    delta = 1e-8
    xHip = np.array([0.5,0.1])

    print('\ntest Direct Kinematics')
    dDKdthetaFD = np.zeros((2,2))
    dDKdthetaFD[:,0] = (direct_kinematics(theta1+delta,theta2).hip-direct_kinematics(theta1,theta2).hip)/delta
    dDKdthetaFD[:,1] = (direct_kinematics(theta1,theta2+delta).hip-direct_kinematics(theta1,theta2).hip)/delta
    print('dDKdthetaFD', dDKdthetaFD)
    jac = jacobian(theta1, theta2).hip
    print('dDKdthetaCalc', jac)

    print('\nTest Cost Function') 
    theta = np.array([theta1, theta2])
    thetaP1 = np.array([theta1+delta, theta2])
    thetaP2 = np.array([theta1, theta2+delta])
    dfdthetaFD = np.zeros(2)
    dfdthetaFD[0] = (cost_func(thetaP1,xHip)-cost_func(theta,xHip))/delta
    dfdthetaFD[1] = (cost_func(thetaP2,xHip)-cost_func(theta,xHip))/delta
    print('dfdthetaFD', dfdthetaFD)
    dfdthetaCalc = cost_func_deriv(theta, xHip)
    print('dfdthetaCalc', dfdthetaCalc, '\n')

#Computes the direct kinematics from theta (returns knee and hip positions)
def direct_kinematics(theta1, theta2):
    x_knee = np.array([l1*np.cos(theta1), l1*np.sin(theta1)])
    x_hip = x_knee + np.array([l2*np.cos(theta1 + theta2), l2*np.sin(theta1 + theta2)])
    robotKinematics = collections.namedtuple('positions', ['knee', 'hip'])
    res = robotKinematics(x_knee, x_hip)
    return res

#Computes the jacobians of the knee and hip positions
def jacobian(theta1, theta2):
    jac_hip = np.matrix([[-l1*np.sin(theta1) - l2*np.sin(theta1 + theta2), -l2*np.sin(theta1 + theta2)],
                         [+l1*np.cos(theta1) + l2*np.cos(theta1 + theta2), +l2*np.cos(theta1 + theta2) ]])
    jac_knee = np.matrix([[-l1*np.sin(theta1) , 0],
                          [+l1*np.cos(theta1), 0]])
    robotJacobians = collections.namedtuple('jacobians', ['knee', 'hip'])
    jac = robotJacobians(jac_knee, jac_hip)
    return jac

#Cost function measuring the squared distance to the target hip position
def cost_func(theta, xHip):
    kin = direct_kinematics(theta[0], theta[1])
    res = (kin.hip[0]-xHip[0])**2 + (kin.hip[1]-xHip[1])**2
    return res

#Derivatives of the cost function
def cost_func_deriv(theta, xHip):
    kin = direct_kinematics(theta[0], theta[1])
    jac = jacobian(theta[0], theta[1])
    dfdtheta0 = 2*(kin.hip[0]-xHip[0])*jac.hip[0,0] + 2*(kin.hip[1]-xHip[1])*jac.hip[1,0]
    dfdtheta1 = 2*(kin.hip[0]-xHip[0])*jac.hip[0,1] + 2*(kin.hip[1]-xHip[1])*jac.hip[1,1]
    return np.array([dfdtheta0, dfdtheta1])

    
#Computes thetas to reach the xHip goal with a simple minimization. Bounds cstr could be added
def inverse_kinematics(xHip, thetaInit=np.array([0.13,-0.15])):
    angles = collections.namedtuple('angles', ['theta1', 'theta2'])
    bnds = ((-1.0, 1.0),(-2.0, 0.0))
    res = minimize(cost_func, thetaInit, args=(xHip), jac=cost_func_deriv, method='SLSQP', options={'disp':True})
    #res = minimize(cost_func, thetaInit, args=(xHip), jac=cost_func_deriv, bounds=bnds, method='SLSQP', options={'disp':True})
    return res.x

#Funny plot of trajectories of the robot
def plot():
    nPoints = 1000
    vTheta1 = np.linspace(0,2*np.pi,nPoints)
    vTheta2 = np.linspace(0,10*np.pi,nPoints)
    vx_knee = np.empty(nPoints)
    vy_knee = np.empty(nPoints)
    vx_hip = np.empty(nPoints)
    vy_hip = np.empty(nPoints)
    for i in np.arange(0,nPoints):
      kin = direct_kinematics(vTheta1[i], vTheta2[i])
      vx_knee[i] = kin.knee[0]
      vy_knee[i] = kin.knee[1]
      vx_hip[i] = kin.hip[0]
      vy_hip[i] = kin.hip[1]
    plt.plot(vx_knee, vy_knee)
    plt.plot(vx_hip, vy_hip)
    plt.show()

#Plots the robot and target
def plot(theta, xHipGoal):
    kin = direct_kinematics(theta[0], theta[1])
    x = np.array([0.0, kin.knee[0], kin.hip[0]])
    y = np.array([0.0, kin.knee[1], kin.hip[1]])
    plt.plot(x, y)
    plt.plot(xHipGoal[0], xHipGoal[1], 'rx')
    plt.show()

#Returns the location of the knee for a given hip position
def kneeFromHip(xHip):
    theta = inverse_kinematics(xHip)
    kin = direct_kinematics(theta[0], theta[1])
    print('Knee:', kin.knee)
    plot(theta,xHip)
    return kin.knee

main()
