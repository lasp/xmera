# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy as np
np.set_printoptions(precision=12)


from xmera.utilities import RigidBodyKinematics as rbk




def results(sigma_RR0, omega_RR0_R, RefStateInData, dt, cmdStateFlag, testReset):

    ansSigma = []
    ansOmega_RN_N = []
    ansdOmega_RN_N = []

    sigma_R0N = RefStateInData.sigma_RN
    R0N = rbk.MRP2C(sigma_R0N)
    omega_R0N_N = RefStateInData.omega_RN_N
    domega_R0N_N = RefStateInData.domega_RN_N

    # compute 0th time step
    s0 = np.array(sigma_RR0)
    s1=rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N

    ansSigma.append(s1.tolist())
    ansOmega_RN_N.append(omega_RN_N.tolist())
    ansdOmega_RN_N.append(domega_RN_N.tolist())
    ansSigma.append(s1.tolist())
    ansOmega_RN_N.append(omega_RN_N.tolist())
    ansdOmega_RN_N.append(domega_RN_N.tolist())

    # compute 1st time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    ansSigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    ansOmega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    ansdOmega_RN_N.append(domega_RN_N.tolist())


    # compute 2nd time step
    B =  rbk.BmatMRP(sigma_RR0)
    sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
    RR0 = rbk.MRP2C(sigma_RR0)
    RN = np.dot(RR0, R0N)
    sigma_RN = rbk.C2MRP(RN)
    ansSigma.append(sigma_RN.tolist())

    omega_RR0_N = np.dot(RN.T, omega_RR0_R)
    omega_RN_N = omega_RR0_N + omega_R0N_N
    ansOmega_RN_N.append(omega_RN_N.tolist())

    domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
    domega_RN_N = domega_RR0_N + domega_R0N_N
    ansdOmega_RN_N.append(domega_RN_N.tolist())


    # Testing Reset function
    if testReset:
        if cmdStateFlag:
            sigma_RR0 = s0
        # compute 0th time step
        s1 = rbk.addMRP(np.array(sigma_R0N), np.array(sigma_RR0))
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N

        ansSigma.append(s1.tolist())
        ansOmega_RN_N.append(omega_RN_N.tolist())
        ansdOmega_RN_N.append(domega_RN_N.tolist())

        # compute 1st time step
        B = rbk.BmatMRP(sigma_RR0)
        sigma_RR0 += dt * 0.25 * np.dot(B, omega_RR0_R)
        RR0 = rbk.MRP2C(sigma_RR0)
        RN = np.dot(RR0, R0N)
        sigma_RN = rbk.C2MRP(RN)
        ansSigma.append(sigma_RN.tolist())

        omega_RR0_N = np.dot(RN.T, omega_RR0_R)
        omega_RN_N = omega_RR0_N + omega_R0N_N
        ansOmega_RN_N.append(omega_RN_N.tolist())

        domega_RR0_N = np.cross(omega_R0N_N, omega_RR0_N)
        domega_RN_N = domega_RR0_N + domega_R0N_N
        ansdOmega_RN_N.append(domega_RN_N.tolist())


    return ansSigma, ansOmega_RN_N, ansdOmega_RN_N
