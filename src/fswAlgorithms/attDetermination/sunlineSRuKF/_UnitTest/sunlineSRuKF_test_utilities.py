import inspect
import os
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('fswAlgorithms')

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Ellipse

color_x = 'dodgerblue'
color_y = 'salmon'
color_z = 'lightgreen'
m2km = 1.0 / 1000.0


def states(x, testName):
    numStates = len(x[0, :]) - 1

    t = np.zeros(len(x[:, 0]))
    for i in range(len(t)):
        t[i] = x[i, 0] * 1E-9

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(331)
    plt.plot(t, x[:, 1], "b", label='Error Filter')
    plt.legend(loc='lower right')
    plt.title('First sHat component')
    plt.grid()

    plt.subplot(332)
    plt.plot(t, x[:, 4], "b")
    plt.title('First rate component (rad/s)')
    plt.grid()

    plt.subplot(333)
    plt.plot(t, x[:, 2], "b")
    plt.title('Second sHat component')
    plt.grid()

    plt.subplot(334)
    plt.plot(t, x[:, 5], "b")
    plt.xlabel('t(s)')
    plt.title('Second rate component (rad/s)')
    plt.grid()

    plt.subplot(335)
    plt.plot(t, x[:, 3], "b")
    plt.xlabel('t(s)')
    plt.title('Third sHat component')
    plt.grid()

    plt.subplot(336)
    plt.plot(t, x[:, 6], "b")
    plt.xlabel('t(s)')
    plt.title('Third rate component (rad/s)')
    plt.grid()

    plt.subplot(337)
    plt.plot(t, x[:, 7], "b")
    plt.xlabel('t(s)')
    plt.title('Bias state (-)')
    plt.grid()

    return plt


def energy(t, energy, testName):
    conserved = np.zeros(len(t))
    for i in range(len(t)):
        conserved[i] = (energy[i] - energy[0]) / energy[0]

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(t, conserved, "b", label='Energy')
    plt.legend(loc='lower right')
    plt.title('Energy ' + testName)
    plt.grid()

    return plt


def state_covar(x, Pflat, testName):
    numStates = len(x[0, :]) - 1

    P = np.zeros([len(Pflat[:, 0]), numStates, numStates])
    t = np.zeros(len(Pflat[:, 0]))
    for i in range(len(Pflat[:, 0])):
        t[i] = x[i, 0] * 1E-9
        P[i, :, :] = Pflat[i, 1:(numStates * numStates + 1)].reshape([numStates, numStates])

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(331)
    plt.plot(t, x[:, 1], "b", label='Error Filter')
    plt.plot(t, x[:, 1] + 3 * np.sqrt(P[:, 0, 0]), 'r--', label='Covar Filter')
    plt.plot(t, x[:, 1] - 3 * np.sqrt(P[:, 0, 0]), 'r--')
    plt.legend(loc='lower right')
    plt.title('First pos component (m)')
    plt.grid()

    plt.subplot(332)
    plt.plot(t, x[:, 4], "b")
    plt.plot(t, x[:, 4] + 3 * np.sqrt(P[:, 3, 3]), 'r--')
    plt.plot(t, x[:, 4] - 3 * np.sqrt(P[:, 3, 3]), 'r--')
    plt.title('Second rate component (m/s)')
    plt.grid()

    plt.subplot(333)
    plt.plot(t, x[:, 2], "b")
    plt.plot(t, x[:, 2] + 3 * np.sqrt(P[:, 1, 1]), 'r--')
    plt.plot(t, x[:, 2] - 3 * np.sqrt(P[:, 1, 1]), 'r--')
    plt.title('Second pos component (m)')
    plt.grid()

    plt.subplot(334)
    plt.plot(t, x[:, 5], "b")
    plt.plot(t, x[:, 5] + 3 * np.sqrt(P[:, 4, 4]), 'r--')
    plt.plot(t, x[:, 5] - 3 * np.sqrt(P[:, 4, 4]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Third rate component (m/s)')
    plt.grid()

    plt.subplot(335)
    plt.plot(t, x[:, 3], "b")
    plt.plot(t, x[:, 3] + 3 * np.sqrt(P[:, 2, 2]), 'r--')
    plt.plot(t, x[:, 3] - 3 * np.sqrt(P[:, 2, 2]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Third pos component (m)')
    plt.grid()

    plt.subplot(336)
    plt.plot(t, x[:, 6], "b")
    plt.plot(t, x[:, 6] + 3 * np.sqrt(P[:, 5, 5]), 'r--')
    plt.plot(t, x[:, 6] - 3 * np.sqrt(P[:, 5, 5]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Third rate component (m/s)')
    plt.grid()

    plt.subplot(337)
    plt.plot(t, x[:, 7], "b")
    plt.plot(t, x[:, 7] + 3 * np.sqrt(P[:, 6, 6]), 'r--')
    plt.plot(t, x[:, 7] - 3 * np.sqrt(P[:, 6, 6]), 'r--')
    plt.xlabel('t(s)')
    plt.title('Bias component (-)')
    plt.grid()

    return plt


def post_fit_residuals(Res, noise, testName):
    MeasNoise = np.zeros(len(Res[:, 0]))
    t = np.zeros(len(Res[:, 0]))
    for i in range(len(Res[:, 0])):
        t[i] = Res[i, 0] * 1E-9
        MeasNoise[i] = 3 * noise
        # Don't plot zero values, since they mean that no measurement is taken
        for j in range(len(Res[0, :]) - 1):
            if -1E-10 < Res[i, j + 1] < 1E-10:
                Res[i, j + 1] = np.nan

    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    plt.subplot(311)
    plt.plot(t, Res[:, 1], "b.", label='Residual')
    plt.plot(t, MeasNoise, 'r--', label='Covar')
    plt.plot(t, -MeasNoise, 'r--')
    plt.legend(loc='lower right')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('First Meas Comp (m)')
    plt.grid()

    plt.subplot(312)
    plt.plot(t, Res[:, 2], "b.")
    plt.plot(t, MeasNoise, 'r--')
    plt.plot(t, -MeasNoise, 'r--')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('Second Meas Comp (m)')
    plt.grid()

    plt.subplot(313)
    plt.plot(t, Res[:, 3], "b.")
    plt.plot(t, MeasNoise, 'r--')
    plt.plot(t, -MeasNoise, 'r--')
    plt.ylim([-10 * noise, 10 * noise])
    plt.title('Third Meas Comp (m)')
    plt.grid()

    return plt


def two_orbits(r_BN, r_BN2):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('$R_x$, km')
    ax.set_ylabel('$R_y$, km')
    ax.set_zlabel('$R_z$, km')
    ax.plot(r_BN[:, 1] * m2km, r_BN[:, 2] * m2km, r_BN[:, 3] * m2km, color_x, label="True orbit")
    for i in range(len(r_BN2[:, 0])):
        if np.abs(r_BN2[i, 1]) > 0 or np.abs(r_BN2[i, 2]) > 0:
            ax.scatter(r_BN2[i, 1] * m2km, r_BN2[i, 2] * m2km, r_BN2[i, 3] * m2km, color=color_y, label="Meas orbit")
    ax.scatter(0, 0, color='r')
    ax.set_title('Spacecraft Orbits')

    return plt
