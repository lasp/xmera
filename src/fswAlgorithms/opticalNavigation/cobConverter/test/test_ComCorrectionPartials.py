import inspect
import numpy as np
import os
import pytest
from xmera.architecture import messaging
from xmera.utilities import SimulationBaseClass, unitTestSupport, macros
import matplotlib
import matplotlib.pyplot as plt

""" Test that analytical partials match numerical partials """

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

matplotlib.rcParams.update({"font.size": 6})
colorsInt = len(matplotlib.colormaps["inferno"].colors) / (10.0)
colorList = []
for i in range(10):
    colorList.append(matplotlib.colormaps["inferno"].colors[int(i * colorsInt)])

@pytest.mark.parametrize("sunDirection", [[1., 1., -1.], [1., -1., 0.]])
@pytest.mark.parametrize("distance", [500e3, 200e4])


def test_com_partials(show_plots, sunDirection, distance):
    com_correction_partials(sunDirection, distance)

def get_analytical_sigma(sunDirection, distance):

    R_object = 30 * 1e3
    R_object_uncer = 1 * 1e3
    sigma_r = 10e3
    r_BdyZero_N = np.array([-distance, -300. * 1e3, 0.])
    vehSunPntN = np.array(sunDirection) / np.linalg.norm(np.array(sunDirection))
    alpha = np.arccos(-np.dot(r_BdyZero_N.T / np.linalg.norm(r_BdyZero_N), vehSunPntN))

    inputFilter = messaging.FilterMsgPayload()
    inputEphem = messaging.EphemerisMsgPayload()

    full_covariance = np.diag([sigma_r**2, sigma_r**2, sigma_r**2, 0.01, 0.01, 0.01])
    inputFilter.numberOfStates = 6
    inputFilter.covar = full_covariance.flatten()
    position_covar = full_covariance[:3,:3]

    inputEphem.r_BdyZero_N = r_BdyZero_N

    position = r_BdyZero_N
    constants_deltaR = (4*R_object/
                                (3*np.pi*np.linalg.norm(position))*(1 - np.cos(alpha))
                                /(1 + (4*R_object/(3*np.pi*np.linalg.norm(position))
                                *(1 - np.cos(alpha)))**2))

    r_hat = (position / np.linalg.norm(position)).reshape(3,)
    deltaBinary_delta_r = (-r_hat/np.linalg.norm(position)*constants_deltaR)

    deltaBinary_delta_R = (constants_deltaR/R_object)

    deltaBinary_deltaAlpha = ((4*R_object
                                    /(3*np.pi*np.linalg.norm(position))
                                    /((1 + (4*R_object/(3*np.pi*np.linalg.norm(position))
                                    *(1 - np.cos(alpha)))**2))))

    deltaAlpha_delta_R = np.dot(vehSunPntN/ (np.linalg.norm(position)), np.eye(3) - np.outer(r_hat, r_hat))

    deltaBinary_r = deltaBinary_delta_r + np.dot(deltaBinary_deltaAlpha,deltaAlpha_delta_R)
    total_deltaBinary_partials = np.dot(np.dot(deltaBinary_r, position_covar), deltaBinary_r.T)
    sigma_beta_squared  = total_deltaBinary_partials + np.dot(deltaBinary_delta_R **2, R_object_uncer ** 2)

    return sigma_beta_squared

def com_correction_partials(sunDirection,distance):

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    R_object = 30 * 1e3
    R_object_uncer = 1 * 1e3
    sigma_r = 10e3
    r_BdyZero_N = np.array([-distance, -300. * 1e3, 0.])
    vehSunPntN = np.array(sunDirection) / np.linalg.norm(np.array(sunDirection))
    alpha = np.arccos(-np.dot(r_BdyZero_N.T / np.linalg.norm(r_BdyZero_N), vehSunPntN))
    sigma_beta_squared = get_analytical_sigma(sunDirection, distance)

    gamma_mean = 4 / (3 * np.pi) * (1 - np.cos(alpha))
    com_offset_mean = np.arctan(gamma_mean * R_object / np.linalg.norm(r_BdyZero_N)) * 180 / np.pi
    number_of_mcs = 10000
    R = R_object_uncer * np.random.randn(number_of_mcs) + R_object
    P = sigma_r * np.random.randn(number_of_mcs, 3) + r_BdyZero_N
    com_offset_factor = np.zeros(number_of_mcs)
    com_offset = np.zeros(number_of_mcs)

    for i in range(number_of_mcs):
        alpha_mc = np.arccos(-np.dot(P[i].T / np.linalg.norm(P[i]), vehSunPntN))
        gamma = 4 / (3 * np.pi) * (1 - np.cos(alpha_mc))
        com_offset_factor[i] = gamma
        com_offset[i] = np.arctan(gamma * R[i] / np.linalg.norm(P[i])) * 180 / np.pi * 3600 - com_offset_mean * 3600

    com_offset_errors = np.array(com_offset).T

    plt.figure(facecolor="w", edgecolor="k")
    plt.hist(com_offset_errors, bins=25, color=colorList[1])
    plt.axvline(
            2 * np.std(com_offset_errors),
            color=colorList[9],
            linewidth=2,
            linestyle="dashed",
            label="2-$\sigma$ numerical",
            )
    plt.axvline(
            -2 * np.std(com_offset_errors),
            color=colorList[9],
            linewidth=2,
            linestyle="dashed",
            )
    plt.axvline(
            2 * np.sqrt(sigma_beta_squared) * 180 / np.pi * 3600,
            color=colorList[7],
            linewidth=2,
            linestyle="dashed",
            label="2-$\sigma$ analytical",
        )
    plt.axvline(
            -2 * np.sqrt(sigma_beta_squared) * 180 / np.pi * 3600,
            color=colorList[7],
            linewidth=2,
            linestyle="dashed",
      )
    plt.gca().ticklabel_format(axis="x", style="sci", scilimits=(0, 0))
    plt.xlabel("COM Angle Error (arcsec)")
    plt.ylabel("Number of runs")
    plt.legend()
    plt.tight_layout()


    np.testing.assert_allclose(
        np.linalg.norm(np.sqrt(sigma_beta_squared) * 180 / np.pi * 3600),
        np.linalg.norm(np.std(com_offset_errors)),
        rtol=2,
        equal_nan=False,
        err_msg="sigma_beta error",
    )

if __name__ == "__main__":
    com_correction_partials([-1., -1., 0.], 3e6)
