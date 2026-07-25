# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import inspect
import os

import numpy as np

# Spice Spacecraft Unit Test
# Purpose:  Verify that SpiceInterface produces correct spacecraft state, attitude
#           reference, and translational reference messages for a SPICE-tracked
#           spacecraft (HST), benchmarked against JPL Horizons.

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from xmera import __path__
bsk_path = __path__[0]

import xmera.architecture.messaging  # noqa: F401
from xmera.utilities import unitTestSupport
from xmera.utilities import SimulationBaseClass
from xmera.simulation import spiceInterface
from xmera.utilities import macros


def test_unitSpiceSc(show_plots):
    """Module Unit Test"""
    [test_fail_count, test_message] = unit_spice_sc(show_plots)
    assert test_fail_count < 1, test_message


def unit_spice_sc(show_plots):
    test_fail_count = 0
    test_messages = []

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    total_sim = SimulationBaseClass.SimBaseClass()
    dyn_unit_test_proc = total_sim.CreateNewProcess(unit_process_name)
    dyn_unit_test_proc.addTask(unit_task_name, macros.sec2nano(0.1))
    date_spice = "2015 February 10, 00:00:00.0 TDB"

    spice_object = spiceInterface.SpiceInterface()
    spice_object.modelTag = "SpiceInterfaceData"
    spice_object.SPICEDataPath = bsk_path + "/supportData/EphemerisData/"
    sc_names = ["HUBBLE SPACE TELESCOPE"]
    spice_object.addSpacecraftNames(sc_names)
    spice_object.UTCCalInit = date_spice
    spice_object.zeroBase = "earth"
    spice_object.loadSpiceKernel("hst_edited.bsp", bsk_path + "/supportData/EphemerisData/")

    total_sim.AddModelToTask(unit_task_name, spice_object)

    total_sim.ConfigureStopTime(macros.sec2nano(0.1))
    total_sim.InitializeSimulation()
    total_sim.ExecuteSimulation()

    spice_object.unloadSpiceKernel("hst_edited.bsp", bsk_path + "/supportData/EphemerisData/")

    truth_position = np.array([-5855529.540348052, 1986110.860522791, -3116764.7117067943])
    truth_velocity = np.array([-1848.9038338503085, -7268.515626753905, -1155.3578832725618])
    truth_att = np.array([0., 0., 0.])
    truth_zero = np.array([0., 0., 0.])
    accuracy = 0.01

    sc_state_msg = spice_object.scStateOutMsgs[0].read()
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_position, sc_state_msg.r_BN_N,
                                                                   accuracy, "scState-r_BN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_position, sc_state_msg.r_CN_N,
                                                                   accuracy, "scState-r_CN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_velocity, sc_state_msg.v_BN_N,
                                                                   accuracy, "scState-v_BN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_velocity, sc_state_msg.v_CN_N,
                                                                   accuracy, "scState-v_CN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_att, sc_state_msg.sigma_BN,
                                                                   accuracy, "scState-sigma_BN",
                                                                   test_fail_count, test_messages)

    att_state_msg = spice_object.attRefStateOutMsgs[0].read()
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_att, att_state_msg.sigma_RN,
                                                                   accuracy, "scState-sigma_RN",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_zero, att_state_msg.omega_RN_N,
                                                                   accuracy, "scState-omega_RN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_zero, att_state_msg.domega_RN_N,
                                                                   accuracy, "scState-domega_RN_N",
                                                                   test_fail_count, test_messages)

    trans_state_msg = spice_object.transRefStateOutMsgs[0].read()
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_position, trans_state_msg.r_RN_N,
                                                                   accuracy, "scState-r_RN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_velocity, trans_state_msg.v_RN_N,
                                                                   accuracy, "scState-v_RN_N",
                                                                   test_fail_count, test_messages)
    test_fail_count, test_messages = unitTestSupport.compareVector(truth_zero, trans_state_msg.a_RN_N,
                                                                   accuracy, "scState-a_RN_N",
                                                                   test_fail_count, test_messages)

    if test_fail_count == 0:
        print(" \n PASSED ")

    return [test_fail_count, "".join(test_messages)]


if __name__ == "__main__":
    test_unitSpiceSc(False)
