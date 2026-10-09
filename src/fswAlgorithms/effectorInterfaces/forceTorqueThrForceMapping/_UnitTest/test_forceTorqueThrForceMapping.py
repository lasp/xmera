# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Interface test for the forceTorqueThrForceMapping module. The C++ gtest in this directory
# examines the numerical solution and the reset checks. This file shows how to connect the
# module in a simulation and shows its behavior.

import sys

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import forceTorqueThrForceMapping
from xmera.utilities import SimulationBaseClass
from xmera.utilities import fswSetupThrusters
from xmera.utilities import macros

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="known to not pass on windows platform")

TASK_NAME = "unitTask"
TASK_RATE = macros.sec2nano(0.5)
COM_B = np.array([0.1, 0.1, 0.1])

# Eight thrusters at the corners of a box. No thruster points along z.
RCS_LOCATIONS = np.array([[-0.86360, -0.82550, 1.79070],
                          [-0.82550, -0.86360, 1.79070],
                          [0.82550, 0.86360, 1.79070],
                          [0.86360, 0.82550, 1.79070],
                          [-0.86360, -0.82550, -1.79070],
                          [-0.82550, -0.86360, -1.79070],
                          [0.82550, 0.86360, -1.79070],
                          [0.86360, 0.82550, -1.79070]])
RCS_DIRECTIONS = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, -1.0, 0.0],
                           [-1.0, 0.0, 0.0],
                           [1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, -1.0, 0.0],
                           [-1.0, 0.0, 0.0]])
NUM_THRUSTERS = len(RCS_LOCATIONS)


def setup_sim(force_request):
    """Make a simulation with the module and a force command. The torque message is not connected.

    Returns (sim, module, force_msg, recorder, config_msgs). Keep config_msgs alive while the
    simulation runs.
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("TestProcess")
    process.addTask(sim.CreateNewTask(TASK_NAME, TASK_RATE))

    module = forceTorqueThrForceMapping.ForceTorqueThrForceMapping()
    module.modelTag = "forceTorqueThrForceMapping"
    sim.AddModelToTask(TASK_NAME, module)

    fswSetupThrusters.clearSetup()
    for location, direction in zip(RCS_LOCATIONS, RCS_DIRECTIONS):
        fswSetupThrusters.create(location, direction, 3.0)
    thr_config_msg = fswSetupThrusters.writeConfigMessage()

    veh_config = messaging.VehicleConfigMsgPayload()
    veh_config.CoM_B = COM_B
    veh_config_msg = messaging.VehicleConfigMsg().write(veh_config)

    force_cmd = messaging.CmdForceBodyMsgPayload()
    force_cmd.forceRequestBody = force_request
    force_msg = messaging.CmdForceBodyMsg().write(force_cmd)

    module.thrConfigInMsg.subscribeTo(thr_config_msg)
    module.vehConfigInMsg.subscribeTo(veh_config_msg)
    module.cmdForceInMsg.subscribeTo(force_msg)

    recorder = module.thrForceCmdOutMsg.recorder()
    sim.AddModelToTask(TASK_NAME, recorder)

    return sim, module, force_msg, recorder, (thr_config_msg, veh_config_msg)


def body_force_and_torque(thr_force):
    """Return the total body force and the torque about the center of mass from the thruster forces."""
    force_vectors = thr_force[:, None] * RCS_DIRECTIONS
    torque = np.cross(RCS_LOCATIONS - COM_B, force_vectors).sum(axis=0)
    return force_vectors.sum(axis=0), torque


def test_thrusters_produce_requested_force():
    """The thruster forces together give the requested force and no torque."""
    force_request = [1.0, 0.0, 0.0]
    sim, module, _, _, _ = setup_sim(force_request)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()

    thr_force = np.array(module.thrForceCmdOutMsg.read().thrForce[:NUM_THRUSTERS])
    force, torque = body_force_and_torque(thr_force)

    assert np.all(thr_force >= 0.0)
    np.testing.assert_allclose(force, force_request, atol=1e-12)
    np.testing.assert_allclose(torque, np.zeros(3), atol=1e-12)


def test_output_follows_force_command():
    """The module reads the force command on each step. Two times the force gives two times the thruster forces."""
    sim, _, force_msg, recorder, _ = setup_sim([1.0, 0.0, 0.0])

    sim.InitializeSimulation()
    sim.ConfigureStopTime(TASK_RATE)
    sim.ExecuteSimulation()

    force_cmd = messaging.CmdForceBodyMsgPayload()
    force_cmd.forceRequestBody = [2.0, 0.0, 0.0]
    force_msg.write(force_cmd)
    sim.ConfigureStopTime(2 * TASK_RATE)
    sim.ExecuteSimulation()

    thr_force = recorder.thrForce[:, :NUM_THRUSTERS]
    np.testing.assert_allclose(thr_force[1], thr_force[0], atol=1e-12)
    np.testing.assert_allclose(thr_force[2], 2.0 * thr_force[0], atol=1e-12)


if __name__ == "__main__":
    test_thrusters_produce_requested_force()
    test_output_follows_force_command()
