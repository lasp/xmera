# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Module Name:        encoder
"""

import numpy as np

from xmera.architecture import messaging
from xmera.simulation import encoder
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def test_encoder():
    r"""
    **Validation Test Description**

    This test makes sure that the Python bindings of the encoder operate in a simulation. The C++ unit tests in
    this folder examine the encoder behavior in more detail.

    The test connects the encoder to a reaction wheel speed message and simulates two steps of one second. The
    encoder uses two clicks for each rotation.

    **Description of Variables Being Tested**

    The test compares the ``wheelSpeeds`` field of the encoder output message with the known values.
    """
    task_name = "unitTask"
    num_rw = 3

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("TestProcess")
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(1)))

    speed_payload = messaging.RWSpeedMsgPayload()
    speed_payload.wheelSpeeds = [100, 200, 300]
    speed_msg = messaging.RWSpeedMsg().write(speed_payload)

    wheel_speed_encoder = encoder.Encoder()
    wheel_speed_encoder.modelTag = "rwSpeedsEncoder"
    wheel_speed_encoder.clicksPerRotation = 2
    wheel_speed_encoder.numRW = num_rw
    wheel_speed_encoder.rwSpeedInMsg.subscribeTo(speed_msg)
    sim.AddModelToTask(task_name, wheel_speed_encoder)

    encoded_log = wheel_speed_encoder.rwSpeedOutMsg.recorder()
    sim.AddModelToTask(task_name, encoded_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1))
    sim.ExecuteSimulation()

    encoded_speeds = np.array(encoded_log.wheelSpeeds)[:, 0:num_rw]
    true_encoded_speeds = np.array([[100.0, 200.0, 300.0],
                                    [31.0 * np.pi, 63.0 * np.pi, 95.0 * np.pi]])

    np.testing.assert_allclose(encoded_speeds, true_encoded_speeds, rtol=0.0, atol=1e-8)


if __name__ == "__main__":
    test_encoder()
