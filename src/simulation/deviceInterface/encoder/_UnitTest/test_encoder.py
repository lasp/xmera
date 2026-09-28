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


def write_speed_message(wheel_speeds):
    """Make a reaction wheel speed message that contains the given wheel speeds."""
    payload = messaging.RWSpeedMsgPayload()
    payload.wheelSpeeds = wheel_speeds
    return messaging.RWSpeedMsg().write(payload)


def test_encoder():
    r"""
    **Validation Test Description**

    This test simulates the encoder for six steps with three reaction wheels. The encoder uses two clicks for each
    rotation. The test changes the input speeds and the signal state between the steps. The encoder output must
    agree with the known output speeds for the nominal, off and stuck signal states.

    **Description of Variables Being Tested**

    The test compares the ``wheelSpeeds`` field of the encoder output message with the known values.
    """
    task_name = "unitTask"
    process_name = "TestProcess"
    num_rw = 3

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess(process_name)
    process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(1)))

    speed_msg = write_speed_message([100, 200, 300])

    wheel_speed_encoder = encoder.Encoder()
    wheel_speed_encoder.modelTag = "rwSpeedsEncoder"
    wheel_speed_encoder.clicksPerRotation = 2
    wheel_speed_encoder.numRW = num_rw
    wheel_speed_encoder.rwSpeedInMsg.subscribeTo(speed_msg)
    sim.AddModelToTask(task_name, wheel_speed_encoder)

    encoded_log = wheel_speed_encoder.rwSpeedOutMsg.recorder()
    sim.AddModelToTask(task_name, encoded_log)

    sim.InitializeSimulation()
    for _ in range(3):
        sim.TotalSim.singleStepProcesses()

    wheel_speed_encoder.rwSignalState = [encoder.SIGNAL_OFF] * num_rw
    sim.TotalSim.singleStepProcesses()

    speed_msg = write_speed_message([500, 400, 300])
    wheel_speed_encoder.rwSpeedInMsg.subscribeTo(speed_msg)
    wheel_speed_encoder.rwSignalState = [encoder.SIGNAL_NOMINAL] * num_rw
    sim.TotalSim.singleStepProcesses()

    speed_msg = write_speed_message([100, 200, 300])
    wheel_speed_encoder.rwSpeedInMsg.subscribeTo(speed_msg)
    wheel_speed_encoder.rwSignalState = [encoder.SIGNAL_STUCK] * num_rw
    sim.TotalSim.singleStepProcesses()

    encoded_speeds = np.array(encoded_log.wheelSpeeds)[:, 0:num_rw]
    true_encoded_speeds = np.array([[100.0, 200.0, 300.0],
                                    [31.0 * np.pi, 63.0 * np.pi, 95.0 * np.pi],
                                    [32.0 * np.pi, 64.0 * np.pi, 95.0 * np.pi],
                                    [0.0, 0.0, 0.0],
                                    [159.0 * np.pi, 127.0 * np.pi, 95.0 * np.pi],
                                    [159.0 * np.pi, 127.0 * np.pi, 95.0 * np.pi]])

    np.testing.assert_allclose(encoded_speeds, true_encoded_speeds, rtol=0.0, atol=1e-8)


if __name__ == "__main__":
    test_encoder()
