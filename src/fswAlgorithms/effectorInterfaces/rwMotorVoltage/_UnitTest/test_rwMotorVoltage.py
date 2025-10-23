# ISC License
#
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect
import os

import numpy as np
import pytest

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import rwMotorVoltage
from xmera.utilities import fswSetupRW
from xmera.utilities import macros
from xmera.architecture import messaging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("use_large_voltage, use_availability, use_torque_loop", [(False, False, False),
                                                                                  (True, False, False),
                                                                                  (False, True, False),
                                                                                  (False, False, True)])

def test_rw_motor_voltage(show_plots, use_large_voltage, use_availability, use_torque_loop):
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name = "TestProcess"         # arbitrary name (don't change)

    unit_test_sim = SimulationBaseClass.SimBaseClass()

    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = rwMotorVoltage.RwMotorVoltage(1.0, 10.0)
    module.modelTag = "rwMotorVoltage"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    module.setVoltageRange(1.0, 11.0)  # test setter method

    if use_torque_loop:
        module.setGainK(1.5)
        rw_speed_message = messaging.RWSpeedMsgPayload()
        rw_speed_message.wheelSpeeds = [1.0, 2.0, 1.5, -3.0]      # rad/sec Omega's
        rw_speed_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)
        module.rwSpeedInMsg.subscribeTo(rw_speed_in_msg)

    # Create RW configuration parameter input message
    G_s_B = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0]
    ]  # the create routine below normalizes these vectors
    fswSetupRW.clearSetup()
    for i in range(4):
        fswSetupRW.create(G_s_B[i],    #           spin axis
                          0.1,              # kg*m^2    J2
                          0.2)              # Nm        uMax
    rw_config_in_msg = fswSetupRW.writeConfigMessage()
    module.rwParamsInMsg.subscribeTo(rw_config_in_msg)
    num_rw = fswSetupRW.getNumOfDevices()

    # Create RW motor torque input message
    rw_torque_msg = messaging.RwMotorTorqueMsgPayload()
    if use_large_voltage:
        rw_torque_msg.motorTorque = [0.5, 0.0, -0.15, -0.5]           # [Nm] RW motor torque cmds
    else:
        rw_torque_msg.motorTorque = [0.05, 0.0, -0.15, -0.2]  # [Nm] RW motor torque cmds
    rw_motor_torque_in_msg = messaging.RwMotorTorqueMsg().write(rw_torque_msg)
    module.torqueInMsg.subscribeTo(rw_motor_torque_in_msg)

    # create RW availability message
    if use_availability:
        rw_availability_message = messaging.RWAvailabilityMsgPayload()
        rw_avail_array = np.zeros(messaging.RW_EFF_CNT, dtype=int)
        rw_avail_array.fill(messaging.AVAILABLE)
        rw_avail_array[2] = messaging.UNAVAILABLE        # make 3rd RW unavailable
        rw_availability_message.wheelAvailability = rw_avail_array
        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.voltageOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    if use_torque_loop:
        rw_speed_message.wheelSpeeds = [1.1, 2.1, 1.1, -4.1]  # rad/sec Omega's
        rw_speed_in_msg.write(rw_speed_message)

    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.5))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # reset the module to test this functionality
    module.reset(1)     # this module reset function needs a time input (in NanoSeconds)

    # run the module again for an additional 1.0 seconds
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    voltage = data_log.voltage[:, :num_rw]

    # set the filtered output truth states
    voltage_true =[]
    if not use_large_voltage and not use_availability and not use_torque_loop:
        voltage_true = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                   ]
    if use_large_voltage and not use_availability and not use_torque_loop:
        voltage_true = [
                   [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                   ]
    if not use_large_voltage and use_availability and not use_torque_loop:
        voltage_true = [
                   [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                   ]
    if not use_large_voltage and not use_availability and use_torque_loop:
        voltage_true = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [5.75, -2.5, -11., -9.5]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [7.25, 0., -11., -11.]
                   ]

    # compare the module results to the truth values
    accuracy = 1e-10

    np.testing.assert_allclose(voltage, voltage_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_rw_motor_voltage(False,
                          False,
                          False,
                          True,
                          "Four")
