#
#  ISC License
#
#  Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import inspect
import os

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import rwNullSpace
from xmera.utilities import SimulationBaseClass, macros
from numpy.linalg import inv

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

@pytest.mark.parametrize("num_wheels, default_desired", [(3, True),
                                                         (4, True),
                                                         (3, False),
                                                         (4, False)])

def test_rw_null_space(num_wheels, default_desired):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))  # Add a new task to the process

    # Construct the rwNullSpace module
    module = rwNullSpace.RwNullSpace()
    module.setOmegaGain(.5) # The feedback gain value applied for the RW despin control law
    module.modelTag = "rwNullSpace"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    num_rw = num_wheels

    input_rw_constellation_msg = messaging.RWConstellationMsgPayload()
    input_rw_constellation_msg.numRW = num_rw

    # Initialize the msg that gives the speed of the reaction wheels
    input_speed_msg = messaging.RWSpeedMsgPayload()

    if default_desired:
        desired_omega = [0]*num_rw
    else:
        desired_omega = [5]*num_rw
        input_desired_speed_msg = messaging.RWSpeedMsgPayload()
        input_desired_speed_msg.wheelSpeeds = desired_omega

    gs_hat = [[1, 0, 0], [0,1,0], [0, 0, 1]]
    if num_wheels == 4:
        gs4_hat = np.array([1,1,1])
        gs4_hat = gs4_hat/np.sqrt(gs4_hat.dot(gs4_hat))
        gs_hat.append(gs4_hat.tolist())

    # Iterate over all of the reaction wheels, create a rwConfigElementFswMsg, and add them to the rwConstellationFswMsg
    rw_config_element_list = list()
    for rw in range(num_rw):
        rw_config_element_msg = messaging.RWConfigElementMsgPayload()
        rw_config_element_msg.gsHat_B = gs_hat[rw] # Spin axis unit vector of the wheel in structure
        rw_config_element_msg.Js = 0.08 # Spin axis inertia of wheel [kgm2]
        rw_config_element_msg.uMax = 0.2 # maximum RW motor torque [Nm]

        # Add this to the list
        rw_config_element_list.append(rw_config_element_msg)

    rw_speeds = [10, 20, 30] # [rad/sec] The current angular velocities of the RW wheel
    if num_wheels == 4:
        rw_speeds.append(40)  # [rad/sec]
    input_speed_msg.wheelSpeeds = rw_speeds

    # Set the array of the reaction wheels in RWConstellationFswMsg to the list created above
    input_rw_constellation_msg.reactionWheels = rw_config_element_list

    input_rw_cmd_msg = messaging.RwMotorTorqueMsgPayload()
    us_control = [0.1, 0.2, 0.15] # [Nm] RW motor torque array
    if num_wheels == 4:
        us_control.append(-0.2) # [Nm]
    input_rw_cmd_msg.motorTorque = us_control

    # Set these messages
    rw_speed_msg = messaging.RWSpeedMsg().write(input_speed_msg)
    rw_config_msg = messaging.RWConstellationMsg().write(input_rw_constellation_msg)
    rw_cmd_msg = messaging.RwMotorTorqueMsg().write(input_rw_cmd_msg)

    data_log = module.rwMotorTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.rwMotorTorqueInMsg.subscribeTo(rw_cmd_msg)
    module.rwSpeedsInMsg.subscribeTo(rw_speed_msg)
    module.rwConfigInMsg.subscribeTo(rw_config_msg)
    if not default_desired:
        rw_desired_msg = messaging.RWSpeedMsg().write(input_desired_speed_msg)
        module.rwDesiredSpeedsInMsg.subscribeTo(rw_desired_msg)

    # Initialize the simulation
    unit_test_sim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unit_test_sim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    motor_torque = data_log.motorTorque[:, :num_rw]

    if num_wheels == 3:
        # in this case there is no nullspace of the RW configuration.  The output torque should be the input torque
        true_vector = [input_rw_cmd_msg.motorTorque[:num_rw]]
    elif num_wheels == 4:
        # in this case there is a 1D nullspace of [Gs]
        GsT = np.array(gs_hat)
        Gs = GsT.transpose()
        tmp = Gs.dot(GsT)
        tmp = GsT.dot(inv(tmp))
        tmp = tmp.dot(Gs)
        tau = np.identity(num_wheels) - tmp
        d = - module.getOmegaGain() * (np.array(rw_speeds) - np.array(desired_omega))
        u_null = tau.dot(d)
        true_torque = np.array(us_control) + u_null
        true_vector = [true_torque.tolist()]

    motor_torque_true = true_vector * 5

    accuracy = 1e-12
    np.testing.assert_allclose(motor_torque, motor_torque_true, atol=accuracy, rtol=0, verbose=True)


if __name__ == '__main__':
    test_rw_null_space(4, True)
