# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        rwMotorVoltage
#   Author:             Hanspeter Schaub
#   Creation Date:      January 16, 2017
#

import inspect
import os

import numpy as np
import pytest

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import rwMotorVoltage
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import macros
from Basilisk.architecture import messaging


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("use_large_voltage, use_availability, use_torque_loop, use_high_speed, test_name", [
    (False, False, False, False, "One")
    , (True,  False, False, False, "Two")
    , (False, True,  False, False, "Three")
    , (False, False, True,  False, "Four")
    , (True, False, False,  True,  "Five")
    , (False, False, False, True,  "Six")
])

# update "module" in this function name to reflect the module name
def test_module(show_plots, use_large_voltage, use_availability, use_torque_loop, use_high_speed, test_name):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run(show_plots, use_large_voltage, use_availability, use_torque_loop, use_high_speed, test_name)
    assert testResults < 1, testMessage


def run(show_plots, use_large_voltage, use_availability, use_torque_loop, use_high_speed, test_name):
    test_fail_count = 0                       # zero unit test result counter
    test_messages = []                       # create empty array to store test log messages
    unit_task_name = "unitTask"               # arbitrary name (don't change)
    unit_process_name: str = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module = rwMotorVoltage.RwMotorVoltage()
    module.modelTag = "rwMotorVoltage"

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    # Initialize the test module configuration data
    # set module parameters
    module.VMin = 1.0     # Volts
    module.VMax = 11.0    # Volts
    module.VMaxSat = 5.0    # Volts

    if use_torque_loop:
        module.K = 1.5
        rw_speed_message = messaging.RWSpeedMsgPayload()
        rw_speed_message.wheelSpeeds = [1.0, 2.0, 1.5, -3.0]      # rad/sec Omega's
        rw_speed_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)
        module.rwSpeedInMsg.subscribeTo(rw_speed_in_msg)
        unitTestSupport.writeTeXSnippet("Omega1", r"$\bm\Omega = " \
                                        + str(rw_speed_message.wheelSpeeds[0:4]) + "$"
                                        , path)

    #
    #   create BSK messages
    #
    # Create RW configuration parameter input message
    gs_matrix_b = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0]         # the create routine below normalizes these vectors
    ]
    fswSetupRW.clear_setup()
    for i in range(4):
        fswSetupRW.create(gs_matrix_b[i],        #           spin axis
                          0.1,              # kg*m^2    J2
                          0.2,              # Nm        uMax
                          418.879)          # rad/s     TorqueSatSpeedLimit
    rw_config_in_msg = fswSetupRW.write_config_message()
    module.rwParamsInMsg.subscribeTo(rw_config_in_msg)
    num_rw = fswSetupRW.get_num_of_devices()

    # Create RW motor torque input message
    us_message_data = messaging.ArrayMotorTorqueMsgPayload()
    if use_large_voltage:
        us_message_data.motorTorque = [0.5, 0.0, -0.15, -0.5]           # [Nm] RW motor torque cmds
    else:
        us_message_data.motorTorque = [0.05, 0.0, -0.15, -0.2]  # [Nm] RW motor torque cmds
    rw_motor_torque_in_msg = messaging.ArrayMotorTorqueMsg().write(us_message_data)
    module.torqueInMsg.subscribeTo(rw_motor_torque_in_msg)

    # create RW availability message
    if use_availability:
        rw_availability_message = messaging.RWAvailabilityMsgPayload()
        rw_avail_array = np.zeros(messaging.MAX_EFF_CNT, dtype=int)
        rw_avail_array.fill(messaging.AVAILABLE)
        rw_avail_array[2] = messaging.UNAVAILABLE        # make 3rd RW unavailable
        rw_availability_message.wheelAvailability = rw_avail_array
        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)

    if use_high_speed:
        rw_speed_message = messaging.RWSpeedMsgPayload()
        rw_speed_message.wheelSpeeds = [500.0, 500.0, -500., -500.]     # rad/sec Omega's
        rw_speed_in_msg = messaging.RWSpeedMsg().write(rw_speed_message)
        module.rwSpeedInMsg.subscribeTo(rw_speed_in_msg)
        us_message_data.motorTorque = [0.05, -0.05, -0.05, 0.05]  # [Nm] RW motor torque cmds
        rw_motor_torque_in_msg = messaging.ArrayMotorTorqueMsg().write(us_message_data)
        module.torqueInMsg.subscribeTo(rw_motor_torque_in_msg)
        if use_large_voltage:
            us_message_data.motorTorque = [0.3, -0.3, -0.15, 0.15]  # [Nm] RW motor torque cmds
            rw_motor_torque_in_msg = messaging.ArrayMotorTorqueMsg().write(us_message_data)
            module.torqueInMsg.subscribeTo(rw_motor_torque_in_msg)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.voltageOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # Need to call the self-init and cross-init methods
    unit_test_sim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    if use_torque_loop:
        rw_speed_message.wheelSpeeds = [1.1, 2.1, 1.1, -4.1]  # rad/sec Omega's
        rw_speed_in_msg.write(rw_speed_message)
        unitTestSupport.writeTeXSnippet("Omega2", r"$\bm\Omega = " \
                                        + str(rw_speed_message.wheelSpeeds[0:4]) + "$"
                                        , path)
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.5))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # reset the module to test this functionality
    module.reset(1)     # this module reset function needs a time input (in NanoSeconds)

    # run the module again for an additional 1.0 seconds
    unit_test_sim.ConfigureStopTime(macros.sec2nano(3.0))        # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()


    # This pulls the actual data log from the simulation run.
    module_output = data_log.voltage[:, :num_rw]
    print(module_output)


    # set the filtered output truth states
    true_vector=[]
    if not use_large_voltage and not use_availability and not use_torque_loop and not use_high_speed:
        true_vector = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                   ]
    if use_large_voltage and not use_availability and not use_torque_loop and not use_high_speed:
        true_vector = [
                   [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                 , [11., 0., -8.5, -11.]
                   ]
    if not use_large_voltage and use_availability and not use_torque_loop and not use_high_speed:
        true_vector = [
                   [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                 , [3.5, 0., 0., -11.]
                   ]
    if not use_large_voltage and not use_availability and use_torque_loop and not use_high_speed:
        true_vector = [
                   [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [5.75, -2.5, -11., -9.5]
                 , [3.5, 0., -8.5, -11.]
                 , [3.5, 0., -8.5, -11.]
                 , [7.25, 0., -11., -11.]
                   ]
    if  use_large_voltage and not use_availability and not use_torque_loop and use_high_speed:
        true_vector = [
                  [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
                , [5., -5., -5., 5.]
            ]
    if  not use_large_voltage and not use_availability and not use_torque_loop and use_high_speed:
        true_vector = [
              [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
            , [ 3.5, -3.5,  -3.5 ,  3.5]
        ]


    # compare the module results to the truth values
    accuracy = 1e-10

    test_fail_count, test_messages = unitTestSupport.compareArray(true_vector, module_output,
                                                               accuracy, "Output Vector",
                                                               test_fail_count, test_messages)




    #   print out success message if no error were found
    snippent_name = "passFail" + test_name
    if test_fail_count == 0:
        color_text = 'ForestGreen'
        print("PASSED: " + module.modelTag)
        passed_text = r'\textcolor{' + color_text + '}{' + "PASSED" + '}'
    else:
        color_text = 'Red'
        passed_text = r'\textcolor{' + color_text + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippent_name, passed_text, path)

    # write TeX Tables for documentation
    module_output = add_time_column(data_log.times(), data_log.voltage)[:, :num_rw + 1]
    result_table = module_output
    result_table[:, 0] = macros.NANO2SEC * result_table[:, 0]
    diff = np.delete(module_output, 0, 1) - true_vector
    result_table = np.insert(result_table, list(range(2, 2 + len(diff.transpose()))), diff, axis=1)

    table_name = "test" + str(use_large_voltage) + str(use_availability) + str(use_torque_loop)
    table_headers = ["time [s]", "$V_{s,1}$", "Error", "$V_{s,2}$", "Error", "$V_{s,3}$", "Error", "$V_{s,4}$", "Error"]
    caption = 'RW voltage output for case {\\tt use_large_voltage = ' + str(use_large_voltage) \
              + ', use_availability = ' + str(use_availability) \
              + ', useTorqueLoop = ' + str(use_torque_loop) + '}.'
    unitTestSupport.writeTableLaTeX(
        table_name,
        table_headers,
        caption,
        result_table,
        path)
    unitTestSupport.writeTeXSnippet("us" + str(use_large_voltage) + str(use_availability) + str(use_torque_loop)
                                    , "$\\bm u_s = " + str(us_message_data.motorTorque[0:num_rw]) + "$"
                                    , path)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [test_fail_count, ''.join(test_messages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                  False
                 ,False       # use large voltage
                 ,False         # use availability
                 ,False        # use torque Loop
                 , True
                 ,"Six"      # testName
               )
