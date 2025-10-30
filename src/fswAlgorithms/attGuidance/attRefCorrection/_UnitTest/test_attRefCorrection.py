#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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
#

import math

import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import attRefCorrection
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import unitTestSupport


@pytest.mark.parametrize("accuracy", [1e-12])

def test_att_ref_correction(show_plots, accuracy):
    r"""
    **Validation Test Description**

    Checks the output of the module that the correct orientation adjustment is applied

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    The ``sigma_RN`` variable of the output message is tested
    """
    [test_results, test_message] = att_ref_correction_test_function(show_plots, accuracy)
    assert test_results < 1, test_message


def att_ref_correction_test_function(show_plots, accuracy):
    """Test method"""
    test_fail_count = 0
    test_messages = []
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    unit_test_sim = SimulationBaseClass.SimBaseClass()
    test_process_rate = macros.sec2nano(0.5)
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # setup module to be tested
    module = attRefCorrection.AttRefCorrection()
    module.modelTag = "attRefCorrectionTag"
    unit_test_sim.AddModelToTask(unit_task_name, module)
    module.sigma_BcB = [math.tan(math.pi/4), 0.0, 0.0]

    # Configure blank module input messages
    att_ref_in_msg_data = messaging.AttRefMsgPayload()
    att_ref_in_msg_data.sigma_RN = [math.tan(math.pi/8), 0.0, 0.0]
    att_ref_in_msg = messaging.AttRefMsg().write(att_ref_in_msg_data)

    # subscribe input messages to module
    module.attRefInMsg.subscribeTo(att_ref_in_msg)

    # setup output message recorder objects
    att_ref_out_msg_rec = module.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, att_ref_out_msg_rec)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))
    unit_test_sim.ExecuteSimulation()

    # pull module data and make sure it is correct
    true_vector = [
        [-math.tan(math.pi / 8), 0.0, 0.0],
        [-math.tan(math.pi / 8), 0.0, 0.0],
        [-math.tan(math.pi / 8), 0.0, 0.0]
    ]
    # compare the module results to the truth values
    for i in range(0, len(true_vector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(att_ref_out_msg_rec.sigma_RN[i], true_vector[i], 3, accuracy):
            test_fail_count += 1
            test_messages.append("FAILED: " + module.modelTag + " Module failed sigma_RN unit test at t=" +
                                str(att_ref_out_msg_rec.times()[i] * macros.NANO2SEC) +
                                "sec\n")

    if test_fail_count == 0:
        print("PASSED: " + module.modelTag)
    else:
        print(test_messages)

    return [test_fail_count, "".join(test_messages)]


if __name__ == "__main__":
    test_att_ref_correction(False, 1e-12)
