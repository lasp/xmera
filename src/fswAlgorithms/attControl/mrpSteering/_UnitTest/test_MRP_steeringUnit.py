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

import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import mrpSteering  # import the module that is to be tested
from xmera.utilities import RigidBodyKinematics
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

@pytest.mark.parametrize("K1", [0.15, 0])
@pytest.mark.parametrize("K3", [1.0, 0])
@pytest.mark.parametrize("omega_max", [1.5 * macros.D2R, 0.001 * macros.D2R])
@pytest.mark.parametrize("ignore_feed_forward", [True, False])

def test_mrp_steering_tracking(show_plots, K1, K3, omega_max, ignore_feed_forward):
    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)  # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = mrpSteering.MrpSteering()
    module.modelTag = "mrpSteering"
    unit_test_sim.AddModelToTask(unit_task_name, module)

    module.setK1(K1)
    module.setK3(K3)
    module.setOmegaMax(omega_max)
    module.setIgnoreFeedforward(ignore_feed_forward)

    guid_cmd_data = messaging.AttGuidMsgPayload()  # Create a structure for the input message
    sigma_BR = np.array([0.3, -0.5, 0.7])
    guid_cmd_data.sigma_BR = sigma_BR
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guid_cmd_data.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guid_cmd_data.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guid_cmd_data.domega_RN_B = domega_RN_B
    guid_in_msg = messaging.AttGuidMsg().write(guid_cmd_data)

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.rateCmdOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.guidInMsg.subscribeTo(guid_in_msg)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unit_test_sim.ExecuteSimulation()

    # Compute truth states
    omega_ast_true, omega_ast_p_true = find_true_values(guid_cmd_data, module)

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(data_log.omega_BastR_B, omega_ast_true, atol=accuracy, rtol=0, verbose=True)
    np.testing.assert_allclose(data_log.omegap_BastR_B, omega_ast_p_true, atol=accuracy, rtol=0, verbose=True)


def find_true_values(guid_cmd_data, module):

    omega_max = module.getOmegaMax()
    sigma = np.asarray(guid_cmd_data.sigma_BR)
    K1 = np.asarray(module.getK1())
    K3 = np.asarray(module.getK3())
    B = RigidBodyKinematics.BmatMRP(sigma)
    omega_ast = []
    omega_ast_p = []

    for i in range(len(sigma)):
        steer_rate = -1*(2*omega_max/np.pi)*np.arctan((K1*sigma[i]+K3*sigma[i]*sigma[i]*sigma[i])*np.pi/(2*omega_max))
        omega_ast.append(steer_rate)


    if not module.getIgnoreFeedforward():
        sigma_p = 0.25*B.dot(omega_ast)
        for i in range(len(sigma)):
            omega_ast_rate = (K1+3*K3*sigma[i]**2)/(1+((K1*sigma[i]+K3*sigma[i]**3)**2)*(np.pi/(2*omega_max))**2)*sigma_p[i]
            omega_ast_p.append(-omega_ast_rate)
    else:
        omega_ast_p = np.asarray([0, 0, 0])

    omega_ast = [omega_ast, omega_ast, omega_ast]
    omega_ast_p = [omega_ast_p, omega_ast_p, omega_ast_p]

    return omega_ast, omega_ast_p


if __name__ == "__main__":
    test_mrp_steering_tracking(False, 0.1, 1.0, 1.0, False)
