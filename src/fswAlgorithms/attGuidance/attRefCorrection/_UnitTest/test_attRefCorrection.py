import math
import numpy as np

from xmera.architecture import messaging
from xmera.fswAlgorithms import attRefCorrection
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros

def test_att_ref_correction():
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
    sigma_RR0 = [[math.tan(math.pi/16)], [0.0], [0.0]]
    module.sigma_RR0 = sigma_RR0

    # Configure blank module input messages
    att_ref_in_msg_data = messaging.AttRefMsgPayload()
    att_ref_in_msg_data.sigma_RN = [math.tan(math.pi/16), 0.0, 0.0]
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
        [math.tan(math.pi / 8), 0.0, 0.0],
        [math.tan(math.pi / 8), 0.0, 0.0],
        [math.tan(math.pi / 8), 0.0, 0.0]
    ]

    # compare the module results to the truth values
    accuracy = 1e-12
    np.testing.assert_allclose(sigma_RR0, module.sigma_RR0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(att_ref_out_msg_rec.sigma_RN, true_vector, atol=accuracy, rtol=0, verbose=True)


if __name__ == "__main__":
    test_att_ref_correction()
