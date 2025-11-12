import numpy as np
import pytest
from xmera.architecture import messaging
from xmera.fswAlgorithms import thrForceMapping
from xmera.utilities import (macros, fswSetupThrusters, SimulationBaseClass)
from .Support.thruster_force_mapping_test_oracle import ThrForceMappingTestOracle


@pytest.mark.parametrize("useDVThruster", [True, False])
@pytest.mark.parametrize(["useCOMOffset","dropThruster", "use2ndLoop"],[
                         (False, 0, False),
                         (False, 1, True),
                         (False, 2, True), # Any time we drop a thruster, we should recompute the solution
                         (True, 0, False)]) # We don't handle the case where there is a dropped thruster and COM offset--see performance analysis.
@pytest.mark.parametrize("asymmetricDrop", [False])
@pytest.mark.parametrize("numControlAxis", [1, 2, 3])
@pytest.mark.parametrize("saturateThrusters", [0,1,2])
def test_thrusterForceTest(show_plots, useDVThruster, useCOMOffset, dropThruster, asymmetricDrop, numControlAxis,
                      saturateThrusters, use2ndLoop):
    if useDVThruster and numControlAxis == 3:
        pytest.skip(f"Skipping this combination {useDVThruster} and {numControlAxis}. Three control axes doesn't work "
                    f"for the dv thruster setup used in this test (only two axes controllable)")

    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    module = thrForceMapping.ThrForceMapping()
    module.setEpsilon(0.0005)
    module.setUse2ndLoop(use2ndLoop)
    module.modelTag = "thrForceMapping"

    # Add test module to the runtime task
    unitTestSim.AddModelToTask(unitTaskName, module)

    # write a vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    if useCOMOffset == 1:
        CoM_B = [0.03,0.001,0.02]
    else:
        CoM_B = [0,0,0]
    vehicleConfigOut.CoM_B = CoM_B
    vcInMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Create an input message and size it because the regular creator of that message
    # is not part of the test.
    inputMessageData = messaging.CmdTorqueBodyMsgPayload()  # Create a structure for the input message
    requestedTorque = [1.0, -0.5, 0.7]             # Set up a list as a 3-vector
    if saturateThrusters>0:        # default angErrThresh is 0, thus this should trigger scaling
        requestedTorque = [10.0, -5.0, 7.0]
    if saturateThrusters==2:        # angle is set and small enough to trigger scaling
        module.setAngErrThresh(10.0*macros.D2R)
    if saturateThrusters==3:        # angle is too large enough to trigger scaling
        module.setAngErrThresh(40.0*macros.D2R)

    inputMessageData.torqueRequestBody = requestedTorque   # write torque request to input message
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(inputMessageData)

    controlAxes_B = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])
    start_index = -controlAxes_B.shape[0] + numControlAxis
    if start_index != 0:
        controlAxes_B[start_index:] = 0
    module.setControlAxesB(controlAxes_B)

    rcsLocationData = np.zeros((messaging.MAX_EFF_CNT, 3))
    rcsDirectionData = np.zeros((messaging.MAX_EFF_CNT, 3))
    if useDVThruster:
        # DV thruster setup
        module.setThrForceSign(-1)
        numThrusters = 6
        rcsLocationData[0:6] = [
            [0, 0.413, -0.1671],
            [0, -0.413, -0.1671],
            [0.35766849176297305, 0.20650000000000013, -0.1671],
            [0.3576684917629732, -0.20649999999999988, -0.1671],
            [-0.35766849176297333, 0.20649999999999968, -0.1671],
            [-0.35766849176297305, -0.20650000000000018, -0.1671]
        ]
        rcsDirectionData[0:6] = [
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0]
        ]
    else:
        # RCS thruster setup
        module.setThrForceSign(1)
        numThrusters = 8
        rcsLocationData[0:8] = [
            [-0.86360, -0.82550, 1.79070],
                [-0.82550, -0.86360, 1.79070],
                [0.82550, 0.86360, 1.79070],
                [0.86360, 0.82550, 1.79070],
                [-0.86360, -0.82550, -1.79070],
                [-0.82550, -0.86360, -1.79070],
                [0.82550, 0.86360, -1.79070],
                [0.86360, 0.82550, -1.79070]
        ]

        rcsDirectionData[0:8] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0]
        ]

    if dropThruster > 0:
        if (dropThruster % 2==0) and asymmetricDrop: # Drop thrusters that don't share the same torque direction
            removedThrusters = 0
            for i in range(0, numThrusters, 2):
                rcsLocationData[i] = [0.0, 0.0, 0.0]
                rcsDirectionData[i] = [0.0, 0.0, 0.0]
                removedThrusters += 1
            if removedThrusters < dropThruster:
                rcsLocationData[1] = [0.0, 0.0, 0.0]
                removedThrusters += 1
        else:
            for i in range(dropThruster):
                rcsLocationData[numThrusters - 1 - i, :] = [0.0, 0.0, 0.0]
                rcsDirectionData[numThrusters - 1 - i, :] = [0.0, 0.0, 0.0]

        indices = []
        for i in range(numThrusters):
            if np.linalg.norm(rcsLocationData[i]) == 0:
                indices = np.append(indices, i)

        offset = 0
        for i in indices:
            idx = (int) (i - offset)
            rcsLocationData = np.delete(rcsLocationData, idx, axis=0)
            rcsDirectionData = np.delete(rcsDirectionData, idx, axis=0)
            rcsLocationData = np.append(rcsLocationData,[[0.0, 0.0, 0.0]], axis=0)
            rcsDirectionData = np.append(rcsDirectionData, [[0.0, 0.0, 0.0]], axis=0)
            offset = offset + 1

        numThrusters = numThrusters - dropThruster

    maxThrust = 0.95
    if useDVThruster:
        maxThrust = 10.0

    fswSetupThrusters.clearSetup()
    for i in range(numThrusters):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrust)
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = module.thrForceCmdOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)
    module.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    module.vehConfigInMsg.subscribeTo(vcInMsg)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The simulation is stopped at the next logging
    # event on or after the simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    moduleOutput = dataLog.thrForce

    test_oracle = ThrForceMappingTestOracle(requestedTorque,
                                            module.getControlAxesB(),
                                            vehicleConfigOut.CoM_B,
                                            rcsLocationData,
                                            rcsDirectionData,
                                            module.getThrForceSign(),
                                            module.getThrForceMag(),
                                            module.getAngErrThresh(),
                                            numThrusters,
                                            module.getEpsilon(),
                                            use2ndLoop)

    F, DNew = test_oracle.results_thrForceMapping()

    trueVector = np.zeros((2, messaging.MAX_EFF_CNT))
    trueVector[0,:] = F
    trueVector[1,:] = F

    C = controlAxes_B
    CT = np.transpose(C)
    D = np.cross(rcsDirectionData, rcsLocationData-CoM_B)
    receivedTorque = -1.0*np.array([np.matmul(np.transpose(D), np.transpose(moduleOutput[0]))])
    receivedTorque = np.append(np.array([]), receivedTorque)

    Lr_offset = np.array([0.0, 0.0, 0.0])
    Lr_B = np.array([0.0, 0.0, 0.0])
    for i in range(0,numThrusters):
        if module.getThrForceSign() < 0 and module.getThrForceMag()[i][0] >= 0:
            Lr_offset -= module.getThrForceMag()[i][0]*np.cross(rcsLocationData[i,:]-CoM_B, rcsDirectionData[i,:]) # off pulsing

    Lr_B = requestedTorque + Lr_offset

    # This is the requested and received torque projected onto the control axes
    Lr_Req_Bar_B = np.matmul(CT, np.matmul(C, Lr_B))
    Lr_Rec_Bar_B = np.matmul(CT, np.matmul(C, receivedTorque))

    # This computes the projected requested and received control torque directions
    Lr_Req_Bar_B_Unit = Lr_Req_Bar_B/np.linalg.norm(Lr_Req_Bar_B)
    Lr_Rec_Bar_B_Unit = Lr_Rec_Bar_B/np.linalg.norm(Lr_Rec_Bar_B)
    if np.linalg.norm(Lr_Rec_Bar_B) == 0.0:
        Lr_Rec_Bar_B_Unit = [0.0, 0.0, 0.0]

    # Check that Python Math and C Math are Identical
    np.testing.assert_allclose(np.array([moduleOutput[0]]),
                               np.array([F]),
                               rtol=0.0,
                               atol=1E-6,
                               equal_nan=False,
                               err_msg="CompareForces")

    # Checks to make sure that no forces are negative
    if not useDVThruster:
        assert np.any(moduleOutput[0] >= 0), "A negative force exists in the C RCS solution. This is not allowed."
        assert np.any(F >= 0), "A negative force exists in the Python RCS solution. This is not allowed."

    np.testing.assert_allclose(np.array([Lr_Rec_Bar_B_Unit]),
                               np.array([Lr_Req_Bar_B_Unit]),
                               rtol=0.0,
                               atol=1E-6,
                               equal_nan=False,
                               err_msg="CompareTorques")


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_thrusterForceTest(False,
                           False,  # useDVThruster
                           False,  # use COM offset
                           2,  # num drop thruster(s)
                           False,  # asymmetric drop
                           3,  # num control axis
                           2,  # saturateThrusters
                           True  # Use 2nd loop
                           )
