#
#   Unit Test Script
#   Module Name:        navAggregate()
#   Author:             Hanspeter Schaub
#   Creation Date:      Feb. 21, 2019
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport
from xmera.fswAlgorithms import navAggregate
from xmera.utilities import macros
from xmera.architecture import messaging

# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("numAttNav, numTransNav", [
      (0, 0)
    , (1, 1)
    , (0, 1)
    , (1, 0)
    , (2, 2)
    , (1, 2)
    , (0, 2)
    , (2, 1)
    , (2, 0)
    , (3, 3)
    , (3, 2)
    , (3, 1)
    , (3, 0)
    , (2, 3)
    , (1, 3)
    , (0, 3)
])

# update "module" in this function name to reflect the module name
def test_navAggregate(show_plots, numAttNav, numTransNav):
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct an instance of the module being tested
    module = navAggregate.NavAggregate()
    module.modelTag = "navAggregate"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Create input messages
    navAtt1Msg = messaging.NavAttMsgPayload()
    navAtt1Msg.timeTag = 11.11
    navAtt1Msg.sigma_BN = [0.1, 0.01, -0.1]
    navAtt1Msg.omega_BN_B = [1., 1., -1.]
    navAtt1Msg.vehSunPntBdy = [-0.1, 0.1, 0.1]
    navAtt1InMsg = messaging.NavAttMsg().write(navAtt1Msg)
    navAtt2Msg = messaging.NavAttMsgPayload()
    navAtt2Msg.timeTag = 22.22
    navAtt2Msg.sigma_BN = [0.2, 0.02, -0.2]
    navAtt2Msg.omega_BN_B = [2., 2., -2.]
    navAtt2Msg.vehSunPntBdy = [-0.2, 0.2, 0.2]
    navAtt2InMsg = messaging.NavAttMsg().write(navAtt2Msg)

    navTrans1Msg = messaging.NavTransMsgPayload()
    navTrans1Msg.timeTag = 11.1
    navTrans1Msg.r_BN_N = [1000.0, 100.0, -1000.0]
    navTrans1Msg.v_BN_N = [1., 1., -1.]
    navTrans1Msg.vehAccumDV = [-10.1, 10.1, 10.1]
    navTrans1InMsg = messaging.NavTransMsg().write(navTrans1Msg)
    navTrans2Msg = messaging.NavTransMsgPayload()
    navTrans2Msg.timeTag = 22.2
    navTrans2Msg.r_BN_N = [2000.0, 200.0, -2000.0]
    navTrans2Msg.v_BN_N = [2., 2., -2.]
    navTrans2Msg.vehAccumDV = [-20.2, 20.2, 20.2]
    navTrans2InMsg = messaging.NavTransMsg().write(navTrans2Msg)

    # create input navigation message containers
    navAtt1 = navAggregate.AggregateAttInput()
    navAtt2 = navAggregate.AggregateAttInput()
    navTrans1 = navAggregate.AggregateTransInput()
    navTrans2 = navAggregate.AggregateTransInput()

    module.setAttMsgCount(numAttNav)
    if numAttNav == 3:       # here the index asks to read from an empty (zero) message
        module.setAttMsgCount(2)

    module.setTransMsgCount(numTransNav)
    if numTransNav == 3:     # here the index asks to read from an empty (zero) message
        module.setTransMsgCount(2)

    if numAttNav <= navAggregate.MAX_AGG_NAV_MSG:
        module.attMsgs = [navAtt1, navAtt2]
        module.attMsgs[0].navAttInMsg.subscribeTo(navAtt1InMsg)
        module.attMsgs[1].navAttInMsg.subscribeTo(navAtt2InMsg)
    else:
        module.attMsgs = [navAtt1] * navAggregate.MAX_AGG_NAV_MSG
        for i in range(navAggregate.MAX_AGG_NAV_MSG):
            module.attMsgs[i].navAttInMsg.subscribeTo(navAtt1InMsg)
    if numTransNav <= navAggregate.MAX_AGG_NAV_MSG:
        module.transMsgs = [navTrans1, navTrans2]
        module.transMsgs[0].navTransInMsg.subscribeTo(navTrans1InMsg)
        module.transMsgs[1].navTransInMsg.subscribeTo(navTrans2InMsg)
    else:
        module.transMsgs = [navTrans1] * navAggregate.MAX_AGG_NAV_MSG
        for i in range(navAggregate.MAX_AGG_NAV_MSG):
            module.transMsgs[i].navTransInMsg.subscribeTo(navTrans1InMsg)

    if numAttNav > 1:       # always read from the last message counter
        module.setAttTimeIdx(numAttNav - 1)
        module.setAttIdx(numAttNav - 1)
        module.setRateIdx(numAttNav - 1)
        module.setSunIdx(numAttNav - 1)
    if numTransNav > 1:     # always read from the last message counter
        module.setTransTimeIdx(numTransNav - 1)
        module.setPosIdx(numTransNav - 1)
        module.setVelIdx(numTransNav - 1)
        module.setDvIdx(numTransNav - 1)

    # Setup logging on the test module output message so that we get all the writes to it
    dataAttLog = module.navAttOutMsg.recorder()
    dataTransLog = module.navTransOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataAttLog)
    unitTestSim.AddModelToTask(unitTaskName, dataTransLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    attTimeTag = np.transpose([dataAttLog.timeTag])
    attSigma = dataAttLog.sigma_BN
    attOmega = dataAttLog.omega_BN_B
    attSunVector = dataAttLog.vehSunPntBdy

    transTimeTag = np.transpose([dataTransLog.timeTag])
    transPos = dataTransLog.r_BN_N
    transVel = dataTransLog.v_BN_N
    transAccum = dataTransLog.vehAccumDV

    # set the filtered output truth states
    if numAttNav == 0 or numAttNav == 3:
        trueAttTimeTag = [[0.0]]*3
        trueAttSigma = [[0., 0., 0.]]*3
        trueAttOmega = [[0., 0., 0.]]*3
        trueAttSunVector = [[0., 0., 0.]]*3

    if numTransNav == 0 or numTransNav == 3:
        trueTransTimeTag = [[0.0]]*3
        trueTransPos = [[0.0, 0.0, 0.0]]*3
        trueTransVel = [[0.0, 0.0, 0.0]]*3
        trueTransAccum = [[0.0, 0.0, 0.0]]*3

    if numAttNav == 1 or numAttNav == 11:
        trueAttTimeTag = [[navAtt1Msg.timeTag]]*3
        trueAttSigma = [navAtt1Msg.sigma_BN]*3
        trueAttOmega = [navAtt1Msg.omega_BN_B]*3
        trueAttSunVector = [navAtt1Msg.vehSunPntBdy]*3

    if numTransNav == 1 or numTransNav == 11:
        trueTransTimeTag = [[navTrans1Msg.timeTag]]*3
        trueTransPos = [navTrans1Msg.r_BN_N]*3
        trueTransVel = [navTrans1Msg.v_BN_N]*3
        trueTransAccum = [navTrans1Msg.vehAccumDV]*3

    if numAttNav == 2:
        trueAttTimeTag = [[navAtt2Msg.timeTag]] * 3
        trueAttSigma = [navAtt2Msg.sigma_BN] * 3
        trueAttOmega = [navAtt2Msg.omega_BN_B] * 3
        trueAttSunVector = [navAtt2Msg.vehSunPntBdy] * 3

    if numTransNav == 2:
        trueTransTimeTag = [[navTrans2Msg.timeTag]]*3
        trueTransPos = [navTrans2Msg.r_BN_N]*3
        trueTransVel = [navTrans2Msg.v_BN_N]*3
        trueTransAccum = [navTrans2Msg.vehAccumDV]*3

    # compare the module results to the truth values
    accuracy = 1e-12

    np.testing.assert_allclose(attTimeTag, trueAttTimeTag, atol=accuracy, rtol=0, err_msg="attTimeTag")
    np.testing.assert_allclose(attSigma, trueAttSigma, atol=accuracy, rtol=0, err_msg="attSigma")
    np.testing.assert_allclose(attOmega, trueAttOmega, atol=accuracy, rtol=0, err_msg="attOmega")
    np.testing.assert_allclose(attSunVector, trueAttSunVector, atol=accuracy, rtol=0, err_msg="attSunVector")
    np.testing.assert_allclose(transTimeTag, trueTransTimeTag, atol=accuracy, rtol=0, err_msg="transTimeTag")
    np.testing.assert_allclose(transPos, trueTransPos, atol=accuracy, rtol=0, err_msg="transPos")
    np.testing.assert_allclose(transVel, trueTransVel, atol=accuracy, rtol=0, err_msg="transVel")
    np.testing.assert_allclose(transAccum, trueTransAccum, atol=accuracy, rtol=0, err_msg="transAccum")


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_navAggregate(False, 2, 2)
