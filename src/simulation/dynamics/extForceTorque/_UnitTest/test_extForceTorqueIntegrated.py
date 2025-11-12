import pytest
from xmera.simulation import extForceTorque
from xmera.simulation import gravityEffector
from xmera.simulation import spacecraft
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import unitTestSupport  # general support file with common unit test functions


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["extForceBodyAndTorque"
                                      , "extForceInertialAndTorque"
                                      ])
def test_ForceBodyAndTorqueAllTest(show_plots, function):
    """Module Unit Test"""
    [testResults, testMessage] = eval(function + '()')
    assert testResults < 1, testMessage


def extForceBodyAndTorque():
    """Module Unit Test"""
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    # Define initial conditions
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    unitTestSim.InitializeSimulation()

    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.modelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[-1], [1], [-1]]
    extFTObject.extForce_B = [[1], [2], [3]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    posRef = scObject.dynManager.getStateObject("hubPosition")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-6.78136423e+06, 4.94628599e+06, 5.48655395e+06]
                ]

    trueSigma = [
                [4.91025978e-01, -4.21586707e-01,  3.61459503e-01]
                ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Body Force and Torque failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Body Force and Torque failed attitude unit test")

    if testFailCount == 0:
        print("PASSED: " + " External Body Force and Torque Inegrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def extForceInertialAndTorque():
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.modelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15  # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    unitTestSim.InitializeSimulation()

    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.modelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[-1], [1], [-1]]
    extFTObject.extForce_N = [[-1], [-0.5], [0.5]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    posRef = scObject.dynManager.getStateObject("hubPosition")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-6.78183900e+06, 4.94674963e+06, 5.48686274e+06]
                ]

    trueSigma = [
                [4.91025978e-01, -4.21586707e-01,  3.61459503e-01]
                ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Inertial Force and Torque failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: External Inertial Force and Torque failed attitude unit test")

    if testFailCount == 0:
        print("PASSED: " + " External Inertial Force and Torque Inegrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    extForceBodyAndTorque()
