from xmera.architecture import messaging
from xmera.simulation import fuelTank
from xmera.simulation import gravityEffector
from xmera.simulation import spacecraft
from xmera.simulation import thrusterDynamicEffector
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros
from xmera.utilities import simIncludeThruster
from xmera.utilities import unitTestSupport  # general support file with common unit test functions
import numpy


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_thrusterIntegratedTest(show_plots):
    [testResults, testMessage] = thrusterIntegratedTest(show_plots)
    assert testResults < 1, testMessage

def thrusterIntegratedTest(show_plots):
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

    # add thruster devices
    thFactory = simIncludeThruster.thrusterFactory()
    TH1 = thFactory.create(
        'MOOG_Monarc_1',
        [1,0,0],                # location in B-frame
        [0,1,0],                 # thruster force direction in B-frame
        thrBlowDownCoeff=[0, 0, 0.9],  # polynomial coefficients for fuel mass to thrust blow down model in descending order
        ispBlowDownCoeff=[0, 0, 227.5]  # polynomial coefficients for fuel mass to Isp blow down model in descending order
    )

    # create thruster object container and tie to spacecraft object
    thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory.addToSpacecraft("Thrusters",
                              thrustersDynamicEffector,
                              scObject)

    # create tank object container
    unitTestSim.fuelTankStateEffector = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    unitTestSim.fuelTankStateEffector.setTankModel(tankModel)
    tankModel.propMassInit = 40.0
    tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
    unitTestSim.fuelTankStateEffector.r_TB_B = [[0.0],[0.0],[0.0]]
    tankModel.radiusTankInit = 46.0 / 2.0 / 3.2808399 / 12.0

    unitTestSim.fuelTankStateEffector.addThrusterSet(thrustersDynamicEffector)
    scObject.addStateEffector(unitTestSim.fuelTankStateEffector)
    fuelLog = unitTestSim.fuelTankStateEffector.fuelTankOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, fuelLog)

    # set thruster commands
    ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
    ThrustMessage.OnTimeRequest = [9.9]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
    thrustersDynamicEffector.cmdsInMsg.subscribeTo(thrCmdMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, unitTestSim.fuelTankStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, thrustersDynamicEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
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

    # Get access to dynManager's translational state
    posRef = scObject.dynManager.getStateObject("hubPosition")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")


    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [dataPos[0][0], dataPos[1][0], dataPos[2][0]]
    dataSigma = [dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]

    truePos = [-6.7815933935338277e+06, 4.9468685979815889e+06, 5.4867416696776701e+06]

    trueSigma = [1.4401781243854264e-01, -6.4168702021364002e-02, 3.0166086824900967e-01]

    accuracy = 1e-8
    numpy.testing.assert_allclose(dataPos, truePos, rtol=accuracy, verbose=True)

    accuracy = 1e-7
    numpy.testing.assert_allclose(dataSigma, trueSigma, rtol=accuracy, verbose=True)

    accuracy = 1e-9
    numpy.testing.assert_allclose(thrustersDynamicEffector.fuelMass, fuelLog.fuelMass[-1], rtol=accuracy, verbose=True)

    if testFailCount == 0:
        print("PASSED: " + " Thruster Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_thrusterIntegratedTest(False)
