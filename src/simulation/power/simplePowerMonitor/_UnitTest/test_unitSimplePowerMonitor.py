import inspect
import os

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'xmera'
splitPath = path.split(bskName)

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.utilities import unitTestSupport                  # general support file with common unit test functions
from xmera.architecture import messaging
from xmera.simulation import simplePowerMonitor
from xmera.utilities import macros

# update "module" in this function name to reflect the module name
def test_module(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = storage_limits(show_plots)
    assert testResults < 1, testMessage


def storage_limits(show_plots):
    """
    **Validation Test Description**

    1. Whether the simpleBattery can add multiple nodes (core base class functionality);
    2. That the battery correctly evaluates how much stored power it should have given a pair of five-watt input messages.

    :param show_plots: Not used; no plots to be shown.

    :return:
    """

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    test_battery = simplePowerMonitor.SimplePowerMonitor()
    test_battery.storedCharge_Init = 0

    powerMsg1 = messaging.PowerNodeUsageMsgPayload()
    powerMsg1.netPower = 5.0
    pw1Msg = messaging.PowerNodeUsageMsg().write(powerMsg1)
    powerMsg2 = messaging.PowerNodeUsageMsgPayload()
    powerMsg2.netPower = 5.0
    pw2Msg = messaging.PowerNodeUsageMsg().write(powerMsg2)


    # Test the addNodeToStorage method:
    test_battery.addPowerNodeToModel(pw1Msg)
    test_battery.addPowerNodeToModel(pw2Msg)
    unitTestSim.AddModelToTask(unitTaskName, test_battery)

    dataLog = test_battery.batPowerOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))

    unitTestSim.ExecuteSimulation()

    storedChargeLog = dataLog.storageLevel
    capacityLog = dataLog.storageCapacity
    netPowerLog = dataLog.currentNetPower

    #   Check 1 - is net power equal to 10.?
    for ind in range(0,len(netPowerLog)):
        currentPower = netPowerLog[ind]
        if currentPower < 10.:
            testFailCount +=1
            testMessages.append("FAILED: SimplePowerMonitor did not correctly log the net power.")

    #   Check 2 - is the stored power equivalent to 10*5 W-s?

    if not unitTestSupport.isDoubleEqualRelative((10.),storedChargeLog[-1], 1e-8):
        testFailCount+=1
        testMessages.append("FAILED: SimplePowerMonitor did not track integrated power. Returned "+str(storedChargeLog[-1,1])+", expected "+str((10.)))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    print(test_module(False))
