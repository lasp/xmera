r"""

This script is a basic demonstration of a script that can be used to rerun a set or subset of Monte Carlo simulations.

.. important::
   This script can only be run once there exists data produced by the ``scenario_AttFeedbackMC.py`` script.


"""

import inspect
import os
import sys

from xmera.utilities.MonteCarlo.Controller import Controller
from xmera.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy

filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))

from xmera import __path__
bskPath = __path__[0]

sys.path.append(path+"/../BskSim/scenarios/")

def run(time=None):
    """
    Instructions:

    1) Change the scenario name

    2) Provide the number of processes to spawn

    3) Provide the run numbers you wish to rerun

    4) Add any new retention policies to the bottom

    """

    # Step 1-3: Change to the relevant scenario
    scenarioName = "scenario_AttFeedback"

    monteCarlo = Controller()
    monteCarlo.numProcess = 3 # Specify number of processes to spawn
    runsList = [1]  # Specify the run numbers to be rerun

    #
    # # Generic initialization
    icName = path + "/" + scenarioName + "MC/"
    newDataDir = path + "/" + scenarioName + "MC/rerun"


    exec('import '+ scenarioName)
    simulationModule = eval(scenarioName + "." + scenarioName) # ex. scenarioMonteCarlo.scenarioMonteCarlo
    if time is not None:
        exec (scenarioName + '.' + scenarioName + '.simBaseTime = time')  # ex. scenarioMonteCarlo.scenarioMonteCarlo.simBaseTime = time
    executionModule = eval(scenarioName + ".runScenario") # ex. scenarioMonteCarlo.run

    monteCarlo.setSimulationFunction(simulationModule)
    monteCarlo.setExecutionFunction(executionModule)
    monteCarlo.setICDir(icName)
    monteCarlo.setICRunFlag(True)
    monteCarlo.setArchiveDir(newDataDir)
    monteCarlo.setExecutionCount(len(runsList))
    monteCarlo.setShouldDisperseSeeds(False)
    monteCarlo.shouldArchiveParameters = False


    # Step 4: Add any additional retention policies desired
    retentionPolicy = RetentionPolicy()
    retentionPolicy.logRate = int(2E9)
    retentionPolicy.addMessageLog("attGuidMsg", ["sigma_BR"])
    monteCarlo.addRetentionPolicy(retentionPolicy)


    failed = monteCarlo.runInitialConditions(runsList)
    assert len(failed) == 0, "Should run ICs successfully"



if __name__ == "__main__":
    run()
