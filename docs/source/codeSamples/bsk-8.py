from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of enabling and disabling tasks
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("cTask", macros.sec2nano(1.)))
    dynProcess.addTask(scSim.CreateNewTask("cppTask", macros.sec2nano(1.)))

    # create modules
    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "cppModule2"
    scSim.AddModelToTask("cppTask", mod2)

    #  initialize Simulation:
    scSim.InitializeSimulation()

    # execute BSK for a single step
    scSim.TotalSim.singleStepProcesses()

    dynProcess.disableTasks()
    print("all tasks disabled")
    scSim.TotalSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.enableTask("cppTask")
    scSim.TotalSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    scSim.disableTask("cppTask")
    scSim.TotalSim.singleStepProcesses()
    print("BSK executed a single simulation step")

    return


if __name__ == "__main__":
    run()
