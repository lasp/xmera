from xmera.architecture import messaging
from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of re-directing module output message to stand-alone messages
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask", macros.sec2nano(1.)))

    # create modules
    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "cppModule2"
    scSim.AddModelToTask("dynamicsTask", mod2)

    # create stand-along message with a C++ interface and re-direct
    # the C++ module output message writing to this stand-alone message
    cppMsg = messaging.ModuleTemplateMsg()
    mod2.dataOutMsg = cppMsg

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(1.0))
    scSim.ExecuteSimulation()

    # read the message values and print them to the terminal
    print("mod2.dataOutMsg:")
    print(mod2.dataOutMsg.read().dataVector)
    print("cppMsg:")
    print(cppMsg.read().dataVector)

    return


if __name__ == "__main__":
    run()
