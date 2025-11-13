from xmera.moduleTemplates import cppModuleTemplate
from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Controlling the simulation time
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")
    fswProcess = scSim.CreateNewProcess("fswProcess", 10)

    # create the dynamics task and specify the integration update time
    fswProcess.addTask(scSim.CreateNewTask("fswTask1", macros.sec2nano(1.)))
    fswProcess.addTask(scSim.CreateNewTask("fswTask2", macros.sec2nano(2.)))
    fswProcess.addTask(scSim.CreateNewTask("fswTask3", macros.sec2nano(3.)), 10)
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask1", macros.sec2nano(1.)))
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask2", macros.sec2nano(5.)), 10)
    dynProcess.addTask(scSim.CreateNewTask("dynamicsTask3", macros.sec2nano(10.)))

    # create modules
    mod1 = cppModuleTemplate.CppModuleTemplate()
    mod1.modelTag = "module1"

    mod2 = cppModuleTemplate.CppModuleTemplate()
    mod2.modelTag = "module2"

    # add modules to various task lists
    scSim.AddModelToTask("dynamicsTask1", mod1, 4)
    scSim.AddModelToTask("dynamicsTask1", mod2, 5)
    scSim.AddModelToTask("dynamicsTask2", mod2)
    scSim.AddModelToTask("dynamicsTask2", mod1)
    scSim.AddModelToTask("dynamicsTask3", mod1)
    scSim.AddModelToTask("dynamicsTask3", mod2)

    scSim.AddModelToTask("fswTask1", mod1)
    scSim.AddModelToTask("fswTask1", mod2, 2)
    scSim.AddModelToTask("fswTask2", mod2)
    scSim.AddModelToTask("fswTask2", mod1)
    scSim.AddModelToTask("fswTask3", mod1)
    scSim.AddModelToTask("fswTask3", mod2)

    # print to the terminal window the execution order of the processes, task lists and modules
    scSim.ShowExecutionOrder()

    # uncomment this code to show the execution order figure and save it off
    # fig = scSim.ShowExecutionFigure(False)
    # fig.savefig("qs-bsk-2b-order.svg", transparent=True, bbox_inches = 'tight', pad_inches = 0)

    return


if __name__ == "__main__":
    run()
