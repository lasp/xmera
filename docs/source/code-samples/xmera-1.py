# SPDX-License-Identifier: ISC
# Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.utilities import SimulationBaseClass
from xmera.utilities import macros


def run():
    """
    Illustration of xmera process and task creation
    """

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess("dynamicsProcess")
    fswProcess = scSim.CreateNewProcess("fswProcess")

    # create the dynamics task and specify the integration update time
    dynProcess.addTask("dynamicsTask", macros.sec2nano(5.))
    dynProcess.addTask("sensorTask", macros.sec2nano(10.))
    fswProcess.addTask("fswTask", macros.sec2nano(10.))

    #  initialize Simulation:
    scSim.InitializeSimulation()

    #   configure a simulation stop time and execute the simulation run
    scSim.ConfigureStopTime(macros.sec2nano(20.0))
    scSim.ExecuteSimulation()

    return


if __name__ == "__main__":
    run()
