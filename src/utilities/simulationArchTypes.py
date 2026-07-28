# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.architecture import sim_model


class ProcessBaseClass(object):
    """Class for a BSK process"""
    def __init__(self, simulation, processData):
        self.simulation = simulation
        self.processData = processData
        self.Name = processData.processName

        self.simulation.procList.append(self)

    def addTask(self, name="", TaskRate=100, FirstStart=0, priority=-1):
        taskData = self.processData.addTask(TaskRate, FirstStart, priority)
        taskData.TaskName = name

        return TaskBaseClass(self.simulation, name, taskData)

    def addInterfaceRef(self, newInt):
        self.processData.addInterfaceRef(newInt)

    def discoverAllMessages(self):
        self.processData.discoverAllMessages()

    def disableTasks(self):
        self.processData.disableTasks()

    def enableTasks(self):
        self.processData.enableTasks()

    def selectProcess(self):
        pass

    def updateTaskPeriod(self, TaskName, newPeriod):
        self.processData.changeTaskPeriod(TaskName, newPeriod)


class TaskBaseClass(object):
    def __init__(self, simulation, TaskName, TaskData):
        self.simulation = simulation
        self.Name = TaskName
        self.TaskData = TaskData
        self.TaskModels = []

        self.simulation.TaskList.append(self)

    def addModel(self, model, priority=-1):
        self.TaskData.addModel(model, priority)
        self.TaskModels.append(model)

        model.bskLogger = self.simulation.bskLogger
        self.simulation.allModels.append((model, None, self))

    def disable(self):
        self.TaskData.disable()

    def enable(self):
        self.TaskData.enable()

    def reset(self, callTime):
        self.TaskData.reset(callTime)
