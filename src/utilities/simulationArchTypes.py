# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.architecture import sim_model


class ProcessBaseClass(object):
    """Class for a BSK process"""
    def __init__(self, procName, procPriority=-1):
        self.Name = procName
        self.processData = sim_model.SysProcess(procName)
        self.processData.processPriority = procPriority

    def addTask(self, newTask, taskPriority=-1):
        self.processData.addTask(newTask.TaskData, taskPriority)

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
    def __init__(self, TaskName, TaskRate, FirstStart=0):
        self.Name = TaskName
        self.TaskData = sim_model.SysModelTask(TaskRate, FirstStart)
        self.TaskData.TaskName = TaskName
        self.TaskModels = []

    def disable(self):
        self.TaskData.disable()

    def enable(self):
        self.TaskData.enable()

    def reset(self, callTime):
        self.TaskData.reset(callTime)
