# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

from xmera.architecture import sim_model

class ModelPriorityPair(object):
    """A model paired with the priority that orders it among the other models in its task.

    Higher priorities are updated first; models of equal priority keep the order
    in which they were added.
    """
    def __init__(self, model=None, priority=-1):
        self.ModelPtr = model
        self.CurrentModelPriority = priority

class ProcessBaseClass(object):
    """Class for a BSK process"""
    def __init__(self, simulation, name, priority):
        self.simulation = simulation
        self.Name = name
        self.priority = priority
        self.tasks = []
        self.processData = None

        self.simulation.procList.append(self)

    def addTask(self, name="", TaskRate=100, FirstStart=0, priority=-1):
        task = TaskBaseClass(self.simulation, self, name, FirstStart, TaskRate, priority)
        self.tasks.append(task)
        return task

    def disableTasks(self):
        for task in self.tasks:
            task.disable()

    def enableTasks(self):
        for task in self.tasks:
            task.enable()

    def updateTaskPeriod(self, TaskName, newPeriod):
        for task in self.tasks:
            if task.Name == TaskName:
                self.simulation.TotalSim.set_update_period(task.TaskData, newPeriod)

class TaskBaseClass(object):
    def __init__(self, simulation, group, name, FirstStart, TaskRate, priority):
        self.simulation = simulation
        self.group = group
        self.Name = name
        self.start = FirstStart
        self.period = TaskRate
        self.priority = priority
        self.TaskData = None
        self.enabled = True

        self.TaskModels = []
        self.models = []

        self.simulation.TaskList.append(self)

    def addModel(self, model, priority=-1):
        self.models.append(ModelPriorityPair(model, priority))
        self.simulation.allModels.append((model, None, self))

        model.bskLogger = self.simulation.bskLogger
        self.TaskModels.append(model)

    def disable(self):
        self.enabled = False
        if self.TaskData:
            self.simulation.TotalSim.disable(self.TaskData)

    def enable(self):
        self.enabled = True
        if self.TaskData:
            self.simulation.TotalSim.enable(self.TaskData)
