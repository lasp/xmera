# SPDX-License-Identifier: ISC
# Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
import inspect

from collections import OrderedDict

import matplotlib.pyplot as plt
import numpy as np
from xmera.architecture import sim_model
from xmera.utilities import deprecated, simulationArchTypes
from xmera.utilities.pythonVariableLogger import PythonVariableLogger
from xmera.utilities.simulationProgessBar import SimulationProgressBar

class EventHandlerClass:
    """Event Handler Class"""
    def __init__(self, eventName, eventRate=int(1E9), eventActive=False,
                 conditionList=[], actionList=[], terminal=False, callerGlobals=None):
        self.eventName = eventName
        self.eventActive = eventActive
        self.eventRate = eventRate
        self.conditionList = conditionList
        self.actionList = actionList
        self.occurCounter = 0
        self.prevTime = -1
        self.checkCall = None
        self.operateCall = None
        self.terminal = terminal
        self.callerGlobals = callerGlobals or {}

    def methodizeEvent(self):
        if self.checkCall is not None:
            return

        cond_expr = ' and '.join(f'({c})' for c in self.conditionList)
        check_code = compile(cond_expr, f'<event:{self.eventName}:check>', 'eval')

        operate_src = '\n'.join(self.actionList)
        operate_code = compile(operate_src, f'<event:{self.eventName}:operate>', 'exec')

        # Build a shared namespace from the caller's globals so condition/action
        # strings can reference any name that was in scope at registration time
        # (e.g. 'np', 'math').  'self' is updated in-place before each call to
        # avoid allocating a new dict on every simulation tick.
        ns = dict(self.callerGlobals)
        ns['self'] = None

        def check_call(sim):
            ns['self'] = sim
            return 1 if eval(check_code, ns) else 0

        def operate_call(sim):
            ns['self'] = sim
            exec(operate_code, ns)

        self.checkCall = check_call
        self.operateCall = operate_call

    def checkEvent(self, parentSim):
        nextTime = int(-1)
        if self.eventActive == False:
            return(nextTime)
        nextTime = self.prevTime + self.eventRate - (self.prevTime%self.eventRate)
        if self.prevTime < 0 or (parentSim.getCurrentNanos()%self.eventRate == 0):
            nextTime = parentSim.getCurrentNanos() + self.eventRate
            eventCount = self.checkCall(parentSim)
            self.prevTime = parentSim.getCurrentNanos()
            if eventCount > 0:
                self.eventActive = False
                self.operateCall(parentSim)
                self.occurCounter += 1
                if self.terminal:
                    parentSim.terminate = True
        return(nextTime)

class SimBaseClass:
    """Simulation Base Class"""
    def __init__(self):
        self.TotalSim = None
        self.TaskList = []
        self.procList = []
        self.StopTime = 0
        self.nextEventTime = 0
        self.lastUpdateTime = 0
        self.terminate = False
        self.oldSyntaxVariableLog = {}
        self.multiProcessVariableLoggers = {}
        self.allModels = []
        self.eventMap = {}
        self.simulationInitialized = False
        self.simulationFinished = False
        self.bskLogger = sim_model.BSKLogger()
        self.showProgressBar = False

    def SetProgressBar(self, value):
        """
        Shows a dynamic progress in the terminal while the simulation is executing.
        """
        self.showProgressBar = value

    def ShowExecutionOrder(self):
        """
        Shows in what order the Xmera processes, task lists and modules are executed
        """
        processLine = (
            "\u001b[32mProcess Name:\u001b[0m {name} , "
            "\u001b[32mpriority:\u001b[0m {priority}" )
        taskLine = (
            "\u001b[33mTask Name:\u001b[0m  {name}, "
            "\u001b[33mpriority:\u001b[0m {priority}, "
            "\u001b[33mTaskPeriod:\u001b[0m {period}s" )
        moduleLine = (
            "\u001b[36mModuleTag:\u001b[0m {name}, "
            "\u001b[36mpriority:\u001b[0m {priority}" )

        for process in sorted(self.procList, key = lambda p: p.priority, reverse=True):
            print(processLine.format(name = process.Name, priority = process.priority))
            for task in sorted(process.tasks, key = lambda t: t.priority, reverse=True):
                print(taskLine.format(name = task.Name, priority = task.priority, period = task.period/1.0e9))
                for module in sorted(task.models, key = lambda m: m.CurrentModelPriority, reverse=True):
                    print(moduleLine.format(name = module.ModelPtr.modelTag, priority = module.CurrentModelPriority))
            print("")

    def ShowExecutionFigure(self, show_plots=False):
        """
        Shows in what order the Xmera processes, task lists and modules are executed
        """
        processList = OrderedDict()
        for process in sorted(self.procList, key = lambda p: p.priority, reverse=True):
            taskList = OrderedDict()
            for task in sorted(process.tasks, key = lambda t: t.priority, reverse=True):
                moduleList = []
                for module in task.models:
                    moduleList.append(module.ModelPtr.modelTag + " (" + str(module.CurrentModelPriority) + ")")
                taskList[task.Name + " (" + str(task.priority) + ", " + str(task.period/1.0e9) + "s)"] = moduleList
            processList[process.Name + " (" + str(process.priority) + ")"] = taskList

        fig = plt.figure()
        plt.rcParams.update({'font.size': 8})
        plt.axis('off')

        processNo = 0
        processWidth = 6
        lineHeight = 0.5
        textBuffer = lineHeight*0.75
        textIndent = lineHeight*0.25
        processGap = 0.5
        for process in processList:
            # Draw process box + priority
            rectangle = plt.Rectangle(((processWidth+processGap)*processNo, 0), processWidth, -lineHeight, ec='g', fc='g')
            plt.gca().add_patch(rectangle)
            plt.text((processWidth+processGap)*processNo + textIndent, -textBuffer, process, color='w')

            taskNo = 0
            currentLine = -lineHeight - textIndent
            for task in processList[process]:
                # Draw task box + priority + task rate
                rectangle = plt.Rectangle(((processWidth + processGap) * processNo + textIndent, currentLine)
                                          , processWidth - 2 * textIndent
                                          , - (1+len(processList[process][task])) * (lineHeight + textIndent),
                                          ec='y', fc=(1,1,1,0))
                plt.gca().add_patch(rectangle)
                rectangle = plt.Rectangle(((processWidth + processGap) * processNo + textIndent, currentLine)
                                          , processWidth - 2 * textIndent, -lineHeight,
                                          ec='y', fc='y')
                plt.gca().add_patch(rectangle)
                plt.text((processWidth + processGap) * processNo + 2*textIndent,
                         currentLine-textBuffer, task, color='black')

                for module in processList[process][task]:
                    # Draw modules + priority
                    currentLine -= lineHeight + textIndent
                    rectangle = plt.Rectangle(((processWidth + processGap) * processNo + 2*textIndent, currentLine)
                                              , processWidth - 4 * textIndent, -lineHeight,
                                              ec='c', fc=(1,1,1,0))
                    plt.gca().add_patch(rectangle)
                    plt.text((processWidth + processGap) * processNo + 3*textIndent,
                             currentLine-textBuffer, module, color='black')

                taskNo += 1
                currentLine -=  lineHeight + 2 * textIndent

            rectangle = plt.Rectangle(((processWidth+processGap)*processNo, 0), processWidth, currentLine, ec='g', fc=(1,1,1,0))
            plt.gca().add_patch(rectangle)
            processNo += 1

        plt.axis('scaled')

        if show_plots:
            plt.show()

        return fig

    def AddModelToTask(self, TaskName, NewModel, ModelData=None, ModelPriority=-1):
        """
        This function is responsible for passing on the logger to a module instance (model), adding the
        model to a particular task, and defining
        the order/priority that the model gets updated within the task.

        :param TaskName (str): Name of the task
        :param NewModel (obj): Model to add to the task
        :param ModelData: None or struct containing, only used for C BSK modules
        :param ModelPriority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return:
        """
        # Supports calling AddModelToTask(TaskName, NewModel, ModelPriority)
        if isinstance(ModelData, int):
            ModelPriority = ModelData
            ModelData = None

        for Task in self.TaskList:
            if Task.Name == TaskName:
                pair = sim_model.ModelPriorityPair()
                pair.CurrentModelPriority = ModelPriority
                pair.ModelPtr = NewModel

                Task.models.append(pair)
                self.allModels.append((NewModel, ModelData, Task) )

                if ModelData is not None:
                    try:
                        ModelData.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(ModelData)
                else:
                    try:
                        NewModel.bskLogger = self.bskLogger
                    except:
                        pass
                    Task.TaskModels.append(NewModel)
                return
        raise ValueError(f"Could not find a Task with name: {TaskName}")

    def CreateNewProcess(self, procName = "", priority = -1):
        """
        Creates a process and adds it to the sim

        :param procName (str): Name of process
        :param priority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return: simulationArchTypes.ProcessBaseClass object
        """
        return simulationArchTypes.ProcessBaseClass(self, procName, priority)

    # When this method is removed, remember to delete the 'oldSyntaxVariableLog' and
    # 'allModels' attributes (as well as any mention of them) as they are no longer needed
    @deprecated.deprecated("2024/09/06",
        "Use the 'logger' function or 'PythonVariableLogger' instead of 'AddVariableForLogging'."
        " See 'https://github.com/lasp/xmera/Learn/xmeraPrinciples/xmeraPrinciples-6.html'"
    )
    def AddVariableForLogging(self, VarName: str, LogPeriod: int = 0, *_, **__):
        """Generates a logger and adds it to the same task as the module
        in `VarName`.

        Args:
            VarName (str): The variable to log in the format "<modelTag>.<variable_name>"
            LogPeriod (int, optional): The minimum time between logs. Defaults to 0.
        """
        if "." not in VarName:
            raise ValueError('The variable to log must be given in the format '
                             '"<modelTag>.<variable_name>"')

        modelTag = VarName.split('.')[0]

        # Calling eval on a pre-compiled string is faster than
        # eval-ing the string (by a large factor)
        compiledExpr = compile(VarName, "<logged-variable>", "eval")

        # Find the model object that corresponds to the given tag, as well as the
        # task where this model was added
        modelOrConfig = task = None
        for model, modelData, task in self.allModels:
            if model.modelTag == modelTag:
                modelOrConfig = modelData or model
                break

        if task is None or modelOrConfig is None:
            raise ValueError(f"Could not find model with tag {modelTag}")

        # The callback logging function 'fun' simply evaluates the given
        # expression. We pass a dictionary '{modelTag: modelOrConfig}'
        # that allows the expression to substitute the modelTag by the
        # actual model object
        def fun(_):
            val = eval(compiledExpr, globals(), {modelTag: modelOrConfig})
            val = np.array(val).squeeze()
            return val

        logger = PythonVariableLogger({"variable": fun}, LogPeriod)
        logger.modelTag = f"Logger:{VarName}"
        self.AddModelToTask(task.Name, logger)

        self.oldSyntaxVariableLog[VarName] = logger

    def AddVariableForMultiProcessLogging(self, VarName: str, LogPeriod: int = 0, *_, **__):
        """
        This function should only be used when parallelizing Simulations. Use PythonVariableLogger instead for
        single process simulations.
        Generates a logger and adds it to the same task as the module
        in `VarName`.
        Args:
            VarName (str): The variable to log in the format "<modelTag>.<variable_name>"
            LogPeriod (int, optional): The minimum time between logs. Defaults to 0.
        """
        if "." not in VarName:
            raise ValueError('The variable to log must be given in the format '
                             '"<modelTag>.<variable_name>"')

        modelTag = VarName.split('.')[0]
        # Calling eval on a pre-compiled string is faster than
        # eval-ing the string (by a large factor)
        compiledExpr = compile(VarName, "<logged-variable>", "eval")

        # Find the model object that corresponds to the given tag, as well as the
        # task where this model was added
        modelOrConfig = task = None
        for model, modelData, task in self.allModels:
            if model.modelTag == modelTag:
                modelOrConfig = modelData or model
                break
        if task is None or modelOrConfig is None:
            raise ValueError(f"Could not find model with tag {modelTag}")

        # The callback logging function 'fun' simply evaluates the given
        # expression. We pass a dictionary '{modelTag: modelOrConfig}'
        # that allows the expression to substitute the modelTag by the
        # actual model object
        def fun(_):
            val = eval(compiledExpr, globals(), {modelTag: modelOrConfig})
            val = np.array(val).squeeze()
            return val

        logger = PythonVariableLogger({"variable": fun}, LogPeriod)
        logger.modelTag = f"Logger:{VarName}"
        self.AddModelToTask(task.Name, logger)

        self.multiProcessVariableLoggers[VarName] = logger

    def InitializeSimulation(self):
        """
        Initialize the BSK simulation.  This runs the reset() method on each module.
        """
        self.TotalSim = sim_model.simulation()
        for process in self.procList:
            process.processData = self.TotalSim.add_task_group(process.priority)

            for task in process.tasks:
                step_list = sim_model.task_step_list()
                for model in sorted(task.models, key = lambda m: m.CurrentModelPriority, reverse=True):
                    step_list.push_back(model.ModelPtr)

                description = sim_model.task_description(step_list)
                description.first_update_nanos = task.start
                description.update_period_nanos = task.period
                description.group = process.processData
                description.priority = process.priority

                task.TaskData = self.TotalSim.add_task(description)

                if not task.enabled:
                    self.TotalSim.disable(task.TaskData)

        self.TotalSim.reset()
        self.lastUpdateTime = 0
        self.simulationInitialized = True

    def ConfigureStopTime(self, TimeStop):
        """
        Set the simulation stop time in nano-seconds.
        """
        self.StopTime = TimeStop

    def getCurrentNanos(self):
        return self.lastUpdateTime

    def ExecuteSimulation(self):
        """
        run the simulation until the prescribed stop time or termination.
        """
        self.initializeEventChecks()

        nextStopTime = self.TotalSim.next_update()
        progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        while self.TotalSim.next_update() <= self.StopTime and not self.terminate:
            if 0 <= self.nextEventTime <= self.lastUpdateTime:
                self.nextEventTime = self.checkEvents()
                self.nextEventTime = self.nextEventTime if self.nextEventTime >= self.TotalSim.next_update() else self.TotalSim.next_update()
            if 0 <= self.nextEventTime < nextStopTime:
                nextStopTime = self.nextEventTime

            if self.terminate:
                break

            self.lastUpdateTime = self.TotalSim.next_update()
            sim_model.step_until(self.TotalSim, nextStopTime)
            progressBar.update(self.lastUpdateTime)

            nextStopTime = self.StopTime
            nextStopTime = nextStopTime if nextStopTime >= self.TotalSim.next_update() else self.TotalSim.next_update()
        self.terminate = False
        progressBar.markComplete()
        progressBar.close()

    def singleStepProcesses(self):
        sim_model.step_next_update(self.TotalSim)

    # @deprecated.deprecated("2024/09/06",
    #     "Deprecated way to access logged variables."
    #     " See 'https://github.com/lasp/xmera/Learn/xmeraPrinciples/xmeraPrinciples-6.html'"
    # )
    def GetLogVariableData(self, LogName):
        """
        Pull the recorded module recorded variable.  The first column is the variable recording time in
        nano-seconds, the additional column(s) are the message data columns.
        """
        if LogName not in self.oldSyntaxVariableLog:
            raise ValueError(f'"{LogName}" is not being logged. Check the spelling.')

        logger = self.oldSyntaxVariableLog[LogName]
        return np.column_stack([logger.times(), logger.variable])

    def GetMultiProcessLoggerVariableData(self, LogName):
        """
        # This function should only be used when parallelizing Simulations. Use PythonVariableLogger instead for
        single process simulations.
        Pull the recorded module recorded variable.  The first column is the variable recording time in
        nano-seconds, the additional column(s) are the message data columns.
        """
        if LogName not in self.multiProcessVariableLoggers:
            raise ValueError(f'"{LogName}" is not being logged. Check the spelling.')

        logger = self.multiProcessVariableLoggers[LogName]
        return logger.GetData("variable")

    def disableTask(self, TaskName):
        """
        Disable this particular task from being executed.
        """
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.disable()

    def enableTask(self, TaskName):
        """
        Enable this particular task to be executed.
        """
        for Task in self.TaskList:
            if Task.Name == TaskName:
                Task.enable()

    def createNewEvent(self, eventName, eventRate=int(1E9), eventActive=False,
                       conditionList=[], actionList=[], terminal=False):
        """
        Create an event sequence that contains a series of tasks to be executed.
        """
        if (eventName in list(self.eventMap.keys())):
            return
        callerGlobals = inspect.currentframe().f_back.f_globals
        newEvent = EventHandlerClass(eventName, eventRate, eventActive,
                                     conditionList, actionList, terminal, callerGlobals)
        self.eventMap.update({eventName: newEvent})

    def initializeEventChecks(self):
        self.eventList = []
        for key, value in self.eventMap.items():
            value.methodizeEvent()
            self.eventList.append(value)
        self.nextEventTime = 0

    def checkEvents(self):
        nextTime = -1
        for localEvent in self.eventList:
            localNextTime = localEvent.checkEvent(self)
            if(localNextTime >= 0 and (localNextTime < nextTime or nextTime <0)):
                nextTime = localNextTime
        return nextTime

    def setEventActivity(self, eventName, activityCommand):
        if eventName not in list(self.eventMap.keys()):
            print("You asked me to set the status of an event that I don't have.")
            return
        self.eventMap[eventName].eventActive = activityCommand

    def setAllButCurrentEventActivity(self, currentEventName, activityCommand, useIndex=False):
        """Set all event activity variables except for the currentEventName event. The ``useIndex`` flag can be used to
        prevent enabling or disabling every task, and instead only alter the ones that belong to the same group (for
        example, the same spacecraft). The distinction is made through an index set after the ``_`` symbol in the event
        name. All events of the same group must have the same index."""

        if useIndex:
            index = currentEventName.partition('_')[2]  # save the current event's index

        for eventName in list(self.eventMap.keys()):
            if currentEventName != eventName:
                if useIndex:
                    if eventName.partition('_')[2] == index:
                        self.eventMap[eventName].eventActive = activityCommand
                else:
                    self.eventMap[eventName].eventActive = activityCommand
