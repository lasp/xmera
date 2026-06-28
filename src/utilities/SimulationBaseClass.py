# SPDX-License-Identifier: ISC
# Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import array
import inspect

# Import some architectural stuff that we will probably always use
import os
import sys
import warnings
import xml.etree.ElementTree as ET
from collections import OrderedDict

import matplotlib.pyplot as plt
import numpy as np
from xmera.architecture import sim_model
from xmera.utilities import deprecated, simulationArchTypes
from xmera.utilities.pythonVariableLogger import PythonVariableLogger
from xmera.utilities.simulationProgessBar import SimulationProgressBar

# Point the path to the module storage area


# define ASCI color codes
processColor = '\u001b[32m'
taskColor = '\u001b[33m'
moduleColor = '\u001b[36m'
endColor = '\u001b[0m'

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
        if self.prevTime < 0 or (parentSim.TotalSim.getCurrentNanos()%self.eventRate == 0):
            nextTime = parentSim.TotalSim.getCurrentNanos() + self.eventRate
            eventCount = self.checkCall(parentSim)
            self.prevTime = parentSim.TotalSim.getCurrentNanos()
            if eventCount > 0:
                self.eventActive = False
                self.operateCall(parentSim)
                self.occurCounter += 1
                if self.terminal:
                    parentSim.terminate = True
        return(nextTime)


class StructDocData:
    """Structure data documentation class"""
    class StructElementDef:
        def __init__(self, type, name, argstring, desc=''):
            self.type = type
            self.name = name
            self.argstring = argstring
            self.desc = desc

    def __init__(self, strName):
        self.strName = strName
        self.structPopulated = False
        self.structElements = {}

    def clearItem(self):
        self.structPopulated = False
        self.structElements = {}

    def populateElem(self, xmlSearchPath):
        if self.structPopulated == True:
            return
        xmlFileUse = xmlSearchPath + '/' + self.strName + '.xml'
        try:
            xmlData = ET.parse(xmlFileUse)
        except:
            print("Failed to parse the XML structure for: " + self.strName)
            print("This file does not exist most likely: " + xmlFileUse)
            return
        root = xmlData.getroot()
        validElement = root.find("./compounddef[@id='" + self.strName + "']")
        for newVariable in validElement.findall(".//memberdef[@kind='variable']"):
            typeUse = newVariable.find('type').text if newVariable.find('type') is not None else \
                None
            nameUse = newVariable.find('name').text if newVariable.find('type') is not None else \
                None
            argstringUse = newVariable.find('argsstring').text if newVariable.find('argsstring') is not None else \
                None
            descUse = newVariable.find('./detaileddescription/para').text if newVariable.find(
                './detaileddescription/para') is not None else \
                None
            if descUse == None:
                descUse = newVariable.find('./briefdescription/para').text if newVariable.find(
                    './briefdescription/para') is not None else \
                    None
            newElement = StructDocData.StructElementDef(typeUse, nameUse, argstringUse, descUse)
            self.structElements.update({nameUse: newElement})
            self.structPopulated = True

    def printElem(self):
        print("    " + self.strName + " Structure Elements:")
        for key, value in self.structElements.items():
            outputString = ''
            outputString += value.type + " " + value.name
            outputString += value.argstring if value.argstring is not None else ''
            outputString += ': ' + value.desc if value.desc is not None else ''
        print("      " + outputString)

class DataPairClass:
    def __init__(self):
        self.outputMessages = set([])
        self.inputMessages = set([])
        self.name = ""
        self.outputDict = {}

class SimBaseClass:
    """Simulation Base Class"""
    def __init__(self):
        self.TotalSim = sim_model.SimModel()
        self.TaskList = []
        self.procList = []
        self.StopTime = 0
        self.nextEventTime = 0
        self.terminate = False
        self.oldSyntaxVariableLog = {}
        self.multiProcessVariableLoggers = {}
        self.allModels = []
        self.eventMap = {}
        self.simBasePath = os.path.dirname(os.path.realpath(__file__)) + '/../'
        self.dataStructIndex = self.simBasePath + '/xml/index.xml'
        self.indexParsed = False
        self.simulationInitialized = False
        self.simulationFinished = False
        self.bskLogger = sim_model.BSKLogger()
        self.showProgressBar = False
        self.allModules = set()

    def SetProgressBar(self, value):
        """
        Shows a dynamic progress in the terminal while the simulation is executing.
        """
        self.showProgressBar = value

    def ShowExecutionOrder(self):
        """
        Shows in what order the Xmera processes, task lists and modules are executed
        """

        for processData in self. TotalSim.processList:
            print(f"{processColor}Process Name: {endColor}" + processData.processName +
                  " , " + processColor + "priority: " + endColor + str(processData.processPriority))
            for task in processData.processTasks:
                print(f"{taskColor}Task Name: {endColor}" + task.TaskPtr.TaskName +
                      ", " + taskColor + "priority: " + endColor + str(task.taskPriority) +
                      ", " + taskColor + "TaskPeriod: " + endColor + str(task.TaskPtr.getTaskPeriod()/1.0e9) + "s")
                for module in task.TaskPtr.TaskModels:
                    print(moduleColor + "ModuleTag: " + endColor + module.ModelPtr.modelTag +
                          ", " + moduleColor + "priority: " + endColor + str(module.CurrentModelPriority))
            print("")


    def ShowExecutionFigure(self, show_plots=False):
        """
        Shows in what order the Xmera processes, task lists and modules are executed
        """
        processList = OrderedDict()
        for processData in self. TotalSim.processList:
            taskList = OrderedDict()
            for task in processData.processTasks:
                moduleList = []
                for module in task.TaskPtr.TaskModels:
                    moduleList.append(module.ModelPtr.modelTag + " (" + str(module.CurrentModelPriority) + ")")
                taskList[task.TaskPtr.TaskName + " (" + str(task.taskPriority) + ", " + str(task.TaskPtr.TaskPeriod/1.0e9) + "s)"] = moduleList
            processList[processData.processName + " (" + str(processData.processPriority) + ")"] = taskList

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
                Task.TaskData.addModel(NewModel, ModelPriority)
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

    def CreateNewProcess(self, procName, priority = -1):
        """
        Creates a process and adds it to the sim

        :param procName (str): Name of process
        :param priority (int): Priority that determines when the model gets updated. (Higher number = Higher priority)
        :return: simulationArchTypes.ProcessBaseClass object
        """
        proc = simulationArchTypes.ProcessBaseClass(procName, priority)
        self.procList.append(proc)
        self.TotalSim.addNewProcess(proc.processData)
        return proc


    def CreateNewTask(self, TaskName, TaskRate, InputDelay=None, FirstStart=0):
        """
        Creates a simulation task on the C-level with a specific update-frequency (TaskRate), an optional delay, and
        an optional start time.

        Args:
            TaskName (str): Name of Task
            TaskRate (int): Number of nanoseconds to elapse before update() is called
            InputDelay (int): (depreciated, unimplemented) Number of nanoseconds simulating a lag of the particular task
            FirstStart (int): Number of nanoseconds to elapse before task is officially enabled

        Returns:
            simulationArchTypes.TaskBaseClass object
        """

        if InputDelay is not self.CreateNewTask.__defaults__[0]:
            deprecated.deprecationWarn("InputDelay", "2024/12/13",
                                       "This input variable is non-functional and now depreciated.")

        Task = simulationArchTypes.TaskBaseClass(TaskName, TaskRate, FirstStart)
        self.TaskList.append(Task)
        return Task

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

    def reset(self, taskName):
        for Task in self.TaskList:
            if Task.Name == taskName:
                Task.reset(self.TotalSim.getCurrentNanos())

    def InitializeSimulation(self):
        """
        Initialize the BSK simulation.  This runs the reset() method on each module.
        """
        self.TotalSim.resetSimulation()
        self.simulationInitialized = True


    def ConfigureStopTime(self, TimeStop):
        """
        Set the simulation stop time in nano-seconds.
        """
        self.StopTime = TimeStop

    @deprecated.deprecated("2024/09/06",
        "Calling 'RecordLogVars' is deprecated and unnecessary."
    )
    def RecordLogVars(self):
        pass

    def ExecuteSimulation(self):
        """
        run the simulation until the prescribed stop time or termination.
        """
        self.initializeEventChecks()

        nextStopTime = self.TotalSim.getNextTaskTime()
        nextPriority = -1
        progressBar = SimulationProgressBar(self.StopTime, self.showProgressBar)
        while self.TotalSim.getNextTaskTime() <= self.StopTime and not self.terminate:
            if self.TotalSim.getCurrentNanos() >= self.nextEventTime >= 0:
                self.nextEventTime = self.checkEvents()
                self.nextEventTime = self.nextEventTime if self.nextEventTime >= self.TotalSim.getNextTaskTime() else self.TotalSim.getNextTaskTime()
            if 0 <= self.nextEventTime < nextStopTime:
                nextStopTime = self.nextEventTime
                nextPriority = -1
            if self.terminate:
                break
            self.TotalSim.stepUntilStop(nextStopTime, nextPriority)
            progressBar.update(self.TotalSim.getNextTaskTime())
            nextPriority = -1
            nextStopTime = self.StopTime
            nextStopTime = nextStopTime if nextStopTime >= self.TotalSim.getNextTaskTime() else self.TotalSim.getNextTaskTime()
        self.terminate = False
        progressBar.markComplete()
        progressBar.close()

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

    def parseDataIndex(self):
        self.dataStructureDictionary = {}
        try:
            xmlData = ET.parse(self.dataStructIndex)
        except:
            print("Failed to parse the XML index.  Likely that it isn't present")
            return
        root = xmlData.getroot()
        for child in root:
            newStruct = StructDocData(child.attrib['refid'])
            self.dataStructureDictionary.update({child.find('name').text:
                                                     newStruct})
        self.indexParsed = True

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


def SetCArray(InputList, VarType, ArrayPointer):
    setitem = getattr(sim_model, VarType + "Array_setitem")

    for (CurrIndex, CurrElem) in enumerate(InputList):
        setitem(ArrayPointer, CurrIndex, CurrElem)


def getCArray(varType, arrayPointer, arraySize):
    getitem = getattr(sim_model, varType + "Array_getitem")

    return [
        getitem(arrayPointer, currIndex)
        for currIndex in range(arraySize)
    ]

def synchronizeTimeHistories(arrayList):
    timeCounter = 0
    for i in range(len(arrayList)):
        while arrayList[i][0,0] > arrayList[0][timeCounter,0]:
            timeCounter += 1
    for i in range(len(arrayList)):
        while(arrayList[i][1,0] < arrayList[0][timeCounter,0]):
            arrayList[i] = np.delete(arrayList[i], 0, 0)

    timeCounter = -1
    for i in range(len(arrayList)):
        while arrayList[i][-1,0] < arrayList[0][timeCounter,0]:
                timeCounter -= 1
    for i in range(len(arrayList)):
        while(arrayList[i][-2,0] > arrayList[0][timeCounter,0]):
            arrayList[i] = np.delete(arrayList[i], -1, 0)

    timeNow = arrayList[0][0,0] #Desirement is to have synched arrays match primary time
    indexPrev = [0]*len(arrayList)

    outputArrayList = [[]]*len(arrayList)
    outputArrayList[0] = arrayList[0][0:-2, :]

    for i in range(1, arrayList[0].shape[0]-1):
        for j in range(1, len(arrayList)):
            while(arrayList[j][indexPrev[j]+1,0] < arrayList[0][i,0]):
                indexPrev[j] += 1

            dataProp = arrayList[j][indexPrev[j]+1,1:] - arrayList[j][indexPrev[j],1:]
            dataProp *= (timeNow - arrayList[j][indexPrev[j],0])/(arrayList[j][indexPrev[j]+1,0] - arrayList[j][indexPrev[j],0])
            dataProp += arrayList[j][indexPrev[j],1:]
            dataRow = [timeNow]
            dataRow.extend(dataProp.tolist())
            outputArrayList[j].append(dataRow)
        timeNow = arrayList[0][i,0]

    for j in range(1, len(arrayList)):
        outputArrayList[j] = np.array(outputArrayList[j])

    return outputArrayList
