# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

import numpy

from xmera.simulation import stateArchitecture


def test_stateData(show_plots):

    stateUse = [[10.0], [20.0]]
    newState = stateArchitecture.StateData(stateUse)
    newState.setState(stateUse)

    predictedDerivative = [[0.0], [0.0]]

    assert newState.getRowSize() == len(stateUse), "State row sized incorrectly"
    assert newState.getColumnSize() == len(stateUse[0]), "State column sized incorrectly"
    assert newState.getState() == stateUse, "State equality check failure."
    assert newState.getStateDeriv() == predictedDerivative, "State derivative zero check failure."

    derivativeInc = [[1.0], [2.5]]
    newState.setDerivative(derivativeInc)
    newState.propagateState(0.1)

    predictedDerivativeNum = numpy.array(predictedDerivative) + numpy.array(derivativeInc)
    obsDerivativeNum = numpy.array(newState.getStateDeriv())
    assert obsDerivativeNum.tolist() == predictedDerivativeNum.tolist(), "State derivative update check failure."

    stateUpdateNum = numpy.array(newState.getState())
    predUpStateNum = numpy.array(stateUse) + predictedDerivativeNum*0.1
    assert stateUpdateNum.tolist() == stateUpdateNum.tolist(), "State propagation update check failure."

    priorState = stateUpdateNum
    scaleFactor = 0.25
    priorState *= scaleFactor
    outState = newState*scaleFactor
    newState.scaleState(scaleFactor)
    stateUpdateNum = numpy.array(newState.getState())
    assert stateUpdateNum.tolist() == priorState.tolist(), "State scaling update check failure."
    assert outState.getState() == newState.getState(), "State scaling via * operator check failure."

    dummyState = stateArchitecture.StateData()
    assert dummyState.getRowSize() == 0, "Dummy state row sized incorrectly"
    assert dummyState.getColumnSize() == 0, "Dummy state column sized incorrectly"

    dummyState.setState(newState.getState())

    outState = dummyState + newState
    assert outState.getState() == (2.0*stateUpdateNum).tolist(), "Plus operator failed on StateData"


def test_stateProperties(show_plots):

    newManager = stateArchitecture.DynParamManager()

    gravList = [[9.81], [0.0], [0.1]]
    gravName = "g_N"
    newManager.createProperty(gravName, gravList)

    propRef = newManager.getPropertyReference(gravName)
    assert propRef == gravList, "Create and property reference matching failed."

    newGravList = [[0.0], [9.81], [-0.1]]
    newManager.setPropertyValue(gravName, newGravList)
    propRef = newManager.getPropertyReference(gravName)
    assert propRef == newGravList, "Set and property reference matching failed."

    newGravList = [[0.0], [9.81*2], [-0.1]]
    newManager.createProperty(gravName, newGravList)
    propRef = newManager.getPropertyReference(gravName)
    assert propRef == newGravList, "Set and property reference matching failed."

    wrongGravList = [[0.0], [9.81], [-0.1]]
    newManager.setPropertyValue(gravName+"Scott", newGravList)
    propRef = newManager.getPropertyReference(gravName+"Scott")
    assert propRef == None, "Set and property reference matching failed."

    massList = [[1500.0]]
    massName = "mass"
    newManager.createProperty(massName, massList)
    massRef = newManager.getPropertyReference(massName)
    assert massRef == massList, "1x1 Eigen property creation failed."


def test_stateArchitecture(show_plots):

    newManager = stateArchitecture.DynParamManager()

    positionName = "position"
    stateDim = [3, 1]
    posState = newManager.registerState(stateDim[0], stateDim[1], positionName)

    velocityName = "velocity"
    stateDim = [3, 1]
    velState = newManager.registerState(stateDim[0], stateDim[1], velocityName)

    flexName = "Array1_flex"
    flexDim = [2, 1]
    flexState = newManager.registerState(flexDim[0], flexDim[1], flexName)
    assert posState.getRowSize() == stateDim[0] or posState.getColumnSize() == stateDim[1], \
        "Position state returned improper size"

    newManager.registerState(stateDim[0], stateDim[1]+2, positionName)

    positionStateLookup = newManager.getStateObject("Array1_flex")

    vectorFactor = 4.0
    vecStart = [[1.0], [2.0], [3.5]]
    posState.setState(vecStart)
    velState.setState(vecStart)
    vectorStart = newManager.getStateVector()
    vectorComposite = vectorStart + vectorStart*vectorFactor + vectorStart*vectorFactor
    numpyOutput = numpy.array(vecStart) + numpy.array(vecStart)*vectorFactor + numpy.array(vecStart)*vectorFactor
    newManager.updateStateVector(vectorComposite)

    assert velState.getState() == numpyOutput.tolist(), "Velocity state update via state-manager failed"

    dt = 1.0
    posState.setDerivative(vecStart)
    newManager.propagateStateVector(dt)
    numpyOutput += numpy.array(vecStart)*dt
    assert posState.getState() == numpyOutput.tolist(), "Position state propagation via state-manager failed"


if __name__ == "__main__":
    test_stateProperties(False)
    test_stateData(False)
    test_stateArchitecture(False)
