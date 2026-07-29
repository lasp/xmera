// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "stateData.h"

StateData::StateData(const StateData& inState) : state(inState.state),
    stateDeriv(inState.stateDeriv),
    stateName(inState.stateName),
    stateEnabled(inState.stateEnabled) {
}

StateData::StateData(std::string inName, const Eigen::MatrixXd& newState) {
    stateName = inName;
    setState(newState);
    if (state.innerSize() != stateDeriv.innerSize() || state.outerSize() != stateDeriv.outerSize()) {
        stateDeriv = state;
    }
    stateDeriv.setZero();
}

void StateData::setState(const Eigen::MatrixXd& newState) {
    state = newState;
    return;
}

void StateData::propagateState(double dt) { state += stateDeriv * dt; }

void StateData::setDerivative(const Eigen::MatrixXd& newDeriv) { stateDeriv = newDeriv; }

void StateData::scaleState(double scaleFactor) { state *= scaleFactor; }

StateData StateData::operator+(const StateData& operand) { return StateData(stateName, state + operand.getState()); }

StateData StateData::operator*(double scaleFactor) {
    StateData newState(stateName, state);
    newState.scaleState(scaleFactor);
    return (newState);
}
