// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <stdint.h>

#include <Eigen/Dense>

/*! @brief state data class*/
struct StateData {
    Eigen::MatrixXd state;       //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;  //!< [-] State derivative value storage

    StateData() = default;

    explicit StateData(Eigen::MatrixXd const &newState) : state(newState), stateDeriv(state) {
        stateDeriv.setZero();
    }

    void setState(Eigen::MatrixXd const &newState) {
        state = newState;
    }

    void propagateState(double const dt) {
        state += stateDeriv * dt;
    }

    void setDerivative(Eigen::MatrixXd const &newDeriv) {
        stateDeriv = newDeriv;
    }

    Eigen::MatrixXd const &getState() const {
        return state;
    }

    Eigen::MatrixXd const &getStateDeriv() const {
        return stateDeriv;
    }

    uint32_t getRowSize() const {
        return ((uint32_t) state.innerSize());
    }

    uint32_t getColumnSize() const {
        return ((uint32_t) state.outerSize());
    }

    void scaleState(double const scaleFactor) {
        state *= scaleFactor;
    }

    StateData operator+(StateData const &operand) const {
        StateData newState(state);
        newState.state += operand.getState();
        return newState;
    }

    StateData operator*(double const scaleFactor) const {
        StateData newState(state);
        newState.state *= scaleFactor;
        return newState;
    }
};

#endif /* STATE_DATA_H */
