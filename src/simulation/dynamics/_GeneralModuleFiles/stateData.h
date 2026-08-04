// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <stdint.h>

#include <Eigen/Dense>

/*! @brief state data class*/
class StateData {
public:
    Eigen::MatrixXd state;       //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;  //!< [-] State derivative value storage

public:
    StateData() = default;
    StateData(StateData const &inState) = default;
    explicit StateData(Eigen::MatrixXd const &newState);
    ~StateData() = default;
    void setState(Eigen::MatrixXd const &newState);
    void propagateState(double dt);
    void setDerivative(Eigen::MatrixXd const &newDeriv);

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

    void scaleState(double scaleFactor);

    StateData operator+(StateData const &operand) const;
    StateData operator*(double scaleFactor) const;
};

#endif /* STATE_DATA_H */
