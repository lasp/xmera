// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_STATE_MODELS_H
#define FILTER_STATE_MODELS_H

#include <Eigen/Core>
#include <optional>

enum class StateType { Position, Velocity, Acceleration, Bias, Consider };

/*! @brief State class */
template<StateType TYPE>
class State {
public:
    State() = default;

    explicit State(const Eigen::VectorXd& initialState)
        : values(initialState)
    {}

    void setValues(const Eigen::VectorXd& componentValues) {
        this->values = componentValues;
    }

    Eigen::VectorXd getValues() const {
        return this->values;
    }

    size_t size() const {
        return values.size();
    }

private:
    Eigen::VectorXd values;
};

using PositionState = State<StateType::Position>;
using VelocityState = State<StateType::Velocity>;
using AccelerationState = State<StateType::Acceleration>;
using BiasState = State<StateType::Bias>;
using ConsiderState = State<StateType::Consider>;


/*! @brief State models used to map a state vector to a measurement */
class FilterStateVector {
   private:
    std::optional<PositionState> position;
    std::optional<VelocityState> velocity;
    std::optional<AccelerationState> acceleration;
    std::optional<BiasState> bias;
    std::optional<ConsiderState> considerParameters;
    Eigen::MatrixXd stm;

   public:
    long size() const;

    FilterStateVector add(const FilterStateVector& vector) const;
    FilterStateVector addVector(const Eigen::VectorXd& vector) const;
    FilterStateVector scale(const double scalar) const;

    Eigen::VectorXd returnValues() const;

    void setPosition(const PositionState& position);
    Eigen::VectorXd getPositionStates() const;
    bool hasPosition() const;

    void setVelocity(const VelocityState& velocity);
    Eigen::VectorXd getVelocityStates() const;
    bool hasVelocity() const;

    void setAcceleration(const AccelerationState& acceleration);
    Eigen::VectorXd getAccelerationStates() const;
    bool hasAcceleration() const;

    void setBias(const BiasState& bias);
    Eigen::VectorXd getBiasStates() const;
    bool hasBias() const;

    void setConsider(const ConsiderState& consider);
    Eigen::VectorXd getConsiderStates() const;
    bool hasConsider() const;

    void attachStm(const Eigen::MatrixXd& stm);
    Eigen::MatrixXd detachStm() const;
};

#endif
