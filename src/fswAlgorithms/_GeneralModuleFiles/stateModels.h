// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTER_STATE_MODELS_H
#define FILTER_STATE_MODELS_H

#include <Eigen/Core>
#include <optional>

enum class StateType { Position, Velocity, Acceleration, Bias, Consider };

/*! @brief State class */
template<StateType TYPE, Eigen::Index SIZE>
class State {
public:
    State() = default;

    explicit State(const Eigen::Vector<double, SIZE>& initialState)
        : values(initialState)
    {}

    void setValues(const Eigen::Vector<double, SIZE>& componentValues) {
        this->values = componentValues;
    }

    Eigen::Vector<double, SIZE> getValues() const {
        return this->values;
    }

    Eigen::Index size() const {
        return this->values.size();
    }

private:
    Eigen::Vector<double, SIZE> values;
};

template<Eigen::Index SIZE>
using PositionState = State<StateType::Position, SIZE>;

template<Eigen::Index SIZE>
using VelocityState = State<StateType::Velocity, SIZE>;

template<Eigen::Index SIZE>
using AccelerationState = State<StateType::Acceleration, SIZE>;

template<Eigen::Index SIZE>
using BiasState = State<StateType::Bias, SIZE>;

template<Eigen::Index SIZE>
using ConsiderState = State<StateType::Consider, SIZE>;


/*! @brief State models used to map a state vector to a measurement */
class FilterStateVector {
   public:
    static constexpr Eigen::Index ROWS = Eigen::Dynamic;
    static constexpr Eigen::Index SIZE = Eigen::Dynamic;
   private:
    std::optional<PositionState<ROWS>> position;
    std::optional<VelocityState<ROWS>> velocity;
    std::optional<AccelerationState<ROWS>> acceleration;
    std::optional<BiasState<ROWS>> bias;
    std::optional<ConsiderState<ROWS>> considerParameters;
    Eigen::Matrix<double, SIZE, SIZE> stm;

   public:
    Eigen::Index size() const;

    FilterStateVector add(const FilterStateVector& vector) const;
    FilterStateVector addVector(const Eigen::Vector<double, SIZE>& vector) const;
    FilterStateVector scale(const double scalar) const;

    Eigen::Vector<double, SIZE> returnValues() const;

    void attachStm(const Eigen::Matrix<double, SIZE, SIZE>& stm);
    Eigen::Matrix<double, SIZE, SIZE> detachStm() const;

    void setPosition(const PositionState<ROWS>& position);
    Eigen::Vector<double, ROWS> getPositionStates() const;
    bool hasPosition() const;

    void setVelocity(const VelocityState<ROWS>& velocity);
    Eigen::Vector<double, ROWS> getVelocityStates() const;
    bool hasVelocity() const;

    void setAcceleration(const AccelerationState<ROWS>& acceleration);
    Eigen::Vector<double, ROWS> getAccelerationStates() const;
    bool hasAcceleration() const;

    void setBias(const BiasState<ROWS>& bias);
    Eigen::Vector<double, ROWS> getBiasStates() const;
    bool hasBias() const;

    void setConsider(const ConsiderState<ROWS>& consider);
    Eigen::Vector<double, ROWS> getConsiderStates() const;
    bool hasConsider() const;
};

#endif
