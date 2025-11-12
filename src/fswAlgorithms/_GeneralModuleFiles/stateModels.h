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

   public:
    Eigen::Index size() const;

    FilterStateVector add(const FilterStateVector& vector) const;
    FilterStateVector addVector(const Eigen::Vector<double, SIZE>& vector) const;
    FilterStateVector scale(const double scalar) const;

    Eigen::Vector<double, SIZE> returnValues() const;

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


namespace xmera{
    template<typename StateVector>
    concept is_state_vector = requires(StateVector state, StateVector const constState) {
        { StateVector::SIZE }
            -> std::convertible_to<Eigen::Index const&>;

        { constState.size() }
            -> std::convertible_to<Eigen::Index>;
        { constState.returnValues() }
            -> std::same_as<Eigen::Vector<double, StateVector::SIZE>>;

        { constState.scale(std::declval<double>()) }
            -> std::same_as<StateVector>;
        { constState.add(constState) }
            -> std::same_as<StateVector>;
        { constState.addVector(constState.returnValues()) }
            -> std::same_as<StateVector>;
    };

    template<typename StateVector>
    concept has_position = requires(StateVector const constState) {
        { StateVector::ROWS }
            -> std::convertible_to<Eigen::Index const&>;
        { constState.hasPosition() }
            -> std::convertible_to<bool>;
        { constState.getPositionStates() }
            -> std::same_as<Eigen::Vector<double, StateVector::ROWS>>;
    };

    template<typename StateVector>
    concept has_velocity = requires(StateVector const constState) {
        { StateVector::ROWS }
            -> std::convertible_to<Eigen::Index const&>;
        { constState.hasVelocity() }
            -> std::convertible_to<bool>;
        { constState.getVelocityStates() }
            -> std::same_as<Eigen::Vector<double, StateVector::ROWS>>;
    };

    template<typename StateVector>
    concept has_acceleration = requires(StateVector const constState) {
        { StateVector::ROWS }
            -> std::convertible_to<Eigen::Index const&>;
        { constState.hasAcceleration() }
            -> std::convertible_to<bool>;
        { constState.getAccelerationStates() }
            -> std::same_as<Eigen::Vector<double, StateVector::ROWS>>;
    };

    template<typename StateVector>
    concept has_bias = requires(StateVector const constState) {
        { StateVector::ROWS }
            -> std::convertible_to<Eigen::Index const&>;
        { constState.hasBias() }
            -> std::convertible_to<bool>;
        { constState.getBiasStates() }
            -> std::same_as<Eigen::Vector<double, StateVector::ROWS>>;
    };

    template<typename StateVector>
    concept has_consider = requires(StateVector const constState) {
        { StateVector::ROWS }
            -> std::convertible_to<Eigen::Index const&>;
        { constState.hasConsider() }
            -> std::convertible_to<bool>;
        { constState.getConsiderStates() }
            -> std::same_as<Eigen::Vector<double, StateVector::ROWS>>;
    };
}

#endif
