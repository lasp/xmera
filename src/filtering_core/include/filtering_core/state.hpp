// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_CORE_STATE_HPP
#define FILTERING_CORE_STATE_HPP

#include <Eigen/Core>
#include <type_traits>

namespace filtering {

// Component tag types. Each declares its size; the StateVector lays them out
// contiguously in an Eigen vector. Tag types carry no storage themselves.
template<int N> struct Position     { static constexpr int size = N; };
template<int N> struct Velocity     { static constexpr int size = N; };
template<int N> struct Acceleration { static constexpr int size = N; };
template<int N> struct Bias         { static constexpr int size = N; };
template<int N> struct MrpAttitude  { static constexpr int size = N; };
template<int N> struct AngularRate  { static constexpr int size = N; };
template<int N> struct Consider     { static constexpr int size = N; };

namespace detail {

template<class Target, class... Pack>
struct offsetOf;

// Match: target is at the head of the pack.
template<class Target, class... Rest>
struct offsetOf<Target, Target, Rest...> {
    static constexpr int value = 0;
};

// Skip: head doesn't match, recurse with one fewer element.
template<class Target, class First, class... Rest>
struct offsetOf<Target, First, Rest...> {
    static constexpr int value = First::size + offsetOf<Target, Rest...>::value;
};

template<class Target, class... Components>
inline constexpr bool contains = (std::is_same_v<Target, Components> || ...);

}  // namespace detail

// Variadic, type-composed state vector.
//
//   using FlybyState = StateVector<Position<3>, Velocity<3>>;          // size = 6
//   using SunlineState = StateVector<Position<3>, Velocity<3>, Bias<3>>;  // size = 9
//
// Storage is a single fixed-size Eigen vector. Components are addressed by
// their tag type (compile-time):
//
//   FlybyState s;
//   s.set<Position<3>>(Eigen::Vector3d{1, 2, 3});
//   auto v = s.get<Velocity<3>>();
//
// Satisfies LinearlyCombinable via add()/scale() on the underlying storage.
template<class... Components>
class StateVector {
public:
    static constexpr int size = (Components::size + ...);
    using Storage = Eigen::Vector<double, size>;

    StateVector() : data(Storage::Zero()) {}
    explicit StateVector(Storage const& s) : data(s) {}

    template<class Component>
        requires detail::contains<Component, Components...>
    Eigen::Vector<double, Component::size> get() const {
        constexpr int offset = detail::offsetOf<Component, Components...>::value;
        return this->data.template segment<Component::size>(offset);
    }

    template<class Component>
        requires detail::contains<Component, Components...>
    void set(Eigen::Vector<double, Component::size> const& value) {
        constexpr int offset = detail::offsetOf<Component, Components...>::value;
        this->data.template segment<Component::size>(offset) = value;
    }

    Storage const& raw() const { return this->data; }

    StateVector add(StateVector const& other) const {
        return StateVector(this->data + other.data);
    }

    StateVector scale(double scalar) const {
        return StateVector(this->data * scalar);
    }

private:
    Storage data;
};

// Compile-time sanity checks: ensure the variadic machinery resolves.
namespace detail {
    using TestState6 = StateVector<Position<3>, Velocity<3>>;
    static_assert(TestState6::size == 6);
    static_assert(offsetOf<Position<3>, Position<3>, Velocity<3>>::value == 0);
    static_assert(offsetOf<Velocity<3>, Position<3>, Velocity<3>>::value == 3);

    using TestState9 = StateVector<Position<3>, Velocity<3>, Bias<3>>;
    static_assert(TestState9::size == 9);
    static_assert(offsetOf<Bias<3>, Position<3>, Velocity<3>, Bias<3>>::value == 6);

    static_assert(contains<Position<3>, Position<3>, Velocity<3>>);
    static_assert(!contains<Bias<3>, Position<3>, Velocity<3>>);
}  // namespace detail

}  // namespace filtering

#endif
