// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef extendedStateVector_h
#define extendedStateVector_h

#include <simulation/dynamics/_GeneralModuleFiles/dynamicObject.h>
#include <simulation/dynamics/_GeneralModuleFiles/dynParamManager.h>

#include <Eigen/Dense>
#include <functional>
#include <stdint.h>
#include <unordered_map>

/*
Each DynamicObject has a series of states associated to them. Each state
is associated with a unique std::string (i.e. "hubPosition"), and their
value is stored as Eigen::MatrixXd. Each of these states is integrated
in parallel: each have their own derivative, which are all calculated
in DynamicObject::equationsOfMotion.

Basilisk also supports the parallel integration of multiple DynamicObjects,
which in practice means that all states of all DynamicObjects need to
be integrated in parallel.

In order to facilitate this task, the ExtendedStateVector was created.
This class is a map that can hold an Eigen::MatrixXd for every state
of every DynamicObject that we want to integrate. Thus, it can be
used to store the value of the states, their derivatives, errors...
in a single, flat object. This is similar to the behaviour of StateVector,
except that this supports multiple DynamicObjects.

ExtendedStateVector supports a series of utility functions that
makes performing state-wise operations easier.
*/

/**
 * Because two different DynamicObjects can have states with the same
 * name, ExtendedStateId is used to give a unique identifier to
 * every state. The first item is the index of the DynamicObject
 * in the integrator's dynPtrs vector. The second item in the pair
 * is the name of the state.
 */
using ExtendedStateId = std::pair<size_t, std::string>;

/** ExtendedStateIdHash is required to make ExtendedStateId hashable (usable as a key in a map) */
struct ExtendedStateIdHash {
    /** Generates a hash value (integer) from an ExtendedStateId object */
    std::size_t operator()(const ExtendedStateId& p) const {
        auto seed = std::hash<size_t>{}(p.first);
        // Algorithm from boost::hash_combine
        seed ^= std::hash<std::string>{}(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

/**
 * Conceptually similar to StateVector, this class allows us to handle
 * the states of multiple DynamicObject with a single object.
 *
 * It also supports several utility functions.
 */
class ExtendedStateVector : public std::unordered_map<ExtendedStateId, Eigen::MatrixXd, ExtendedStateIdHash> {
   public:
    /**
     * Builds a ExtendedStateVector from all states in the given
     * dynamic objects
     */
    static ExtendedStateVector fromStates(const std::vector<DynamicObject*>& dynPtrs);

    /**
     * Builds a ExtendedStateVector from the derivatives of all states
     * in the given dynamic objects
     */
    static ExtendedStateVector fromStateDerivs(const std::vector<DynamicObject*>& dynPtrs);

    /** Adds the values of `rhs` to this
     *
     * This functions as a state-wise addition operation.
     */
    ExtendedStateVector &operator+=(const ExtendedStateVector& rhs);

    /** Returns a new ExtendedStateVector that is the result of multiplying each state by a constant
     */
    ExtendedStateVector operator*(const double rhs) const;

    /** Calls StateData::setState for every entry in in this */
    void setStates(std::vector<DynamicObject*>& dynPtrs) const;
};

#endif /* extendedStateVector_h */
