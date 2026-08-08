// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder

#include "extendedStateVector.h"

ExtendedStateVector ExtendedStateVector::fromStates(const std::vector<DynamicObject*>& dynPtrs) {
    ExtendedStateVector result;

    for (size_t dynIndex = 0; dynIndex < dynPtrs.size(); dynIndex++) {
        for (auto&& [stateName, stateData] : dynPtrs.at(dynIndex)->dynManager.stateContainer.stateMap) {
            result.emplace(std::make_pair(dynIndex, stateName), stateData.getState());
        }
    }

    return result;
}

ExtendedStateVector ExtendedStateVector::fromStateDerivs(const std::vector<DynamicObject*>& dynPtrs) {
    ExtendedStateVector result;

    for (size_t dynIndex = 0; dynIndex < dynPtrs.size(); dynIndex++) {
        for (auto&& [stateName, stateData] : dynPtrs.at(dynIndex)->dynManager.stateContainer.stateMap) {
            result.emplace(std::make_pair(dynIndex, stateName), stateData.getStateDeriv());
        }
    }

    return result;
}

ExtendedStateVector &ExtendedStateVector::operator+=(const ExtendedStateVector &rhs) {
    for (auto&& [extendedStateId, stateMatrix] : *this) {
        stateMatrix += rhs.at(extendedStateId);
    }

    return *this;
}

ExtendedStateVector ExtendedStateVector::operator*(const double rhs) const {
    ExtendedStateVector result;
    result.reserve(this->size());

    for (auto&& [extendedStateId, stateMatrix] : *this) {
        result.emplace(extendedStateId, stateMatrix * rhs);
    }

    return result;
}

void ExtendedStateVector::setStates(std::vector<DynamicObject*>& dynPtrs) const {
    for (auto&& [extendedStateId, stateMatrix] : *this) {
        const auto& [dynObjIndex, stateName] = extendedStateId;

        StateData& stateData = dynPtrs.at(dynObjIndex)->dynManager.stateContainer.stateMap.at(stateName);
        stateData.setState(stateMatrix);
    }
}
