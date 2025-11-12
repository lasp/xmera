#include "extendedStateVector.h"

ExtendedStateVector ExtendedStateVector::fromStates(const std::vector<DynamicObject*>& dynPtrs)
{
    return fromStateData(dynPtrs, [](const StateData& data) { return data.getState(); });
}

ExtendedStateVector ExtendedStateVector::fromStateDerivs(const std::vector<DynamicObject*>& dynPtrs)
{
    return fromStateData(dynPtrs, [](const StateData& data) { return data.getStateDeriv(); });
}

ExtendedStateVector ExtendedStateVector::map(
    std::function<Eigen::MatrixXd(const size_t&, const std::string&, const Eigen::MatrixXd&)>
        functor) const
{
    ExtendedStateVector result;
    result.reserve(this->size());

    for (auto&& [extendedStateId, stateMatrix] : *this) {
        const auto& [dynObjIndex, stateName] = extendedStateId;
        result.emplace(extendedStateId, functor(dynObjIndex, stateName, stateMatrix));
    }

    return result;
}

void ExtendedStateVector::apply(
    std::function<void(const size_t&, const std::string&, const Eigen::MatrixXd&)> functor) const
{
    for (auto&& [extendedStateId, stateMatrix] : *this) {
        const auto& [dynObjIndex, stateName] = extendedStateId;
        functor(dynObjIndex, stateName, stateMatrix);
    }
}

void ExtendedStateVector::modify(
    std::function<void(const size_t&, const std::string&, Eigen::MatrixXd&)> functor)
{
    for (auto&& [extendedStateId, stateMatrix] : *this) {
        const auto& [dynObjIndex, stateName] = extendedStateId;
        functor(dynObjIndex, stateName, stateMatrix);
    }
}

ExtendedStateVector ExtendedStateVector::operator+=(const ExtendedStateVector& rhs)
{
    this->modify([&rhs](const size_t& dynObjIndex,
                        const std::string& stateName,
                        Eigen::MatrixXd& thisState) {
        thisState += rhs.at({dynObjIndex, stateName});
    });

    return *this;
}

ExtendedStateVector ExtendedStateVector::operator*(const double rhs) const
{
    return this->map([rhs](const size_t& dynObjIndex,
                           const std::string& stateName,
                           const Eigen::MatrixXd& thisState) { return thisState * rhs; });
}

void ExtendedStateVector::setStates(std::vector<DynamicObject*>& dynPtrs) const
{
    this->apply([&dynPtrs](const size_t& dynObjIndex,
                                 const std::string& stateName,
                                 const Eigen::MatrixXd& thisState) {
        StateData& stateData =
            dynPtrs.at(dynObjIndex)->dynManager.stateContainer.stateMap.at(stateName);
        stateData.setState(thisState);
    });
}

ExtendedStateVector
ExtendedStateVector::fromStateData(const std::vector<DynamicObject*>& dynPtrs,
                                   std::function<Eigen::MatrixXd(const StateData&)> functor)
{
    ExtendedStateVector result;

    for (size_t dynIndex = 0; dynIndex < dynPtrs.size(); dynIndex++) {
        for (auto&& [stateName, stateData] :
             dynPtrs.at(dynIndex)->dynManager.stateContainer.stateMap) {
            result.emplace(std::make_pair(dynIndex, stateName), functor(stateData));
        }
    }

    return result;
}
