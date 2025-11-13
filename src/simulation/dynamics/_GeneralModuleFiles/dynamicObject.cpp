#include "dynamicObject.h"

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator) {
    if (this->isDynamicsSynced) {
        bskLogger.bskLog(BSK_WARNING,
                         "You cannot set the integrator of a DynamicObject with synced integration. "
                         "If you want to change the integrator, change the integrator of the primary "
                         "DynamicObject.");
        return;
    }

    if (!newIntegrator) {
        bskLogger.bskLog(BSK_ERROR, "New integrator cannot be a null pointer");
        return;
    }

    if (newIntegrator->dynPtrs.at(0) != this) {
        bskLogger.bskLog(BSK_ERROR, "New integrator must have been created using this DynamicObject");
        return;
    }

    // If there was already an integrator set, then whatever dynPtrs that the
    // original integrator had take priority over the dynPtrs of newIntegrator
    if (this->integrator) {
        newIntegrator->dynPtrs = std::move(this->integrator->dynPtrs);
    }

    delete this->integrator;

    this->integrator = newIntegrator;
}

void DynamicObject::syncDynamicsIntegration(DynamicObject* dynPtr) {
    this->integrator->dynPtrs.push_back(dynPtr);
    dynPtr->isDynamicsSynced = true;
}

void DynamicObject::integrateState(double integrateToThisTime) {
    if (this->isDynamicsSynced) return;

    for (const auto& dynPtr : this->integrator->dynPtrs) {
        dynPtr->preIntegration(integrateToThisTime);
    }

    this->integrator->integrate(this->timeBefore, this->timeStep);

    for (const auto& dynPtr : this->integrator->dynPtrs) {
        dynPtr->postIntegration(integrateToThisTime);
    }
}
