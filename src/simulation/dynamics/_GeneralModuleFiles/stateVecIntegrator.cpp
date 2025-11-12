#include "stateVecIntegrator.h"
#include "dynamicObject.h"

/*! @brief Constructor */
StateVecIntegrator::StateVecIntegrator(DynamicObject* dyn)
{
    this->dynPtrs.push_back(dyn);
}

/*! @brief Destructor */
StateVecIntegrator::~StateVecIntegrator(void)
{
    this->dynPtrs.clear();
}
