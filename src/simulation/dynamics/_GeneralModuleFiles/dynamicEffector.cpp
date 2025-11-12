#include "dynamicEffector.h"

/*! This is the constructor, just setting the variables to zero */
DynamicEffector::DynamicEffector()
{
    // Set forces and torques to zero
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
    return;
}

/*! This is the destructor, nothing to report here */
DynamicEffector::~DynamicEffector()
{
    return;
}

/*! This method is an optional method by a dynamic effector and allows the dynamics effector to add direct contributions
    to a state effector derivative. Example - a thruster's mDot will impact a fuel tanks total mDot */
void DynamicEffector::computeStateContribution(double integTime)
{
    return;
}
