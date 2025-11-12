#ifndef stateVecIntegrator_h
#define stateVecIntegrator_h

#include <vector>

class DynamicObject;

/*! @brief state vector integrator class */
class StateVecIntegrator
{

public:
    StateVecIntegrator(DynamicObject* dynIn);
    virtual ~StateVecIntegrator(void);
    virtual void integrate(double currentTime, double timeStep) = 0; //!< class method
    std::vector<DynamicObject*> dynPtrs; //!< This is an object that contains the method equationsOfMotion(), also known as the F function.

};


#endif /* StateVecIntegrator_h */
