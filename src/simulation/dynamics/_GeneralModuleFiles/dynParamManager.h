#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <stdint.h>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <simulation/dynamics/_GeneralModuleFiles/stateData.h>
#include <architecture/utilities/bskLogging.h>

/*! state vector class */
class StateVector {
   public:
    std::map<std::string, StateData> stateMap;  //!< class method
   public:
    StateVector operator+(const StateVector& operand);  //!< class method
    StateVector operator*(double scaleFactor);          //!< class method
};

/*! dynamic parameter manager class */
class DynParamManager {
   public:
    std::map<std::string, Eigen::MatrixXd> dynProperties;  //!< class variable
    StateVector stateContainer;                            //!< class variable
    BSKLogger bskLogger;                                   //!< -- BSK Logging
   public:
    DynParamManager();
    ~DynParamManager();
    StateData* registerState(uint32_t nRow, uint32_t nCol, std::string stateName);  //!< class method
    StateData* getStateObject(std::string stateName);                               //!< class method
    StateVector getStateVector();                                                   //!< class method
    void updateStateVector(const StateVector& newState);                            //!< class method
    void propagateStateVector(double dt);                                           //!< class method
    Eigen::MatrixXd* createProperty(std::string propName,
                                    const Eigen::MatrixXd& propValue);  //!< class method
    Eigen::MatrixXd* getPropertyReference(std::string propName);        //!< class method
    void setPropertyValue(const std::string propName,
                          const Eigen::MatrixXd& propValue);  //!< class method
};

#endif /* STATE_MANAGER_H */
