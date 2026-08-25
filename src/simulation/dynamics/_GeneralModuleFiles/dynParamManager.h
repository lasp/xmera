// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

    void setDerivativesFrom(StateVector const &other) {
        auto itSelf = this->stateMap.begin();
        auto itOther = other.stateMap.begin();
        for (; itSelf != this->stateMap.end(); ++itSelf, ++itOther) {
            itSelf->second.stateDeriv = itOther->second.stateDeriv;
        }
    }

    void propagateState(double dt) {
        for (auto &[_, data] : this->stateMap) { data.propagateState(dt); }
    }
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
