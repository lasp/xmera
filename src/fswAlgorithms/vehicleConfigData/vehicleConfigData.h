// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef VEHICLECONFIGDATACPP_H
#define VEHICLECONFIGDATACPP_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/VehicleConfigMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>

class VehicleConfigData : public SysModel {
   public:
    void reset(uint64_t callTime) override;

    double ISCPntB_B[9];                              /*!< [kg m^2] Spacecraft Inertia */
    double CoM_B[3];                                  /*!< [m] Center of mass of spacecraft in body*/
    double massSC;                                    /*!< [kg] Spacecraft mass */
    Message<VehicleConfigMsgPayload> vecConfigOutMsg; /*!< [-] Name of the output properties message*/

    BSKLogger* bskLogger;  //!< BSK Logging
};

#endif  // VEHICLECONFIGDATACPP_H
