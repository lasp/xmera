// SPDX-License-Identifier: ISC
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "vehicleConfigData.h"
#include <architecture/utilities/linearAlgebra.h>
#include <architecture/utilities/macroDefinitions.h>

void VehicleConfigData::reset(uint64_t callTime) {
    /*! - Zero the output message data */
    auto localConfigData = VehicleConfigMsgPayload();

    /*! - Copy over the center of mass location */
    v3Copy(this->CoM_B, localConfigData.CoM_B);

    /*! - Copy over the inertia */
    m33Copy(RECAST3X3 this->ISCPntB_B, RECAST3X3 localConfigData.ISCPntB_B);

    /*! - Copy over the mass */
    localConfigData.massSC = this->massSC;

    /*! - Write output properties to the messaging system*/
    this->vecConfigOutMsg.write(localConfigData, this->moduleID, callTime);
}
