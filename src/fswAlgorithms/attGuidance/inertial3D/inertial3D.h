#ifndef INERTIAL3D_H
#define INERTIAL3D_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include "inertial3DAlgorithm.h"
#include <stdint.h>
#include <Eigen/Core>

/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
class Inertial3D : public SysModel {
   public:
    Inertial3D() = default;
    ~Inertial3D() final = default;

    void updateState(uint64_t callTime) override;
    void setSigmaR0N(const Eigen::Vector3d& sigma_RN);
    const Eigen::Vector3d& getSigmaR0N() const;

    Message<AttRefMsgPayload> attRefOutMsg;  //!< reference attitude output message

   private:
    Inertial3DAlgorithm algorithm{};
};

#endif
