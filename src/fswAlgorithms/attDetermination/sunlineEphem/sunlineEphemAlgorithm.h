#ifndef _SUNLINE_EPHEM_ALGORITHM_H_
#define _SUNLINE_EPHEM_ALGORITHM_H_

#include <architecture/msgPayloadDef/EphemerisMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/utilities/rigidBodyKinematics.hpp>
#include <assert.h>
#include <stdint.h>
#include <Eigen/Core>

class SunlineEphemAlgorithm {
   public:
    NavAttMsgPayload updateState(const EphemerisMsgPayload& sunPos,
                                 const NavTransMsgPayload& scPos,
                                 const NavAttMsgPayload& scAtt) const;
};

#endif
