#ifndef POINTCLOUDMSG_H
#define POINTCLOUDMSG_H

#include <architecture/msgPayloadDef/definitions.h>
#include <array>

//!@brief N-D point cloud
/*! This message contains a point cloud and corresponding time tag
 */
typedef struct
    //@cond DOXYGEN_IGNORE
    PointCloudMsgPayload
//@endcond
{
    uint64_t timeTag;                                 //!< --[ns]   Current vehicle time-tag associated with cloud
    bool valid;                                       //!< --  Quality of measurement
    int numberOfPoints;                               //!< -- [-] Number of points detected
    double points[MAX_SICP_POINTS * SICP_POINT_DIM];  //!< -- [-]  Point cloud array
} PointCloudMsgPayload;

#endif /* POINTCLOUDMSG_H */
