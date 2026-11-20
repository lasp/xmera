// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef CAMERA_RENDER_REQUEST_MSG_H
#define CAMERA_RENDER_REQUEST_MSG_H

#include <cstdint>

/*! @brief A request for a camera to render a single image immediately.
 */
struct CameraRenderRequestMsgPayload {
    int64_t cameraID;  //!< ID of the camera that should render
};

#endif
