/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "fswAlgorithms/transDetermination/ephemNavConverter/ephemNavConverterAlgorithm.h"

inline void v3Copy(double const v[3], double result[3]) {
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}

/*! Update method for the ephemNavConverter algorithm. This method reads in the ephemeris messages and copies the
 translation ephemeris to the navigation translation interface message.
 @return NavTransMsgPayload Translational navigation message
 @param callTime [ns] Time the method is called
 @param ephemerisInMsg Ephemeris message
 */
NavTransMsgPayload EphemNavConverterAlgorithm::update(uint64_t callTime, EphemerisMsgPayload ephemerisInMsg) {
    // Create the output message
    auto navTransMsgBuffer = NavTransMsgPayload{};

    // Map timeTag, position and velocity vector to output message
    navTransMsgBuffer.timeTag = ephemerisInMsg.timeTag;
    v3Copy(ephemerisInMsg.r_BdyZero_N, navTransMsgBuffer.r_BN_N);
    v3Copy(ephemerisInMsg.v_BdyZero_N, navTransMsgBuffer.v_BN_N);

    return navTransMsgBuffer;
}
