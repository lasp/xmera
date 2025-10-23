/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
%module spiceInterface
%{
   #include "spiceInterface.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_string.i>
%include <std_vector.i>

%template() std::vector<std::string>;

// Declaring planetFrames %naturalvar removes the need for the StringVector wrapper:
//    mySpiceInterface.planetFrames = ["a", "b", "c"]
// is allowed, which is more pythonic than:
//    mySpiceInterface.planetFrames = spiceInterface.StringVector(["a", "b", "c"])
// (which is also allowed)
// However, modifiying in place is forbidden:
//    mySpiceInterface.planetFrames[2] = "bb"
// this raises an error because mySpiceInterface.planetFrames is returned by value
%naturalvar SpiceInterface::planetFrames;

%include <architecture/_GeneralModuleFiles/sys_model.i>

%include "spiceInterface.h"

%include <architecture/msgPayloadDef/EpochMsgPayload.h>

%include <architecture/msgPayloadDef/SpicePlanetStateMsgPayload.h>

%include <architecture/msgPayloadDef/SpiceTimeMsgPayload.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>

%include <architecture/msgPayloadDef/AttRefMsgPayload.h>

%include <architecture/msgPayloadDef/TransRefMsgPayload.h>
