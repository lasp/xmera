// SPDX-License-Identifier: ISC
// Copyright (c) 2021, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module encoder
%{
   #include "encoder.h"
%}

%include "exception.i"

%exception {
    try {
        $action
    } catch (const std::invalid_argument& e) {
        SWIG_exception(SWIG_ValueError, e.what());
    } catch (const std::exception& e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    }
}

%include <std_string.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%include <architecture/_GeneralModuleFiles/sys_model.i>

%include <stdint.i>
%include <attribute.i>
%attribute(Encoder, std::size_t, numRW, getNumRW, setNumRW)
%attribute(Encoder, std::uint32_t, clicksPerRotation, getClicksPerRotation, setClicksPerRotation)

%include <std_vector.i>

// These traits change an EncoderSignal to and from a Python integer.
// Thus SWIG can change a Python list to a std::vector<EncoderSignal>.
%fragment(SWIG_Traits_frag(EncoderSignal), "header", fragment="StdTraits", fragment=SWIG_AsVal_frag(long), fragment=SWIG_From_frag(long)) {
namespace swig {
    template <> struct traits<EncoderSignal> {
        typedef value_category category;
        static const char* type_name() { return "EncoderSignal"; }
    };
    template <> struct traits_asval<EncoderSignal> {
        typedef EncoderSignal value_type;
        static int asval(PyObject* obj, value_type* val) {
            long value = 0;
            int const result = SWIG_AsVal(long)(obj, &value);
            if (SWIG_IsOK(result) && val) { *val = static_cast<EncoderSignal>(value); }
            return result;
        }
    };
    template <> struct traits_from<EncoderSignal> {
        static PyObject* from(EncoderSignal const& value) { return SWIG_From(long)(static_cast<long>(value)); }
    };
}
}
%template(EncoderSignalVector) std::vector<EncoderSignal>;
%naturalvar Encoder::signalStates;
%attributestring(Encoder, std::vector<EncoderSignal>, signalStates, getSignalStates, setSignalStates)

%include <architecture/utilities/macroDefinitions.h>
%include "encoder.h"

%include <architecture/msgPayloadDef/RWSpeedMsgPayload.h>
