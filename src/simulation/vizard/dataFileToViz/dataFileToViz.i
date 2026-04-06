// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

%module dataFileToViz
%{
   #include "dataFileToViz.h"
%}

%include <architecture/_GeneralModuleFiles/swig_conly_data.i>
%include <std_string.i>
%import <architecture/_GeneralModuleFiles/sys_model.i>
%include <std_vector.i>


%include "dataFileToViz.h"
%include <simulation/vizard/_GeneralModuleFiles/vizStructures.h>

%include <architecture/msgPayloadDef/SCStatesMsgPayload.h>
%include <architecture/msgPayloadDef/RWConfigLogMsgPayload.h>
%include <architecture/msgPayloadDef/THROutputMsgPayload.h>


// Instantiate templates used by example
namespace std {
    %template(VizThrConfig) vector<ThrClusterMap, std::allocator<ThrClusterMap> >;
    %template(ThrClusterMapVectorVector) vector <vector <ThrClusterMap, allocator<ThrClusterMap> >, allocator<vector<ThrClusterMap>> >;
    %template(THROutputMsgOutMsgsVector) vector<Message<THROutputMsgPayload>, allocator<Message<THROutputMsgPayload>> >;
    %template(THROutputMsgOutMsgsPtrVector) vector<Message<THROutputMsgPayload>*, allocator<Message<THROutputMsgPayload>*> >;
    %template(THROutputMsgInMsgsVector) vector<ReadFunctor<THROutputMsgPayload>, allocator<ReadFunctor<THROutputMsgPayload>> >;
    %template(THROutputOutMsgsVectorVector) vector <vector <Message<THROutputMsgPayload>*, allocator<Message<THROutputMsgPayload>*> >, allocator<vector <Message<THROutputMsgPayload>*>> >;
    %template(RWConfigLogMsgOutMsgsVector) vector<Message<RWConfigLogMsgPayload>, allocator<Message<RWConfigLogMsgPayload>> >;
    %template(RWConfigLogMsgOutMsgsPtrVector) vector<Message<RWConfigLogMsgPayload>*, allocator<Message<RWConfigLogMsgPayload>*> >;
    %template(RWConfigLogMsgInMsgsVector) vector<ReadFunctor<RWConfigLogMsgPayload>, allocator<ReadFunctor<RWConfigLogMsgPayload>> >;
    %template(RWConfigLogMsgInMsgsVectorVector) vector <vector <Message<RWConfigLogMsgPayload>*, allocator<Message<RWConfigLogMsgPayload>*> >, allocator<vector <Message<RWConfigLogMsgPayload>*>> >;
    %template(IntVector) std::vector<int, std::allocator<int>>;
}
