// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "moduleTemplates/cppModuleTemplate/cppModuleTemplate.h"

#include "architecture/utilities/linearAlgebra.h"

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
CppModuleTemplate::CppModuleTemplate() {}

/*! Module Destructor.  */
CppModuleTemplate::~CppModuleTemplate() { return; }

/*! This method is used to reset the module.
    @return void
 */
void CppModuleTemplate::reset(uint64_t currentSimNanos) {
    /*! - reset any required variables */
    this->dummy = 0.0;
    bskLogger.bskLog(BSK_INFORMATION, "Variable dummy set to %f in reset.", this->dummy);
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
 */
void CppModuleTemplate::updateState(uint64_t currentSimNanos) {
    ModuleTemplateMsgPayload outMsgBuffer{}; /*!< local output message copy */
    double inputVector[3];
    v3SetZero(inputVector);

    /*! - Read the optional input messages */
    if (this->dataInMsg.isLinked()) {
        ModuleTemplateMsgPayload inMsgBuffer = this->dataInMsg();
        v3Copy(inMsgBuffer.dataVector, inputVector);
    }

    /*! - Add the module specific code */
    double Lr[3]; /*!< [unit] variable description */
    v3Copy(inputVector, Lr);
    this->dummy += 1.0;
    Lr[0] += this->dummy;

    /*! - store the output message */
    v3Copy(Lr, outMsgBuffer.dataVector);

    /*! - write the module output message */
    this->dataOutMsg.write(&outMsgBuffer, this->moduleID, currentSimNanos);

    /* this logging statement is not typically required.  It is done here to see in the
     quick-start guide which module is being executed */
    bskLogger.bskLog(
        BSK_INFORMATION, "Module ID %lld ran Update at %fs", this->moduleID, (double)currentSimNanos / (1e9));
}
