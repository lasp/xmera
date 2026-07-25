// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <architecture/system_model/sim_instant.h>

#include <algorithm>
#include <iostream>

//! Step a process until its next update time is after `stopTime`.
/*!
 *  @param[in] process
 *    The process to step
 *  @param[in] stopTime
 *    The time up to which (and including which) we want to step tasks.
 *  @return
 *    The time at which the next task after `stopTime` will occur.
 */
static SimInstant stepProcessUpTo(SysProcess &process, SimInstant stopTime) {
    while (true) {
        auto nextTaskTime = SimInstant::atNanos(process.getNextTaskTime()).atPriority(process.processPriority);

        if (nextTaskTime.realNanos == SimInstant::endOfTime().realNanos) { return nextTaskTime; }
        if (stopTime < nextTaskTime) { return nextTaskTime; }

        process.singleStepNextTask(stopTime.realNanos);
    }
}

void SimModel::stepUntilStop(uint64_t SimStopTime, int64_t stopPri) {
    //! @todo Remove this misplaced flush on stdout.
    std::cout << std::flush;

    // Keep single-stepping until the next process would occur after the stop time.
    auto simStopTime = SimInstant::atNanos(SimStopTime).atPriority(stopPri);
    while (true) {
        auto nextTaskTime = SimInstant::atNanos(this->NextTaskTime).atPriority(this->nextProcPriority);

        if (nextTaskTime > simStopTime) { break; }

        // If we're not stopping at this time, run processes of *all* priorities.
        int64_t inPri = (SimStopTime == this->NextTaskTime) ? stopPri : SimInstant::endOfTime().causalPriority;
        this->singleStepProcesses(inPri);
    }
}

SysProcess &SimModel::addNewProcess(std::string name, int64_t priority) {

    // Find the index separating lower priorities from higher priorities.
    auto it = this->processList.begin();
    for (; it != this->processList.end(); ++it) {
        if (priority > (*it)->processPriority) { break; }
    }

    it = this->processList.emplace(it, std::make_unique<SysProcess>(name));
    (*it)->processPriority = priority;

    return *it->get();
}

void SimModel::singleStepProcesses(int64_t const stopPri) {
    // Advance the simulation clock to the time of the next task.
    this->CurrentNanos = this->NextTaskTime;

    // Step all processes up to the desired stop instant.
    // Keep track of the earliest resumption time *following* the stop instant.
    auto stopTime = SimInstant::atNanos(this->CurrentNanos).atPriority(stopPri);
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &localProc : this->processList) {
        if (!localProc->isEnabled()) { continue; }

        nextTaskTime = std::min(nextTaskTime, stepProcessUpTo(*localProc, stopTime));
    }

    // Record the next earliest resumption time. Note that `nextTaskTime` should
    // only be `endOfTime` if no process was enabled.
    if (nextTaskTime != SimInstant::endOfTime()) {
        this->NextTaskTime = nextTaskTime.realNanos;
        this->nextProcPriority = nextTaskTime.causalPriority;
    }
}

void SimModel::resetSimulation() {
    // Reset all processes
    //! @todo Should we skip resetting disabled processes?
    this->CurrentNanos = 0;
    for (auto &process : this->processList) { process->reset(this->CurrentNanos); }

    // Figure out which process will update first
    auto nextTaskTime = SimInstant::endOfTime();
    for (auto &process : this->processList) {
        if (!process->isEnabled()) { continue; }

        auto nextProcTime = SimInstant::atNanos(process->getNextTaskTime()).atPriority(process->processPriority);
        nextTaskTime = std::min(nextTaskTime, nextProcTime);
    }
    this->NextTaskTime = nextTaskTime.realNanos;
    this->nextProcPriority = nextTaskTime.causalPriority;
}
