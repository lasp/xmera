// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sys_process.h"

#include <architecture/system_model/sim_instant.h>

#include <iostream>
#include <utility>

void SysProcess::reset(uint64_t initialSimNanos) {
    for (auto const &task : this->processTasks) { task.TaskPtr->resetModels(initialSimNanos); }

    /*! @todo `initialSimNanos` isn't necessarily the right reset value for
     *  `nextTaskTime`. Instead, we should query each task after reset to
     *  determine the earliest next update time.
     */
    this->nextTaskTime = initialSimNanos;
}

void SysProcess::reInitialize() {
    // Reset the next update time for every task in this process.
    for (auto const &task : this->processTasks) {
        SysModelTask* localTask = task.TaskPtr;
        localTask->reset();
    }

    // Reform the list of tasks to ensure that it is sorted by priority.
    // (This shouldn't really be necessary...)
    std::vector<ModelScheduleEntry> taskPtrs = this->processTasks;
    this->processTasks.clear();
    for (auto const &task : taskPtrs) { this->addTask(task.TaskPtr, task.taskPriority); }
}

std::vector<ModelScheduleEntry>::iterator SysProcess::getNextTask() {
    auto nextTaskIt = this->processTasks.begin();

    for (auto it = this->processTasks.begin(); it != this->processTasks.end(); it++) {
        auto currentUpdateTime = SimInstant::atNanos(it->NextTaskStart).atPriority(it->taskPriority);
        auto earliestUpdateTime = SimInstant::atNanos(nextTaskIt->NextTaskStart).atPriority(nextTaskIt->taskPriority);

        if (currentUpdateTime < earliestUpdateTime) { nextTaskIt = it; }
    }

    return nextTaskIt;
}

void SysProcess::singleStepNextTask(uint64_t nextSimNanos) {
    // Find the next soonest task to update
    auto nextTaskIt = this->getNextTask();
    if (nextTaskIt == this->processTasks.end()) {
        return;
    } else if (nextTaskIt->NextTaskStart > nextSimNanos) {
        // If the requested time does not meet our next start time, just return
        this->nextTaskTime = nextTaskIt->NextTaskStart;
        return;
    }

    // Update the next task, and record when it wants to be updated again
    SysModelTask* localTask = nextTaskIt->TaskPtr;
    localTask->executeModels(nextSimNanos);
    nextTaskIt->NextTaskStart = localTask->getNextStartTime();

    // Figure out when we are going to be called next for scheduling purposes
    this->nextTaskTime = this->getNextTask()->NextTaskStart;
}

void SysProcess::addTask(SysModelTask* task, int32_t priority) {
    this->scheduleTask({
        .NextTaskStart = task->getNextStartTime(),
        .TaskUpdatePeriod = task->getTaskPeriod(),
        .taskPriority = priority,
        .TaskPtr = task,
    });

    // It may be surprising to users that adding a task might re-enable
    // a process that was previously disabled...
    this->enable();
}

void SysProcess::scheduleTask(ModelScheduleEntry const &scheduleEntry) {
    SimInstant newTaskTime = {
        .realNanos = scheduleEntry.NextTaskStart,
        .causalPriority = scheduleEntry.taskPriority,
    };

    // Find the index separating earlier start times from later start times.
    auto it = this->processTasks.begin();
    for (; it != this->processTasks.end(); ++it) {
        SimInstant itTaskTime = {
            .realNanos = it->NextTaskStart,
            .causalPriority = it->taskPriority,
        };

        /*! @todo Why are we including `realNanos` in the comparison? This throws
         *  off the order in which modules are reset (which, morally, should be
         *  pure priority order), and causes lower-priority tasks to appear earlier
         *  in the list even if they only happen to *start* earlier in the simulation.
         *  We don't maintain next-resumption-order throughout the simulation,
         *  so why do we start with it that way?
         */
        if (newTaskTime < itTaskTime) { break; }
    }

    // Insert the module at this index. (It's okay if it's the end() iterator.)
    this->processTasks.insert(it, scheduleEntry);
}

bool SysProcess::changeTaskPeriod(std::string const &taskName, uint64_t newPeriod) {
    for (auto &entry : this->processTasks) {
        if (entry.TaskPtr->TaskName != taskName) { continue; }

        entry.TaskPtr->setPeriod(newPeriod);
        entry.NextTaskStart = entry.TaskPtr->getNextStartTime();
        entry.TaskUpdatePeriod = entry.TaskPtr->getTaskPeriod();

        return true;
    }

    return false;
}
