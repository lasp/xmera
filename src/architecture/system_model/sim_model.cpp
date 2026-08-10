// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "sim_model.h"

#include <algorithm>

void SimModel::stepUntilStop(uint64_t stopNanos, int64_t stopPriority) {
    if (this->jobHeap.empty()) { return; }

    // If reprioritizations or other heap-invalidating modifications have occurred, re-heap the heap.
    // Delaying heapification to this point allows us to amortize the impact of a series of independent,
    // uncorrelated modifications by re-heaping just once before we require the heap property again.
    if (!this->isHeap) {
        std::make_heap(this->jobHeap.begin(), this->jobHeap.end());
        this->isHeap = true;
    }

    auto stopTime =
        (stopNanos == std::numeric_limits<uint64_t>::max())
            ? SimInstant::atNanos(std::numeric_limits<uint64_t>::max() - 1).atPriority(std::numeric_limits<int64_t>::min())
            : SimInstant::atNanos(stopNanos).atPriority(stopPriority);

    while (this->jobHeap.front().nextProcessTime() <= stopTime) {
        // Extract the front element from the heap (moving it to the back).
        std::pop_heap(this->jobHeap.begin(), this->jobHeap.end());

        // Act on the soonest job (now at the back of the vector).
        {
            auto &job = this->jobHeap.back();

            // Advance the simulation clock to the time of this job.
            this->CurrentNanos = job.task->nextUpdateNanos;
            if (this->CurrentNanos == SimInstant::endOfTime().realNanos) { return; }

            // Re-schedule the job in the future (using saturating addition).
            job.task->nextUpdateNanos += job.task->updatePeriodNanos;
            if (job.task->nextUpdateNanos < job.task->updatePeriodNanos) {
                job.task->nextUpdateNanos = SimInstant::endOfTime().realNanos;
            }

            // Execute the job.
            if (job.process->enabled && job.task->taskActive) {
                for (auto &modelPair : job.task->TaskModels) {
                    modelPair.ModelPtr->updateState(this->CurrentNanos);
                }
            }
        }

        // Insert the rescheduled job back into the heap.
        std::push_heap(this->jobHeap.begin(), this->jobHeap.end());
    }
}

SysProcess &SimModel::addNewProcess(std::string name, int64_t priority) {
    // Find the index separating lower priorities from higher priorities.
    auto it = this->processList.begin();
    for (; it != this->processList.end(); ++it) {
        if (priority > (*it)->processPriority) { break; }
    }

    it = this->processList.emplace(it, std::make_unique<SysProcess>(SysProcess::Passkey{}, name, priority));

    return *it->get();
}

void SimModel::resetSimulation() {
    this->CurrentNanos = 0;

    // Reset all processes, tasks, and modules.
    for (auto &process : this->processList) {
        for (auto &task : process->processTasks) {
            task->nextUpdateNanos = task->firstUpdateNanos;
            for (auto const &modelPair : task->TaskModels) {
                modelPair.ModelPtr->reset(this->CurrentNanos);
            }
        }
    }

    // Gather all simulation jobs.
    //! @todo Move this responsibility into `SysProcess::addTask`.
    this->jobHeap.clear();
    this->isHeap = false;
    size_t process_id = 0;
    size_t task_id = 0;
    for (auto const &process : this->processList) {
        for (auto const &task : process->processTasks) {
            this->jobHeap.push_back({
                .process = process.get(),
                .process_id = process_id,
                .task = task,
                .task_id = task_id,
            });

            task_id += 1;
        }

        process_id += 1;
    }
}
