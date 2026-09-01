// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "simulation.h"

#include <algorithm>
#include <execution>

namespace xmera::simulation {
    struct simulation::reset_comparator {
        simulation const &sim;

        explicit reset_comparator(simulation const &sim) : sim(sim) {}

        bool operator()(task_id const x, task_id const y) {
            auto const &tx = this->sim.tasks.at(x.id);
            auto const &ty = this->sim.tasks.at(y.id);

            auto const &gx = this->sim.groups.at(tx.group.id);
            auto const &gy = this->sim.groups.at(ty.group.id);

            // Order lexicographically by (1) group priority, (2) group creation order,
            // (3) task priority, and (4) task creation order.
            return (gx.priority < gy.priority) ? false
                 : (gx.priority > gy.priority) ? true
                 : (tx.group.id < ty.group.id) ? true
                 : (tx.group.id > ty.group.id) ? false
                 : (tx.priority < ty.priority) ? false
                 : (tx.priority > ty.priority) ? true
                                               : (x.id < y.id);
        }
    };

    struct simulation::update_comparator {
        simulation const &sim;

        explicit update_comparator(simulation const &sim) : sim(sim) {}

        bool operator()(task_id const x, task_id const y) {
            auto const &tx = this->sim.tasks.at(x.id);
            auto const &ty = this->sim.tasks.at(y.id);

            auto const &gx = this->sim.groups.at(tx.group.id);
            auto const &gy = this->sim.groups.at(ty.group.id);

            // Order lexicographically by (1) next update time, (2) group priority,
            // (3) group creation order, (4) task priority, and (5) task creation order.
            return (tx.next_update_nanos < ty.next_update_nanos) ? false
                 : (tx.next_update_nanos > ty.next_update_nanos) ? true
                 : (gx.priority < gy.priority)                   ? true
                 : (gx.priority > gy.priority)                   ? false
                 : (tx.group.id < ty.group.id)                   ? false
                 : (tx.group.id > ty.group.id)                   ? true
                 : (tx.priority < ty.priority)                   ? true
                 : (tx.priority > ty.priority)                   ? false
                                                                 : (x.id > y.id);
        }
    };

    //! Truncate `time` to the nearest mutiple of `period`
    static inline uint64_t project_to_schedule(uint64_t current_time, uint64_t first_time, uint64_t period) {
        // Judge everything relative to the origin, `first_time`.
        current_time -= first_time;

        // Truncate `time` to the nearest mutiple of `period`.
        current_time = (current_time / period) * period;

        // Re-add the offset against `first_time`.
        current_time += first_time;

        return current_time;
    }

    void simulation::ensure_heap() const {
        if (this->is_heap) { return; }

        std::make_heap(this->job_heap.begin(), this->job_heap.end(), update_comparator(*this));
        this->is_heap = true;
    }

    group_id simulation::add_task_group(int64_t const priority) {
        group_id const group = {this->groups.size()};

        this->groups.emplace_back(group_record{.priority = priority, .enabled = true});

        return group;
    }

    task_id simulation::add_task(task_description &&parameters) {
        task_id const task = {this->tasks.size()};

        this->tasks.emplace_back(
            task_record{
                .steps = std::move(parameters.steps),
                .first_update_nanos = parameters.first_update_nanos,
                .next_update_nanos = parameters.first_update_nanos,
                .update_period_nanos = parameters.update_period_nanos,
                .group = parameters.group,
                .priority = parameters.priority,
                .enabled = true,
            }
        );

        this->job_heap.emplace_back(task);
        this->is_heap = false;

        return task;
    }

    void simulation::enable(group_id group) {
        this->groups.at(group.id).enabled = true;
    }

    void simulation::disable(group_id group) {
        this->groups.at(group.id).enabled = false;
    }

    void simulation::enable(task_id task) {
        this->tasks.at(task.id).enabled = true;
    }

    void simulation::disable(task_id task) {
        this->tasks.at(task.id).enabled = false;
    }

    void simulation::set_update_period(task_id task_id, uint64_t update_period_nanos) {
        auto &task = this->tasks.at(task_id.id);

        if (task.next_update_nanos > task.first_update_nanos) {
            task.next_update_nanos = update_period_nanos
                                   + project_to_schedule(
                                         task.next_update_nanos - task.update_period_nanos,
                                         task.first_update_nanos,
                                         update_period_nanos
                                   );
        }

        task.update_period_nanos = update_period_nanos;

        // Mark the heap for re-heaping according to the task's new update time.
        this->is_heap = false;
    }

    void simulation::reset() {
        std::sort(std::execution::par, this->job_heap.begin(), this->job_heap.end(), reset_comparator(*this));
        this->is_heap = false;

        for (auto task_id : this->job_heap) {
            auto &task = this->tasks.at(task_id.id);
            task.next_update_nanos = task.first_update_nanos;

            for (auto const &step : task.steps) { step->reset(0); }
        }
    }

    void simulation::step() {
        if (this->job_heap.empty()) { return; }

        // Extract the front element from the heap (moving it to the back).
        this->ensure_heap();
        std::pop_heap(this->job_heap.begin(), this->job_heap.end(), update_comparator(*this));
        auto &task = this->tasks.at(this->job_heap.back().id);
        auto &group = this->groups.at(task.group.id);

        // Re-schedule the job in the future (using saturating addition).
        auto current_update_nanos = task.next_update_nanos;
        task.next_update_nanos += task.update_period_nanos;
        if (task.next_update_nanos < task.update_period_nanos) { task.next_update_nanos = END_OF_TIME; }
        std::push_heap(this->job_heap.begin(), this->job_heap.end(), update_comparator(*this));

        // Execute the job.
        if (group.enabled && task.enabled) {
            for (auto step : task.steps) { step->updateState(current_update_nanos); }
        }
    }

    uint64_t simulation::next_update() const {
        if (this->job_heap.empty()) { return END_OF_TIME; }

        this->ensure_heap();
        return this->tasks.at(this->job_heap.front().id).next_update_nanos;
    }

    int64_t simulation::next_priority() const {
        if (this->job_heap.empty()) { return MIN_PRIORITY; }

        this->ensure_heap();
        return this->groups.at(this->tasks.at(this->job_heap.front().id).group.id).priority;
    }

    int64_t simulation::priority(group_id group) const {
        return this->groups.at(group.id).priority;
    }

    int64_t simulation::priority(task_id task) const {
        return this->tasks.at(task.id).priority;
    }

    uint64_t simulation::period(task_id task) const {
        return this->tasks.at(task.id).update_period_nanos;
    }
}
