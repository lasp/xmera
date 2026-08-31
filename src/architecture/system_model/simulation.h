// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMAheader_simulation
#define XMAheader_simulation

#include <architecture/_GeneralModuleFiles/sys_model.h>

#include <stdint.h>

#include <vector>

namespace xmera::simulation {
    //! An identifier for a task group in a `simulation`
    /** Any `group_id` should only be used with the `simulation` it came from. */
    struct group_id { size_t id; };

    //! An identifier for a task in a `simulation`
    /** Any `task_id` should only be used with the `simulation` it came from. */
    struct task_id { size_t id; };

    //! The default priority used by tasks and task groups if none is otherwise indicated
    //
    // The choice of `-1` is historical, but changing it would break existing assumptions
    // about the relative order of explicitly-prioritized vs. implicitly-prioritized tasks.
    static inline constexpr int64_t DEFAULT_PRIORITY = -1;

    //! The identifier for the default group into which tasks are placed if none is otherwise indicated
    static inline constexpr group_id DEFAULT_GROUP = {0};

    //! The lowest possible task (or task group) priority
    static inline constexpr int64_t MIN_PRIORITY = std::numeric_limits<int64_t>::min();

    //! The latest possible simulation time
    /**
     * Tasks will only be updated up to `END_OF_TIME - 1`, so `END_OF_TIME` is a sentinel indicating
     * that simulation can proceed no further.
     */
    static inline constexpr uint64_t END_OF_TIME = std::numeric_limits<uint64_t>::max();

    //! A description of a task to be added to a simulation
    struct task_description {
        //! When the task should first be updated
        uint64_t first_update_nanos = 0;
        //! After the first update, the period on which the task should continue to be updated
        uint64_t update_period_nanos = 100;

        //! The task group to which this task belongs
        group_id group = DEFAULT_GROUP;
        //! The priority this task holds among others in its group
        int64_t priority = DEFAULT_PRIORITY;

        //! The sequence of steps to be performed by this task
        std::vector<SysModel*> steps;

        //! Create a task_description from a sequence of steps, with all other parameters defaulted
        task_description(std::vector<SysModel*> &&steps) : steps(std::move(steps)) {}
    };

    //! A collection of concurrent tasks to be updated on independent schedules
    /*!
     *  An instance of `simulation` (a "process") is a collection of prioritized tasks, each
     *  with its own update cadence. Tasks can be further organized into prioritized groups.
     *  If two tasks update at the same time, ties are broken in order of:
     *
     *  - Group priority (higher is earlier)
     *  - Group creation order (older is earlier)
     *  - Task priority (higher is earlier)
     *  - Task creation order (older is earlier)
     *
     *  All tasks reset at time zero, so the priority hierarchy also dictates reset order.
     */
    class simulation {
    public:
        //! Add a new prioritized task group
        /*!
         *  The priority of the group dictates the order in which its tasks are reset and
         *  updated among those of other groups. Higher priorities go first; if two processes
         *  have the same priority, the one added earlier takes precedence.
         *
         *  @important
         *    This method must not be invoked during simulation; it must only be
         *    invoked before `reset()` occurs for any given simulation run.
         */
        group_id add_task_group(int64_t priority = DEFAULT_PRIORITY);

        //! Add a new task with the given scheduling and execution parameters
        task_id add_task(task_description &&parameters);

        //! Permit the tasks in this group to be updated at subsequent update times
        /**
         * If a task has been individually disabled, it will remain effectively disabled.
         */
        void enable(group_id group);

        //! Inhibit the tasks in this group from being updated at subsequent update times
        /**
         * Even if a task has been individually enabled, it will remain effectively disabled.
         */
        void disable(group_id group);

        //! Permit the task to be updated at subsequent update times
        /**
         * If the task's group has been collectively disabled, it will remain effectively disabled.
         */
        void enable(task_id task);

        //! Inhibit the task from being updated at subsequent update times
        /**
         * Even if the task's group has been individually enabled, it will remain effectively disabled.
         */
        void disable(task_id task);

        //! Change a task's update period and recompute its next update time
        /*!
         *  @param[in] task
         *    The name of the task to change the period of
         *  @param[in] update_period_nanos
         *    The new period between update times in nanoseconds
         */
        void set_update_period(task_id task, uint64_t update_period_nanos);

        //! Reset all tasks to the initial state before their first update
        void reset();

        //! Step the task with the soonest update time
        /**
         *  @important
         *    This method must only be invoked once `reset()` has been invoked
         *    at least once.
         */
        void step();

        //! Get the time of the next task to be updated
        uint64_t next_update() const;

        //! Get the group priority of the next task to be updated
        int64_t next_priority() const;

        //! Get the priority of the given group
        int64_t priority(group_id group) const;

        //! Get the priority of the given task
        int64_t priority(task_id task) const;

        //! Get the current update period of the given task
        uint64_t period(task_id task) const;

    private:
        //! A comparator for ordering tasks by reset priority
        struct reset_comparator;

        //! A comparator for ordering tasks by update time
        struct update_comparator;

        //! Re-heapify the `job_heap` if `is_heap` is false
        void ensure_heap() const;

    private:
        //! Per-task scheduling and execution information
        struct task_record {
            //! The sequence of steps to be performed by this task
            std::vector<SysModel*> steps;

            //! When the task should first be updated
            uint64_t first_update_nanos;
            //! When the task should next be updated
            uint64_t next_update_nanos;
            //! The period of time between successive updates
            uint64_t update_period_nanos;
            //! The task group to which this task belongs
            group_id group;
            //! The priority this task holds among others in its group
            int64_t priority;

            //! Whether subsequent updates are permitted (true) or inhibited (false)
            bool enabled;
        };

        //! Per-group scheduling information
        struct group_record {
            //! The priority this group holds amongst others
            int64_t priority;
            //! Whether subsequent updates for tasks in this group are permitted (true) or inhibited (false)
            bool enabled;
        };

        //! The priority and enabled status of each task group
        /** One task group (the "default" group) always exists. */
        std::vector<group_record> groups = {{.priority = DEFAULT_PRIORITY, .enabled = true}};

        //! The collection of tasks to be simulated
        std::vector<task_record> tasks = {};

        //! A prioritized heap of simulation jobs
        // This is `mutable` so that the heap invariant can remain broken until the last moment,
        // allowing re-heapification to occur during logically const queries.
        mutable std::vector<task_id> job_heap = {};

        //! Whether `job_heap` currently has the heap property
        mutable bool is_heap = true;
    };

    //! Step the simulation forward until the next update would occur after the given stop time
    /*!
     *  This function invokes `step()` repeatedly, stopping only once the given time has been reached.
     *
     *  @important
     *    This method must only be invoked once `reset()` has been invoked
     *    at least once.
     *
     *  @param[in] stop_nanos
     *    The latest time at which tasks should be updated
     *  @param[in] stop_priority
     *    The least group priority at which tasks should be updated
     */
    static inline void step_until(simulation &sim, uint64_t stop_nanos, int64_t stop_priority = DEFAULT_PRIORITY) {
        stop_nanos = std::min(stop_nanos, END_OF_TIME - 1);

        while (sim.next_update() < stop_nanos) { sim.step(); }
        while (sim.next_update() == stop_nanos && sim.next_priority() >= stop_priority) { sim.step(); }
    }

    //! Step the tasks sharing the earliest resumption time
    /*!
     *  Of all tasks in the simulation, one or more will have the earliest
     *  resumption time. (If two tasks resume at the same time, both are "earliest".)
     *  This method will allow those tasks to resume, but potentially only
     *  partially: only the tasks within groups whose priority is at least
     *  `stop_priority` will be allowed to resume.
     *
     *  @important
     *    This function must only be invoked once `reset()` has been invoked
     *    at least once.
     *
     *  @param[in] stop_priority
     *    The least group priority at which tasks should be updated
     */
    static inline void step_next_update(simulation &sim, int64_t stop_priority = DEFAULT_PRIORITY) {
        step_until(sim, sim.next_update(), stop_priority);
    }
}

#endif
