// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/system_model/simulation.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using xmera::simulation::simulation;
using xmera::simulation::step_next_update;

TEST(simulation, taskTimings_withEmptySim) {
    RecordProperty(
        "description",
        ("On an empty simulation, the task timings should jump from 0 to the latest"
         " possible time.")
    );

    auto sim = simulation();

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Why? Well, the sim never has to update. UINT_MAX and INT_MIN are our best
    // sentinels for indicating that nothing ever happens.)
    sim.reset();
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    step_next_update(sim);
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));
}

TEST(simulation, taskTimings_withDisabledProcesses) {
    RecordProperty(
        "description",
        ("On a simulation with only disabled processes, the task timings should"
         " jump from 0 to the latest possible time.")
    );

    auto sim = simulation();
    auto group1 = sim.add_task_group(1);
    auto group2 = sim.add_task_group(2);

    sim.disable(group1);
    sim.disable(group2);

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Why? Well, the sim never has to update. UINT_MAX and INT_MIN are our best
    // sentinels for indicating that nothing ever happens.)
    sim.reset();
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    step_next_update(sim);
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));
}

TEST(simulation, taskTimings_withEmptyProcesses) {
    RecordProperty(
        "description",
        ("On a simulation with empty processes, the task timings should jump from"
         " 0 to the latest possible time.")
    );

    auto sim = simulation();
    auto group1 [[maybe_unused]] = sim.add_task_group(1);
    auto group2 [[maybe_unused]] = sim.add_task_group(2);

    // Immediately after a reset, the "next" task should occur "at the end of time".
    // (Nevertheless, the next priority is that of the highest-priority process.)
    sim.reset();
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));

    // After stepping, the "next" task should still occur "at the end of time".
    // (What else could it be?)
    step_next_update(sim);
    EXPECT_THAT(sim.next_update(), Eq(xmera::simulation::END_OF_TIME));
    EXPECT_THAT(sim.next_priority(), Eq(xmera::simulation::MIN_PRIORITY));
}
