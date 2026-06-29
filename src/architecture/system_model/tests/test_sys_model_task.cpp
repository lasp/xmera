// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/system_model/sys_model_task.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdexcept>

using testing::Eq;
using testing::IsFalse;
using testing::IsTrue;

namespace {
    //! A module that throws an exception if a caller violates its lifecycle expectations
    class LifecycleCheckModule : public SysModel {
    public:
        void reset(uint64_t initialSimNanos) override {
            this->hasBeenReset = true;
            this->lastUpdateTime = initialSimNanos;
        }

        void updateState(uint64_t nextSimNanos) override {
            if (!this->hasBeenReset) { throw std::domain_error("update() called before reset()"); }

            if (nextSimNanos < this->lastUpdateTime) { throw std::domain_error("non-monotonic update occurred"); }

            this->lastUpdateTime = nextSimNanos;
        }

    public:
        //! Whether this module has ever been reset
        bool hasBeenReset = false;

        //! The simulation time at which the simulation was last updated or reset
        /*! If `hasBeenReset` is false, this value should be treated as undefined. */
        uint64_t lastUpdateTime = 0;
    };
}

TEST(LifecycleCheckModuleTest, normalLifecycleSucceeds) {
    RecordProperty("description", "The LifecycleCheck module does not throw under normal use");

    auto module = LifecycleCheckModule{};

    auto currentSimNanos = 123;

    // Initially, the module should believe it has not yet been reset.
    EXPECT_THAT(module.hasBeenReset, IsFalse());

    // After resetting, the module should believe it has been reset,
    // and should remember the time of reset.
    ASSERT_NO_THROW(module.reset(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));

    // The module should then be able to update, even if the update occurs
    // at the same time as the prior reset.
    ASSERT_NO_THROW(module.updateState(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));

    currentSimNanos += 200;

    // Updating at a later point in time poses no issue.
    ASSERT_NO_THROW(module.updateState(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));

    // Updating twice at the same time is fine too.
    ASSERT_NO_THROW(module.updateState(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));

    currentSimNanos = 1;

    // We may reset to an earlier time.
    ASSERT_NO_THROW(module.reset(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));

    currentSimNanos += 1;

    // An update may occur at a time less than a previous update,
    // so long as there is a reset in between.
    ASSERT_NO_THROW(module.updateState(currentSimNanos));
    EXPECT_THAT(module.hasBeenReset, IsTrue());
    EXPECT_THAT(module.lastUpdateTime, Eq(currentSimNanos));
}

TEST(LifecycleCheckModuleTest, nonmonotonicUpdateFails) {
    RecordProperty(
        "description",
        ("Performing an update before performing reset, or with an earlier time than"
         " the previous update or reset, is a lifecycle violation")
    );

    auto module = LifecycleCheckModule{};

    EXPECT_ANY_THROW(module.updateState(0));
    module.reset(10);
    EXPECT_ANY_THROW(module.updateState(0));
    module.updateState(15);
    EXPECT_ANY_THROW(module.updateState(0));
}

TEST(SysModelTaskTest, normalLifecycleSucceeds) {
    RecordProperty("description", "Normal usage of a task upholds the lifecycle contract of its modules");

    auto module = LifecycleCheckModule{};

    auto task = SysModelTask(100, 0);
    task.addModel(&module);

    // Run the task through some number of simulations at its scheduled cadence,
    // with each simulation starting at a different time.
    for (auto initialTime = 0; initialTime < 30; initialTime += 10) {
        // The initial time should be passed directly to the module.
        task.resetModels(initialTime);
        EXPECT_THAT(module.hasBeenReset, IsTrue());
        EXPECT_THAT(module.lastUpdateTime, Eq(initialTime));

        for (auto i = 0; i < 10; ++i) {
            auto nextUpdateTime = task.getNextStartTime();

            // The update time should be passed directly to the module.
            task.executeModels(nextUpdateTime);
            EXPECT_THAT(module.hasBeenReset, IsTrue());
            EXPECT_THAT(module.lastUpdateTime, Eq(nextUpdateTime));
        }
    }
}
