// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/logging.h"

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace {
    class Logging : public testing::Test {
    protected:
        void SetUp() override {
            xmera::logging::setSink(std::make_shared<spdlog::sinks::ostream_sink_mt>(this->output));
        }

        void TearDown() override {
            xmera::logging::resetLevels();
            xmera::logging::setSink(xmera::logging::parseSink("stdout"));
        }

        std::ostringstream output;
    };
}  // namespace

TEST_F(Logging, writesLevelAndLoggerName) {
    xmera::logging::get("nameModule")->error("message linked: {}", false);

    EXPECT_NE(this->output.str().find("[error] [nameModule] message linked: false"), std::string::npos);
}

TEST_F(Logging, getReturnsSameLoggerForSameName) {
    EXPECT_EQ(xmera::logging::get("sameModule"), xmera::logging::get("sameModule"));
}

TEST_F(Logging, filtersMessagesBelowGlobalLevel) {
    xmera::logging::setLevel(spdlog::level::warn);
    auto logger = xmera::logging::get("filterModule");

    logger->info("hidden");
    logger->warn("shown");

    EXPECT_EQ(this->output.str().find("hidden"), std::string::npos);
    EXPECT_NE(this->output.str().find("shown"), std::string::npos);
}

TEST_F(Logging, globalLevelAppliesToExistingAndLaterLoggers) {
    auto existing = xmera::logging::get("existingModule");
    xmera::logging::setLevel(spdlog::level::err);
    auto later = xmera::logging::get("laterModule");

    EXPECT_EQ(existing->level(), spdlog::level::err);
    EXPECT_EQ(later->level(), spdlog::level::err);
}

TEST_F(Logging, namedLevelAppliesBeforeLoggerExists) {
    xmera::logging::setLevel("earlyModule", spdlog::level::off);

    xmera::logging::get("earlyModule")->error("hidden");

    EXPECT_TRUE(this->output.str().empty());
}

TEST_F(Logging, namedLevelOverridesGlobalLevel) {
    xmera::logging::setLevel("quietModule", spdlog::level::off);
    xmera::logging::setLevel(spdlog::level::trace);

    EXPECT_EQ(xmera::logging::get("quietModule")->level(), spdlog::level::off);
}

TEST_F(Logging, resetLevelsRemovesNamedLevels) {
    xmera::logging::setLevel("resetModule", spdlog::level::off);
    xmera::logging::resetLevels();
    xmera::logging::setLevel(spdlog::level::warn);

    EXPECT_EQ(xmera::logging::get("resetModule")->level(), spdlog::level::warn);
}

TEST_F(Logging, enabledFollowsLowestLevel) {
    xmera::logging::setLevel(spdlog::level::err);
    EXPECT_FALSE(xmera::logging::enabled(spdlog::level::warn));

    xmera::logging::setLevel("verboseModule", spdlog::level::trace);
    EXPECT_TRUE(xmera::logging::enabled(spdlog::level::trace));
}

TEST_F(Logging, doesNotTruncateLongMessages) {
    std::string const longText(1'000, 'x');

    xmera::logging::get("longModule")->error("{}", longText);

    EXPECT_NE(this->output.str().find(longText), std::string::npos);
}

TEST(LoggingParse, parsesLevelNames) {
    EXPECT_EQ(xmera::logging::parseLevel("warning"), spdlog::level::warn);
    EXPECT_EQ(xmera::logging::parseLevel("off"), spdlog::level::off);
    EXPECT_THROW(xmera::logging::parseLevel("warn"), std::invalid_argument);
}

TEST(LoggingParse, parsesSinkSpecifications) {
    EXPECT_NE(xmera::logging::parseSink("null"), nullptr);
    EXPECT_THROW(xmera::logging::parseSink("file:"), std::invalid_argument);
    EXPECT_THROW(xmera::logging::parseSink("syslog"), std::invalid_argument);
}
