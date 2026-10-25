// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/logging.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace xmera::logging {
    namespace {
        constexpr char const* pattern = "%^[%l]%$ [%n] %v";

        struct State {
            std::mutex mutex;
            std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers;
            std::unordered_map<std::string, spdlog::level::level_enum> namedLevels;
            spdlog::level::level_enum globalLevel = spdlog::level::debug;
            std::shared_ptr<spdlog::sinks::dist_sink_mt> sink;

            State() : sink(std::make_shared<spdlog::sinks::dist_sink_mt>()) {
                auto stdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                stdoutSink->set_pattern(pattern);
                this->sink->add_sink(stdoutSink);
            }

            spdlog::level::level_enum levelFor(std::string const &name) const {
                auto it = this->namedLevels.find(name);
                return it == this->namedLevels.end() ? this->globalLevel : it->second;
            }
        };

        State &state() {
            static State instance;
            return instance;
        }

        // The lowest level that any logger writes. A call below it can return without a lookup.
        std::atomic<int> lowestLevel{spdlog::level::debug};

        void updateLowestLevel(State const &s) {
            int lowest = s.globalLevel;
            for (auto const &[name, level] : s.namedLevels) { lowest = std::min<int>(lowest, level); }
            lowestLevel.store(lowest, std::memory_order_relaxed);
        }
    }  // namespace

    std::shared_ptr<spdlog::logger> get(std::string const &name) {
        State &s = state();
        std::lock_guard lock(s.mutex);
        auto it = s.loggers.find(name);
        if (it != s.loggers.end()) { return it->second; }
        auto logger = std::make_shared<spdlog::logger>(name, s.sink);
        logger->set_level(s.levelFor(name));
        s.loggers.emplace(name, logger);
        return logger;
    }

    void setLevel(spdlog::level::level_enum level) {
        State &s = state();
        std::lock_guard lock(s.mutex);
        s.globalLevel = level;
        for (auto const &[name, logger] : s.loggers) { logger->set_level(s.levelFor(name)); }
        updateLowestLevel(s);
    }

    void setLevel(std::string const &name, spdlog::level::level_enum level) {
        State &s = state();
        std::lock_guard lock(s.mutex);
        s.namedLevels[name] = level;
        auto it = s.loggers.find(name);
        if (it != s.loggers.end()) { it->second->set_level(level); }
        updateLowestLevel(s);
    }

    bool enabled(spdlog::level::level_enum level) {
        return level >= lowestLevel.load(std::memory_order_relaxed);
    }

    void setSink(spdlog::sink_ptr sink) {
        sink->set_pattern(pattern);
        state().sink->set_sinks({std::move(sink)});
    }

    spdlog::level::level_enum parseLevel(std::string_view name) {
        static std::unordered_map<std::string_view, spdlog::level::level_enum> const levels{
            {"trace", spdlog::level::trace},
            {"debug", spdlog::level::debug},
            {"info", spdlog::level::info},
            {"warning", spdlog::level::warn},
            {"error", spdlog::level::err},
            {"critical", spdlog::level::critical},
            {"off", spdlog::level::off},
        };
        auto it = levels.find(name);
        if (it == levels.end()) {
            throw std::invalid_argument(
                "Unknown log level '" + std::string(name)
                + "'. Use trace, debug, info, warning, error, critical or off."
            );
        }
        return it->second;
    }

    spdlog::sink_ptr parseSink(std::string_view spec) {
        constexpr std::string_view filePrefix = "file:";
        if (spec == "stdout") { return std::make_shared<spdlog::sinks::stdout_color_sink_mt>(); }
        if (spec == "stderr") { return std::make_shared<spdlog::sinks::stderr_color_sink_mt>(); }
        if (spec == "null") { return std::make_shared<spdlog::sinks::null_sink_mt>(); }
        if (spec.starts_with(filePrefix) && spec.size() > filePrefix.size()) {
            return std::make_shared<spdlog::sinks::basic_file_sink_mt>(std::string(spec.substr(filePrefix.size())));
        }
        throw std::invalid_argument(
            "Unknown log sink '" + std::string(spec) + "'. Use stdout, stderr, null or file:<path>."
        );
    }
}  // namespace xmera::logging
