// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef XMERA_LOGGING_H
#define XMERA_LOGGING_H

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <string_view>

/*! @brief Named spdlog loggers that share one sink and one level configuration.

    A module gets its logger by name, usually its modelTag. A module that does not log has no logger.
    Use only these functions to get loggers. Do not use the spdlog registry, because each shared library
    can have its own copy of that registry.
 */
namespace xmera::logging {
    /*! Get the logger with this name. Make it on the shared sink at the configured level if it does not exist. */
    std::shared_ptr<spdlog::logger> get(std::string const &name);

    /*! Set the level for all loggers that do not have a level set by name. */
    void setLevel(spdlog::level::level_enum level);

    /*! Set the level for the logger with this name. The level applies also if the logger does not exist yet. */
    void setLevel(std::string const &name, spdlog::level::level_enum level);

    /*! Return true if at least one logger writes messages at this level. */
    bool enabled(spdlog::level::level_enum level);

    /*! Send the messages of all loggers to this sink. */
    void setSink(spdlog::sink_ptr sink);

    /*! Convert a level name (trace, debug, info, warning, error, critical, off) to a level.
        @throw std::invalid_argument if the name is not a level name.
     */
    spdlog::level::level_enum parseLevel(std::string_view name);

    /*! Make a sink from a specification: "stdout", "stderr", "null", or "file:<path>".
        @throw std::invalid_argument if the specification is not known.
     */
    spdlog::sink_ptr parseSink(std::string_view spec);
}  // namespace xmera::logging

#endif
