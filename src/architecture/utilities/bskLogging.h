// SPDX-License-Identifier: ISC
// Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef _BSK_LOG_
#define _BSK_LOG_

#include "xmera_core_export.h"

// maximum length of info to log in a reference to BSKLogging in C, not relevant in C++
#define MAX_LOGGING_LENGTH 255

typedef enum {
    BSK_DEBUG,
    BSK_INFORMATION,
    BSK_WARNING,
    BSK_ERROR,
    BSK_SILENT  // the coder should never use this flag when using bskLog().  It is used to turn off all output
} logLevel_t;

XMERA_CORE_EXPORT void setDefaultLogLevel(logLevel_t logLevel);
XMERA_CORE_EXPORT logLevel_t getDefaultLogLevel();
XMERA_CORE_EXPORT void printDefaultLogLevel();

/// \cond DO_NOT_DOCUMENT

#ifdef __cplusplus
#include <map>
#include <string>

// Helper function to get log level map (avoids static member export issues)
inline const std::map<int, const char*>& getLogLevelMap() {
    static const std::map<int, const char*> logLevelMap = {
        {0, "BSK_DEBUG"},
        {1, "\033[92mBSK_INFORMATION\033[0m"},
        {2, "\033[93mBSK_WARNING\033[0m"},
        {3, "\033[91mBSK_ERROR\033[0m"},
        {4, "BSK_SILENT"}
    };
    return logLevelMap;
}

/*! BSK logging class */
class XMERA_CORE_EXPORT BSKLogger final {
   public:
    BSKLogger();
    BSKLogger(logLevel_t logLevel);
    void setLogLevel(logLevel_t logLevel);
    void printLogLevel();
    int getLogLevel();
    void bskLog(logLevel_t targetLevel, const char* info, ...);

   private:
    logLevel_t _logLevel;
};

#else
typedef struct BSKLogger BSKLogger;
#endif

#ifdef __cplusplus
#define EXTERN extern "C"
#else
#define EXTERN
#endif

EXTERN BSKLogger* _BSKLogger(void);
EXTERN void _BSKLogger_d(BSKLogger*);
EXTERN void _printLogLevel(BSKLogger*);
EXTERN void _setLogLevel(BSKLogger*, logLevel_t);
EXTERN void _bskLog(BSKLogger*, logLevel_t, const char*);

/// \endcond

#endif
