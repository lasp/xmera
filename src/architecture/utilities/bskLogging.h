#ifndef _BSK_LOG_
#define _BSK_LOG_


//maximum length of info to log in a reference to BSKLogging in C, not relevant in C++
#define MAX_LOGGING_LENGTH 255

typedef enum {
    BSK_DEBUG,
    BSK_INFORMATION,
    BSK_WARNING,
    BSK_ERROR,
    BSK_SILENT          // the coder should never use this flag when using bskLog().  It is used to turn off all output
} logLevel_t;

extern logLevel_t LogLevel;
void printDefaultLogLevel();

/// \cond DO_NOT_DOCUMENT

#ifdef __cplusplus
#include <map>
#include <string>

void setDefaultLogLevel(logLevel_t logLevel);
logLevel_t getDefaultLogLevel();

/*! BSK logging class */
class BSKLogger final
{
    public:
        BSKLogger();
        BSKLogger(logLevel_t logLevel);
        void setLogLevel(logLevel_t logLevel);
        void printLogLevel();
        int getLogLevel();
        void bskLog(logLevel_t targetLevel, const char* info, ...);

    //Provides a mapping from log level enum to str
    public:
        static std::map<int, const char*> logLevelMap;

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
