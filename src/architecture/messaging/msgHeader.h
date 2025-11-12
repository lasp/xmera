#ifndef msgHeader_h
#define msgHeader_h

/*! @brief message system 2 header information structure */
typedef struct {
    int64_t isLinked;       //!< flag if the message has is connected to another message
    int64_t isWritten;      //!< flag if the message conntent has ever been written
    uint64_t timeWritten;   //!< [ns] time the message was written
    int64_t moduleID;       //!< ID of the module who wrote the message, negative value for Python module, non-negative for C/C++ modules
}MsgHeader;

#endif /* msgHeader_h */
