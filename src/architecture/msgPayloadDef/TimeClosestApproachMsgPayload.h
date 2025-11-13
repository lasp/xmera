#ifndef TCA_MESSAGE_H
#define TCA_MESSAGE_H

/*! @brief structure for on-board time of closest approach during a flyby */
typedef struct
    //@cond DOXYGEN_IGNORE
    TimeClosestApproachMsgPayload
//@endcond
{
    double timeTag;              //!< [s] Current time of validity for output
    double timeClosestApproach;  //!< [s] predicted time of closest approach in spacecraft time
    double standardDeviation;    //!< [s] time of closest approach standard deviation

} TimeClosestApproachMsgPayload;

#endif /* TCA_MESSAGE_H */
