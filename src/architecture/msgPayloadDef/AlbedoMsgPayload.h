#ifndef albedoSimMsg_H
#define albedoSimMsg_H

/*! albedo message definition */
typedef struct {
    // Maximum albedo acting on the instrument
    // considering the instrument's position
    double albedoAtInstrumentMax;  //!< [-] Max albedo flux ratio at instrument
    double AfluxAtInstrumentMax;   //!< [W/m^2] Max albedo flux at instrument
    // Albedo acting on the instrument
    // considering the unit normal and fov of the instrument in addition to the position
    double albedoAtInstrument;  //!< [-] Albedo flux ratio at instrument
    double AfluxAtInstrument;   //!< [W/m^2] Albedo flux at instrument
} AlbedoMsgPayload;
#endif /* albedoSimMsg_h */
