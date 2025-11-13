#ifndef MRP_ROTATION_ALGORITHM_H
#define MRP_ROTATION_ALGORITHM_H

#include <stdint.h>

#include <architecture/msgPayloadDef/AttRefMsgPayload.h>
#include <architecture/msgPayloadDef/AttStateMsgPayload.h>
#include <Eigen/Core>

/*! @brief MRP Rotation class */
class MrpRotationAlgorithm {
   public:
    void reset();
    AttRefMsgPayload update(uint64_t callTime, AttRefMsgPayload inputRef, AttStateMsgPayload attStates);

    void checkRasterCommands();
    void computeTimeStep(uint64_t callTime);
    AttRefMsgPayload computeMRPRotationReference(Eigen::Vector3d sigma_R0N,
                                                 Eigen::Vector3d omega_R0N_N,
                                                 Eigen::Vector3d domega_R0N_N);

    void setSigmaRR0(const Eigen::Vector3d& sigma);
    const Eigen::Vector3d getSigmaRR0() const;
    void setOmegaRR0(const Eigen::Vector3d& omega);
    const Eigen::Vector3d getOmegaRR0() const;
    void enableDynamicReference();
    const bool isDynamicReferenceEnabled() const;

   private:
    Eigen::Vector3d sigma_RR0{
        Eigen::Vector3d::Zero()};  //!< [-] current MRP attitude coordinate set with respect to the input reference
    Eigen::Vector3d omega_RR0_R{
        Eigen::Vector3d::Zero()};     //!< [rad/s] angular velocity vector relative to input reference
    Eigen::Vector3d cmdSet{};         //!< [] msg commanded initial MRP sigma_RR0 set with respect to input reference
    Eigen::Vector3d cmdRates{};       //!< [rad/s] msg commanded constant angular velocity vector omega_RR0_R
    Eigen::Vector3d priorCmdSet{};    //!< [] prior commanded MRP set
    Eigen::Vector3d priorCmdRates{};  //!< [rad/s] prior commanded angular velocity vector
    uint64_t priorTime{};             //!< [ns] last time the guidance module is called
    double dt{};                      //!< [s] integration time-step
    bool
        dynamicReferenceEnabled{};  //!< true if desired attitude input message is linked to provide a dynamic reference
};

#endif
