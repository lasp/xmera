#include "ephemDifferenceWithUncertaintyAlgorithm.h"

/*! During an update, this module computes the difference between two ephemeris messages and its covariance.
 @return tuple of (navTransOutMsgBuffer, filterOutMsgBuffer)
 @param ephemBaseInBuffer  The ephemeris of the base celestial object
 @param ephemSecondaryInBuffer  The ephemeris of the secondary celestial object
 */
std::tuple<NavTransMsgPayload, FilterMsgPayload> EphemDifferenceWithUncertaintyAlgorithm::updateState(
    EphemerisMsgPayload ephemBaseInBuffer,
    EphemerisMsgPayload ephemSecondaryInBuffer) const {
    // take timeTag from secondary, as timeTag from primary/base may be constant due to stand-alone message
    double timeTag = ephemSecondaryInBuffer.timeTag;

    /*! - compute relative states */
    Eigen::Vector3d r_1_N = cArrayAsEigenVector(ephemBaseInBuffer.r_BdyZero_N);
    Eigen::Vector3d v_1_N = cArrayAsEigenVector(ephemBaseInBuffer.v_BdyZero_N);
    Eigen::Vector3d r_2_N = cArrayAsEigenVector(ephemSecondaryInBuffer.r_BdyZero_N);
    Eigen::Vector3d v_2_N = cArrayAsEigenVector(ephemSecondaryInBuffer.v_BdyZero_N);

    Eigen::Vector3d r_21_N = r_2_N - r_1_N;
    Eigen::Vector3d v_21_N = v_2_N - v_1_N;

    int numStates = 6;
    Eigen::VectorXd state_21_N(numStates);
    state_21_N << r_21_N, v_21_N;

    /*! - compute relative covariance matrix */
    Eigen::MatrixXd covar_21_N(numStates, numStates);
    covar_21_N = this->covarianceBase + this->covarianceSecondary;

    /*! - output messages */
    NavTransMsgPayload navTransOutMsgBuffer{};
    FilterMsgPayload filterOutMsgBuffer{};

    navTransOutMsgBuffer.timeTag = timeTag;
    eigenVectorToCArray(r_21_N, navTransOutMsgBuffer.r_BN_N);
    eigenVectorToCArray(v_21_N, navTransOutMsgBuffer.v_BN_N);

    filterOutMsgBuffer.numberOfStates = numStates;
    filterOutMsgBuffer.timeTag = timeTag;
    eigenMatrixXToCArray(state_21_N, filterOutMsgBuffer.state);
    eigenMatrixXToCArray(covar_21_N, filterOutMsgBuffer.covar);

    return std::make_tuple(navTransOutMsgBuffer, filterOutMsgBuffer);
}

/*! Set the state covariance of the base celestial object (e.g. asteroid)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertaintyAlgorithm::setCovarianceBase(const Eigen::MatrixXd stateCovariance) {
    this->covarianceBase.resize(stateCovariance.rows(), stateCovariance.cols());
    this->covarianceBase << stateCovariance;
}

/*! Get the state covariance of the base celestial object (e.g. asteroid)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertaintyAlgorithm::getCovarianceBase() const { return this->covarianceBase; }

/*! Set the state covariance of the secondary celestial object (e.g. spacecraft)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertaintyAlgorithm::setCovarianceSecondary(const Eigen::MatrixXd stateCovariance) {
    this->covarianceSecondary.resize(stateCovariance.rows(), stateCovariance.cols());
    this->covarianceSecondary << stateCovariance;
}

/*! Get the state covariance of the secondary celestial object (e.g. spacecraft)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertaintyAlgorithm::getCovarianceSecondary() const {
    return this->covarianceSecondary;
}
