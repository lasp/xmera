#include "measurementModels.h"

/*! Function to represent measurement model which inputs a stateModel and outputs a matrix
 * @param FilterStateVector
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::model(const FilterStateVector& state) const { return this->measurementModel(state); }

/*! Set function to represent measurement model which inputs a stateModel and outputs a matrix
 * @param std::function<const Eigen::MatrixXd(const FilterStateVector&)>
 */
void MeasurementModel::setMeasurementModel(
    const std::function<const Eigen::MatrixXd(const FilterStateVector&)>& modelCalculator) {
    this->measurementModel = modelCalculator;
}

/*! Function to represent measurement matrix which inputs a stateModel and outputs a matrix (partial of measurement
 * @param FilterStateVector
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::computeMeasurementMatrix(const FilterStateVector& state) const {
    return this->measurementPartials(state);
}

/*! Set function to represent measurement matrix which inputs a stateModel and outputs a matrix (partial of measurement
 * model with respect to state)
 * @param std::function<const Eigen::MatrixXd(const FilterStateVector&)>
 */
void MeasurementModel::setMeasurementMatrix(
    const std::function<const Eigen::MatrixXd(const FilterStateVector&)>& hMatrixCalculator) {
    this->measurementPartials = hMatrixCalculator;
}

/*! Subtract measurements. By default this is a linear subtraction, but could be subMrp or
 * other functions depending on the measurement type
 * @param Eigen::VectorXd measurementObserved
 * @param Eigen::VectorXd measurementPredicted
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::subMeasurements(const Eigen::VectorXd& measurementObserved,
                                                  const Eigen::VectorXd& measurementPredicted) const {
    return this->measurementSubtraction(measurementObserved, measurementPredicted);
}

/*! Set function to add measurements. By default this add function is an R^n subtraction, but could be subMrp or
 * other functions depending on the measurement type
 * @param subFunction std::function<const Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>
 */
void MeasurementModel::setMeasurementSubtraction(
    const std::function<const Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& subFunction) {
    this->measurementSubtraction = subFunction;
}

/*! Return the size of the observation
   @return size_t
*/
size_t MeasurementModel::size() const { return this->getObservation().size(); }

/*! Get measurement name
 * @return std::string
 */
std::string MeasurementModel::getMeasurementName() const { return this->name; }

/*! Set measurement name
 * @param std::string
 */
void MeasurementModel::setMeasurementName(std::string_view measurementName) { this->name = measurementName; }

/*! Get measurement time tag
 * @return double
 */
double MeasurementModel::getTimeTag() const { return this->timeTag; }

/*! Set measurement time tag
 * @param double
 */
void MeasurementModel::setTimeTag(const double measurementTimeTag) { this->timeTag = measurementTimeTag; }

/*! Get measurement validity
 * @return Eigen::VectorXd
 */
bool MeasurementModel::getValidity() const { return this->validity; }

/*! Set measurement validity
 * @param bool
 */
void MeasurementModel::setValidity(const bool measurementValidity) { this->validity = measurementValidity; }

/*! Get measurement observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getObservation() const { return this->observation; }

/*! Set measurement observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setObservation(const Eigen::VectorXd& measurementObserved) {
    this->observation = measurementObserved;
}

/*! Get measurement noise
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd MeasurementModel::getMeasurementNoise() const { return this->noise; }

/*! Set measurement noise
 * @param Eigen::MatrixXd
 */
void MeasurementModel::setMeasurementNoise(const Eigen::MatrixXd& measurementNoise) { this->noise = measurementNoise; }

/*! Get post fit residuals of observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getPostFitResiduals() const { return this->postFitResiduals; }

/*! Set post fit residuals of observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setPostFitResiduals(const Eigen::VectorXd& measurementPostFit) {
    this->postFitResiduals = measurementPostFit;
}

/*! Get pre fit residuals of observation
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::getPreFitResiduals() const { return this->preFitResiduals; }

/*! Set pre fit residuals of observation
 * @param Eigen::VectorXd
 */
void MeasurementModel::setPreFitResiduals(const Eigen::VectorXd& measurementPreFit) {
    this->preFitResiduals = measurementPreFit;
}

/*! Measurement model that returns the position component of the state
 * @param FilterStateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::positionStates(const FilterStateVector& state) { return state.getPositionStates(); }

/*! Measurement model that returns the unit vector of the position state
 * @param FilterStateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::normalizedPositionStates(const FilterStateVector& state) {
    return state.getPositionStates() / state.getPositionStates().norm();
}

/*! Measurement model that returns the position component of the state, and performs a MRP shadow set check
 * @param FilterStateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::mrpStates(const FilterStateVector& state) {
    return mrpSwitch(Eigen::Vector3d(state.getPositionStates()), 1.0);
}

/*! Measurement model that returns the velocity component of the state
 * @param state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::velocityStates(const FilterStateVector& state) { return state.getVelocityStates(); }

/*! Measurement model that returns the velocity component of the state with Bias
 * @param FilterStateVector state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd MeasurementModel::velocityStatesWithBias(const FilterStateVector& state) {
    Eigen::VectorXd observation = state.getVelocityStates();
    if (state.hasBias()) {
        assert(observation.size() == state.getBiasStates().size());
        observation += state.getBiasStates();
    }
    return observation;
}

/*! Measurement model that takes the first 3 components of the state vector and interprets them as MRPs
 * @param state
 * @return Eigen::VectorXd
 */
Eigen::VectorXd mrpFirstThreeStates(Eigen::VectorXd state) { return mrpSwitch(state.head<3>().eval(), 1.0); }
