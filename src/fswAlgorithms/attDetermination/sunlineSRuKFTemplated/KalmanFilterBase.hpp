#ifndef BASILISK_KALMANFILTER_H
#define BASILISK_KALMANFILTER_H

#include "IMeasurement.hpp"
#include "State.hpp"

#include <architecture/utilities/macroDefinitions.h>

#include <array>

template<int StateDim, typename Derived>
class KalmanFilterBase {
public:
    using StateType = State<StateDim>;
    double previousFilterTimeTag = 0; //!< [s]  Time tag for state covar/etc
    std::function<StateType(double time, const StateType&)>& fx;

    void updateState(const uint64_t currentSimNanos,
                    std::array<std::unique_ptr<IMeasurement<StateDim>>, 5>& measurements) {
//        this->customInitializeUpdate();
        /*! Sort the vector in chronological order */
        this->orderMeasurementsChronologically(measurements);
        /*! Loop through all of the measurements assuming they are in chronological order by first testing if a value
         * has been populated in the measurements array*/
        for (const auto& measurement : measurements) {
//        for (int index = 0; index < MAX_MEASUREMENT_NUMBER; ++index) {
//            auto measurement = MeasurementModel();
//            if (!this->measurements[index].has_value()) {
//                continue;
//            } else {
//                measurement = this->measurements[index].value();
//            }
            /*! - If the time tag from a valid measurement is new compared to previous step,
            propagate and update the filter*/
            if (measurement->getTimeTag() >= this->previousFilterTimeTag && measurement->getValidity()) {
                /*! - time update to the measurement time and compute pre-fit residuals*/
//                this->timeUpdate(measurement.getTimeTag());
                static_cast<Derived*>(this)->predictImpl(measurement->getTimeTag());
//                measurement.setPreFitResiduals(this->computeResiduals(measurement));
                /*! - measurement update and compute post-fit residuals  */
//                this->measurementUpdate(measurement);
                static_cast<Derived*>(this)->updateImpl(measurement.get());
//                measurement.setPostFitResiduals(measurement.getObservation() - measurement.model(this->state));
//                this->measurements[index] = measurement;
            }
        }
        /*! - If current clock time is further ahead than the last measurement time, then
        propagate to this current time-step*/
        if ((double)currentSimNanos * NANO2SEC > this->previousFilterTimeTag) {
            static_cast<Derived*>(this)->predictImpl((double)currentSimNanos * NANO2SEC);
//            this->timeUpdate((double)currentSimNanos * NANO2SEC);
        }
//        this->customFinalizeUpdate();
//        this->writeOutputMessages(currentSimNanos);
    }
    // Delegates to derived implementation
//    void update(const std::vector<std::unique_ptr<IMeasurement<StateDim>>>& measurements) {
//        static_cast<Derived*>(this)->updateImpl(measurements);
//    }

//    void predict() {
//        static_cast<Derived*>(this)->predictImpl();
//    }

    const StateType& getState() const {
        return static_cast<const Derived*>(this)->getStateImpl();
    }

    void setState(const StateType& s) {
        static_cast<Derived*>(this)->setStateImpl(s);
    }

    void setDynamicsModel(std::function<StateType(double time, const StateType& state)> f) {
        fx = std::move(f);
    }

    /*!- Order the measurements chronologically (standard sort)
     @return void
     */
    void orderMeasurementsChronologically(std::array<std::unique_ptr<IMeasurement<StateDim>>, 5>& measurements) {
        std::sort(measurements.begin(), measurements.end(),
              [](const auto& a, const auto& b) {
                  return a->getTimeTag() < b->getTimeTag();
              });
    }
};



#endif //BASILISK_KALMANFILTER_H
