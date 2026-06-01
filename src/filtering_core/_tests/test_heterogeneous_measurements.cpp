// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

// Demonstrates and pins the heterogeneous-measurement pattern: a single filter
// consuming more than one *kind* of measurement on one timeline.
//
// The filtering_core pieces are kind-agnostic — measurement_queue,
// apply_sequential, and the SequentialFilter concept all operate over an
// arbitrary `Measurement` type, and srukf::update<M> is templated per
// measurement model (each model carries its own M::size). A closed tagged
// union over the input PODs is just such a `Measurement` type, so it flows
// through the existing core with no change: the filter that knows its kinds
// names the union and visits it to a per-kind overload.
//
// The deployment target is a freestanding C++ environment without <variant>,
// so the closed set is a hand-rolled tagged union (enum tag + union + a visit()
// switch) rather than std::variant — no heap, no exceptions, fixed-size, and
// trivially copyable (so the union needs no manual destructor).
//
// This toy filter is the copy-paste reference for a future multi-sensor filter
// (e.g. sunline = gyro + CSS). It is a real SRUKF (not a stub) and exercises
// two measurement kinds of *different observation sizes* (3 and 1).

#include <filtering_core/concepts.hpp>
#include <filtering_core/kalman_filter.hpp>
#include <filtering_core/srukf_interface.hpp>
#include <filtering_core/state.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <new>
#include <vector>

namespace filtering::hetero_test {
namespace {

using State   = filtering::StateVector<filtering::Position<3>, filtering::Velocity<3>>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector1 = Eigen::Matrix<double, 1, 1>;

// ---- Input PODs (what a host marshals from messages) ----------------------

// Kind A: a direct 3-vector position fix.
struct PositionFix {
    double          timeTag = 0;
    Eigen::Vector3d r       = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covar   = Eigen::Matrix3d::Identity();
};

// Kind B: a scalar speed measurement (|v|).
struct SpeedMeas {
    double timeTag = 0;
    double speed   = 0;
    double variance = 1;
};

// Closed set of kinds this filter consumes, on one shared timeline. A
// hand-rolled tagged union stands in for std::variant (unavailable in the
// freestanding target): enum tag + union + a visit() that switches on the tag.
// No heap, no exceptions, fixed-size (the storage is the largest alternative).
//
// The alternatives are trivially destructible (Eigen fixed-size types are),
// so overwriting the active member needs no destructor call. They are not
// trivially copy-*assignable*, though — Eigen defines operator= — which deletes
// the union's implicit copy/assign, so we provide them explicitly: copy the
// tag and placement-construct the active member.
struct Measurement {
    enum class Kind { PositionFix, SpeedMeas } kind;
    union {
        PositionFix positionFix;
        SpeedMeas   speedMeas;
    };

    Measurement(PositionFix m) : kind(Kind::PositionFix), positionFix(m) {}
    Measurement(SpeedMeas m) : kind(Kind::SpeedMeas), speedMeas(m) {}

    Measurement(Measurement const& other) { this->constructFrom(other); }
    Measurement& operator=(Measurement const& other) {
        if (this != &other) {
            this->constructFrom(other);
        }
        return *this;
    }

    template<class F>
    void visit(F&& f) const {
        switch (this->kind) {
            case Kind::PositionFix: f(this->positionFix); break;
            case Kind::SpeedMeas:   f(this->speedMeas);   break;
        }
    }

private:
    void constructFrom(Measurement const& other) {
        this->kind = other.kind;
        switch (other.kind) {
            case Kind::PositionFix: ::new (&this->positionFix) PositionFix(other.positionFix); break;
            case Kind::SpeedMeas:   ::new (&this->speedMeas) SpeedMeas(other.speedMeas);       break;
        }
    }
};

// ---- Measurement models (satisfy Measurement<M, State>) -------------------

struct PositionModel {
    static constexpr int size = 3;
    Eigen::Vector3d observed = Eigen::Vector3d::Zero();
    Eigen::Matrix3d measNoise = Eigen::Matrix3d::Identity();

    Eigen::Vector3d observation() const { return this->observed; }
    Eigen::Vector3d model(State const& s) const { return s.get<filtering::Position<3>>(); }
    Eigen::Matrix3d noise() const { return this->measNoise; }
    Eigen::Vector3d subtract(Eigen::Vector3d const& a, Eigen::Vector3d const& b) const {
        return a - b;
    }
};

struct SpeedModel {
    static constexpr int size = 1;
    Vector1 observed = Vector1::Zero();
    Vector1 measNoise = Vector1::Ones();

    Vector1 observation() const { return this->observed; }
    Vector1 model(State const& s) const {
        Vector1 v;
        v(0) = s.get<filtering::Velocity<3>>().norm();
        return v;
    }
    Eigen::Matrix<double, 1, 1> noise() const { return this->measNoise; }
    Vector1 subtract(Vector1 const& a, Vector1 const& b) const { return a - b; }
};

static_assert(filtering::Measurement<PositionModel, State>);
static_assert(filtering::Measurement<SpeedModel, State>);

// ---- Toy multi-sensor filter ----------------------------------------------
//
// Satisfies SequentialFilter<ToyFilter, Measurement>: timeUpdate(dt) +
// measurementUpdate(Measurement const&). The variant is unwrapped here, in the
// filter that knows its kinds; the core never sees it.
class ToyFilter {
public:
    void configure() {
        this->srukf.setAlpha(0.1);
        this->srukf.setBeta(2.0);
        Matrix6 p0 = Matrix6::Identity();
        p0.topLeftCorner<3, 3>() *= 100.0;   // 100 m^2 position
        p0.bottomRightCorner<3, 3>() *= 10.0;  // 10 (m/s)^2 velocity
        this->srukf.setInitialCovariance(p0);
        this->srukf.setProcessNoise(Matrix6::Identity() * 1e-6);

        // Constant-velocity dynamics.
        this->srukf.dynamics = [](double /*t*/, State const& s) -> State {
            State xDot;
            xDot.set<filtering::Position<3>>(s.get<filtering::Velocity<3>>());
            xDot.set<filtering::Velocity<3>>(Eigen::Vector3d::Zero());
            return xDot;
        };
    }

    void setInitialState(State const& s) { this->srukf.setInitialMean(s); }
    void reset() { this->srukf.reset(); }

    // ---- SequentialFilter<ToyFilter, Measurement> ----
    void timeUpdate(double dt) { this->srukf.timeUpdate(dt); }

    void measurementUpdate(Measurement const& m) {
        m.visit([this](auto const& meas) { this->applyMeasurement(meas); });
    }

    State   getMean() const { return this->srukf.getMean(); }
    Matrix6 getSqrtCovar() const { return this->srukf.getSqrtCovar(); }

    // Dispatch record: 0 = PositionFix, 1 = SpeedMeas, in the order applied.
    std::vector<int>    kindLog;
    std::vector<double> timeLog;

private:
    void applyMeasurement(PositionFix const& fix) {
        PositionModel model;
        model.observed  = fix.r;
        model.measNoise = fix.covar;
        this->srukf.update(model);
        this->kindLog.push_back(0);
        this->timeLog.push_back(fix.timeTag);
    }

    void applyMeasurement(SpeedMeas const& meas) {
        SpeedModel model;
        model.observed(0)  = meas.speed;
        model.measNoise(0) = meas.variance;
        this->srukf.update(model);
        this->kindLog.push_back(1);
        this->timeLog.push_back(meas.timeTag);
    }

    SrukfInterface<State> srukf;
};

static_assert(filtering::SequentialFilter<ToyFilter, Measurement>);

State makeState(Eigen::Vector3d const& r, Eigen::Vector3d const& v) {
    State s;
    s.set<filtering::Position<3>>(r);
    s.set<filtering::Velocity<3>>(v);
    return s;
}

double covarNorm(Matrix6 const& sqrtCovar) {
    return (sqrtCovar * sqrtCovar.transpose()).norm();
}

}  // namespace

// A variant of measurement kinds flows through the existing measurement_queue
// and apply_sequential unchanged. Both kinds are dispatched in ascending time
// order across the shared timeline, regardless of enqueue order, and the queue
// is empty on return.
TEST(HeterogeneousMeasurements, DispatchedInTimeOrderAcrossKinds) {
    ToyFilter filter;
    filter.configure();
    filter.setInitialState(makeState({0, 0, 0}, {1, 0, 0}));
    filter.reset();

    measurement_queue<Measurement, 8> queue;

    // Enqueue out of order and interleaved across kinds.
    SpeedMeas s1;   s1.timeTag = 2.5; s1.speed = 1.0; s1.variance = 0.01;
    PositionFix p1; p1.timeTag = 1.0; p1.r = {1, 0, 0}; p1.covar = Eigen::Matrix3d::Identity() * 0.01;
    PositionFix p2; p2.timeTag = 4.0; p2.r = {4, 0, 0}; p2.covar = Eigen::Matrix3d::Identity() * 0.01;

    queue.enqueue(s1.timeTag, Measurement{s1});
    queue.enqueue(p1.timeTag, Measurement{p1});
    queue.enqueue(p2.timeTag, Measurement{p2});

    apply_sequential(queue, filter, 0.0, 5.0);

    // Position@1.0, Speed@2.5, Position@4.0 — ascending time, mixed kinds.
    ASSERT_EQ(filter.kindLog.size(), 3u);
    EXPECT_EQ(filter.kindLog[0], 0);
    EXPECT_EQ(filter.kindLog[1], 1);
    EXPECT_EQ(filter.kindLog[2], 0);
    EXPECT_DOUBLE_EQ(filter.timeLog[0], 1.0);
    EXPECT_DOUBLE_EQ(filter.timeLog[1], 2.5);
    EXPECT_DOUBLE_EQ(filter.timeLog[2], 4.0);

    // apply_sequential drains the queue (same semantics for a variant element).
    EXPECT_TRUE(queue.isEmpty());
}

// Both kinds — observation sizes 3 (position) and 1 (speed) — run through the
// same srukf::update<M> and reduce the filter covariance. Proves heterogeneous
// observation dimensions need no special handling.
TEST(HeterogeneousMeasurements, BothObservationSizesReduceCovariance) {
    ToyFilter filter;
    filter.configure();
    Eigen::Vector3d const r0{0, 0, 0};
    Eigen::Vector3d const v0{2, 0, 0};
    filter.setInitialState(makeState(r0, v0));
    filter.reset();

    double const covar0 = covarNorm(filter.getSqrtCovar());

    measurement_queue<Measurement, 16> queue;
    // Feed consistent measurements of the constant-velocity truth.
    for (int i = 1; i <= 6; ++i) {
        double const t = static_cast<double>(i);
        Eigen::Vector3d const rTrue = r0 + v0 * t;
        if (i % 2 == 1) {
            PositionFix p;
            p.timeTag = t;
            p.r = rTrue;
            p.covar = Eigen::Matrix3d::Identity() * 0.04;
            queue.enqueue(t, Measurement{p});
        } else {
            SpeedMeas s;
            s.timeTag = t;
            s.speed = v0.norm();
            s.variance = 0.04;
            queue.enqueue(t, Measurement{s});
        }
    }

    apply_sequential(queue, filter, 0.0, 7.0);

    // Saw at least one of each kind.
    EXPECT_GT(filter.kindLog.size(), 0u);
    EXPECT_NE(std::find(filter.kindLog.begin(), filter.kindLog.end(), 0), filter.kindLog.end());
    EXPECT_NE(std::find(filter.kindLog.begin(), filter.kindLog.end(), 1), filter.kindLog.end());

    double const covarN = covarNorm(filter.getSqrtCovar());
    EXPECT_LT(covarN, covar0);  // measurements of both sizes informed the state
}

}  // namespace filtering::hetero_test
