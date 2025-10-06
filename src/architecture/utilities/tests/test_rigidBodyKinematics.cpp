/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include <gtest/gtest.h>

#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "architecture/utilities/tests/rbk_float_wrappers.h"
#include <Eigen/Dense>
#include <functional>
#include <random>

// Test-wide Helpers
using FloatingPointTypes = ::testing::Types<float, double>;

template <typename T>
T kinematicsAccuracy() {
    if constexpr (std::is_same_v<T, float>)
        return static_cast<T>(1e-4);
    else
        return static_cast<T>(1e-8);
}

template <typename T>
Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>> cArray33ToEigenMatrix33(const T (&array)[3][3]) {
    return Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>>(&array[0][0]);
}

template <typename T>
Eigen::Map<const Eigen::Matrix<T, 3, 4, Eigen::RowMajor>> cArray34ToEigenMatrix34(const T (&array)[3][4]) {
    return Eigen::Map<const Eigen::Matrix<T, 3, 4, Eigen::RowMajor>>(&array[0][0]);
}

template <typename T>
Eigen::Map<const Eigen::Matrix<T, 4, 3, Eigen::RowMajor>> cArray43ToEigenMatrix43(const T (&array)[4][3]) {
    return Eigen::Map<const Eigen::Matrix<T, 4, 3, Eigen::RowMajor>>(&array[0][0]);
}

template <typename Derived, typename T>
void eigenMatrix3ToCArray(const Eigen::MatrixBase<Derived>& mat, T out[3][3]) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) out[i][j] = mat(i, j);
}

// Addition and subtraction of kinematic representation tests
template <typename T>
struct AdditiveTestCase {
    std::string name;
    std::function<void(T*, T*, T*)> cFunc;
    std::function<Eigen::Matrix<T, 3, 1>(Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>)> cppFunc;
    std::function<Eigen::Matrix<T, 3, 3>(Eigen::Matrix<T, 3, 1>)> dcmFunc;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const AdditiveTestCase<T>& param) {
    os << param.name;
#if defined(__cpp_rtti)
    if constexpr (std::is_same_v<T, float>) {
        os << "_float";
    } else if constexpr (std::is_same_v<T, double>) {
        os << "_double";
    }
#endif
    return os;
}

template <typename T>
class AdditiveTest : public ::testing::TestWithParam<AdditiveTestCase<T>> {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-3.14, 3.14};
    const T accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }
};

template <typename T>
std::string GetTypeName();

template <>
std::string GetTypeName<float>() {
    return "float";
}

template <>
std::string GetTypeName<double>() {
    return "double";
}

template <typename T>
std::string MakeNameWithType(const AdditiveTestCase<T>& testCase) {
    std::string name;
    for (char c : testCase.name) {
        if (std::isalnum(c) || c == '_') {
            name += c;
        } else {
            name += '_';
        }
    }
    return name + "_" + GetTypeName<T>();
}

std::string AdditiveTestNameFloat(const ::testing::TestParamInfo<AdditiveTestCase<float>>& info) {
    return MakeNameWithType<float>(info.param);
}

std::string AdditiveTestNameDouble(const ::testing::TestParamInfo<AdditiveTestCase<double>>& info) {
    return MakeNameWithType<double>(info.param);
}

template <typename T>
void runAdditiveTest(const AdditiveTestCase<T>& param,
                     const Eigen::Matrix<T, 3, 1>& a,
                     const Eigen::Matrix<T, 3, 1>& b,
                     T accuracy) {
    Eigen::Matrix<T, 3, 1> expected;
    T expectedArray[3] = {};
    param.cFunc(const_cast<T*>(a.data()), const_cast<T*>(b.data()), expectedArray);
    expected = Eigen::Matrix<T, 3, 1>(expectedArray);

    Eigen::Matrix<T, 3, 1> result = param.cppFunc(a, b);
    Eigen::Matrix<T, 3, 3> dcm = param.dcmFunc(result);

    EXPECT_LT((dcm * dcm.transpose() - Eigen::Matrix<T, 3, 3>::Identity()).norm(), accuracy);
    EXPECT_LT((result - expected).norm(), accuracy);
}

class AdditiveFloatTest : public AdditiveTest<float> {};

TEST_P(AdditiveFloatTest, AdditiveProperties) {
    const auto& param = GetParam();
    Vec3 a = this->randVec3();
    Vec3 b = this->randVec3();
    runAdditiveTest<float>(param, a, b, accuracy);
}

class AdditiveDoubleTest : public AdditiveTest<double> {};

TEST_P(AdditiveDoubleTest, AdditiveProperties) {
    const auto& param = GetParam();
    Vec3 a = this->randVec3();
    Vec3 b = this->randVec3();
    runAdditiveTest<double>(param, a, b, accuracy);
}

INSTANTIATE_TEST_SUITE_P(
    FloatAddTests,
    AdditiveFloatTest,
    ::testing::Values(
        AdditiveTestCase<float>{"addMRP",
                                addMRP_float,
                                [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return addMrp<float>(a, b); },
                                [](const Eigen::Vector3f& a) { return mrpToDcm<float>(a); }},
        AdditiveTestCase<float>{"addPrvs",
                                addPRV_float,
                                [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return addPrv<float>(a, b); },
                                [](const Eigen::Vector3f& a) { return prvToDcm<float>(a); }},
        AdditiveTestCase<float>{
            "addEulerAngles",
            addEuler321_float,
            [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return addEulerAngles321<float>(a, b); },
            [](const Eigen::Vector3f& a) { return eulerAngles321ToDcm<float>(a); }},
        AdditiveTestCase<float>{"subMRPs",
                                subMRP_float,
                                [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return subMrp<float>(a, b); },
                                [](const Eigen::Vector3f& a) { return mrpToDcm<float>(a); }},
        AdditiveTestCase<float>{"subPrvs",
                                subPRV_float,
                                [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return subPrv<float>(a, b); },
                                [](const Eigen::Vector3f& a) { return prvToDcm<float>(a); }},
        AdditiveTestCase<float>{
            "subEulerAngles",
            subEuler321_float,
            [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) { return subEulerAngles321<float>(a, b); },
            [](const Eigen::Vector3f& a) { return eulerAngles321ToDcm<float>(a); }}),
    AdditiveTestNameFloat);

INSTANTIATE_TEST_SUITE_P(
    DoubleAddTests,
    AdditiveDoubleTest,
    ::testing::Values(AdditiveTestCase<double>{
                          "addMRP",
                          addMRP,
                          [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return addMrp<double>(a, b); },
                          [](const Eigen::Vector3d& a) { return mrpToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "addPrvs",
                          addPRV,
                          [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return addPrv<double>(a, b); },
                          [](const Eigen::Vector3d& a) { return prvToDcm<double>(a); }},
                      AdditiveTestCase<double>{"addEulerAngles",
                                               addEuler321,
                                               [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                                                   return addEulerAngles321<double>(a, b);
                                               },
                                               [](const Eigen::Vector3d& a) { return eulerAngles321ToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subMRPs",
                          subMRP,
                          [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return subMrp<double>(a, b); },
                          [](const Eigen::Vector3d& a) { return mrpToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subPrvs",
                          subPRV,
                          [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return subPrv<double>(a, b); },
                          [](const Eigen::Vector3d& a) { return prvToDcm<double>(a); }},
                      AdditiveTestCase<double>{
                          "subEulerAngles",
                          subEuler321,
                          [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                              return subEulerAngles321<double>(a, b);
                          },
                          [](const Eigen::Vector3d& a) { return eulerAngles321ToDcm<double>(a); }}),
    AdditiveTestNameDouble);

// Euler Parameters add/subtract tests
template <typename T>
struct EulerTestCase {
    std::string name;
    void (*cFunc)(T*, T*, T*);
    std::function<Eigen::Matrix<T, 4, 1>(Eigen::Matrix<T, 4, 1>, Eigen::Matrix<T, 4, 1>)> cppFunc;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const EulerTestCase<T>& param) {
    os << param.name;
    return os;
}

template <typename T>
class EulerParameterTest : public ::testing::TestWithParam<EulerTestCase<T>> {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec4 = Eigen::Matrix<T, 4, 1>;

    const T accuracy = kinematicsAccuracy<T>();

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> angleDistribution{-3.14, 3.14};

    Vec4 randomEulerParameter() {
        T phi = angleDistribution(generator);
        Vec3 axis(angleDistribution(generator), angleDistribution(generator), angleDistribution(generator));
        axis.normalize();
        return Vec4(
            std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));
    }

    T normConstraintViolation(const Vec4& ep) { return std::abs(ep.squaredNorm() - static_cast<T>(1)); }

    void runTest(const EulerTestCase<T>& param) {
        Vec4 ep1 = randomEulerParameter();
        Vec4 ep2 = randomEulerParameter();

        EXPECT_LT(normConstraintViolation(ep1), accuracy);
        EXPECT_LT(normConstraintViolation(ep2), accuracy);

        T expectedArray[4] = {};
        param.cFunc(ep1.data(), ep2.data(), expectedArray);
        Vec4 expected = Eigen::Map<Vec4>(expectedArray);

        Vec4 result = param.cppFunc(ep1, ep2);

        EXPECT_LT(normConstraintViolation(result), accuracy)
            << "Holonomic constraint violated for test: " << param.name;

        EXPECT_NEAR((result - expected).norm(), 0, accuracy) << "C/C++ mismatch for test: " << param.name;
    }
};

using FloatEulerTest = EulerParameterTest<float>;
using DoubleEulerTest = EulerParameterTest<double>;

std::vector<EulerTestCase<float>> getFloatEulerTests() {
    return {
        {"addEulerParameters", addEP_float, addEp<float>},
        {"subEulerParameters", subEP_float, subEp<float>},
    };
}

std::vector<EulerTestCase<double>> getDoubleEulerTests() {
    return {
        {"addEulerParameters", addEP, addEp<double>},
        {"subEulerParameters", subEP, subEp<double>},
    };
}

TEST_P(FloatEulerTest, HolonomicConstraintAndCReferenceMatch) { this->runTest(GetParam()); }

TEST_P(DoubleEulerTest, HolonomicConstraintAndCReferenceMatch) { this->runTest(GetParam()); }

std::string EulerTestNameFloat(const ::testing::TestParamInfo<EulerTestCase<float>>& info) {
    std::string name = info.param.name;
    for (char& c : name)
        if (!std::isalnum(c)) c = '_';
    return name + "_float";
}

std::string EulerTestNameDouble(const ::testing::TestParamInfo<EulerTestCase<double>>& info) {
    std::string name = info.param.name;
    for (char& c : name)
        if (!std::isalnum(c)) c = '_';
    return name + "_double";
}

INSTANTIATE_TEST_SUITE_P(FloatTests, FloatEulerTest, ::testing::ValuesIn(getFloatEulerTests()), EulerTestNameFloat);

INSTANTIATE_TEST_SUITE_P(DoubleTests, DoubleEulerTest, ::testing::ValuesIn(getDoubleEulerTests()), EulerTestNameDouble);

// Bmatrix tests
template <typename T>
class BMatrixTypedTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Mat43 = Eigen::Matrix<T, 4, 3>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-3.14, 3.14};
    const T accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }

    Vec4 randEp() {
        Vec3 axis = randVec3().normalized();
        T phi = dist(generator);
        return Vec4(
            std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));
    }
};

TYPED_TEST_SUITE(BMatrixTypedTest, FloatingPointTypes);

TYPED_TEST(BMatrixTypedTest, BinvEp) {
    auto ep = this->randEp();
    TypeParam expectedArray[3][4] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BinvEP_float(ep.data(), expectedArray);
    else
        BinvEP(ep.data(), expectedArray);
    auto expected = cArray34ToEigenMatrix34(expectedArray);
    EXPECT_LT((binvEp(ep) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BinvMrp) {
    auto mrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BinvMRP_float(mrp.data(), expectedArray);
    else
        BinvMRP(mrp.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT(((1 + mrp.dot(mrp)) * (1 + mrp.dot(mrp)) * binvMrp(mrp) - bmatMrp(mrp).transpose()).norm(),
              this->accuracy);
    EXPECT_LT((binvMrp(mrp) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BinvPrv) {
    auto prv = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BinvPRV_float(prv.data(), expectedArray);
    else
        BinvPRV(prv.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((binvPrv(prv) * bmatPrv(prv) - Eigen::Matrix<TypeParam, 3, 3>::Identity()).norm(), this->accuracy);
    EXPECT_LT((binvPrv(prv) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BinvEuler321) {
    auto euler = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BinvEuler321_float(euler.data(), expectedArray);
    else
        BinvEuler321(euler.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((binvEulerAngles321(euler) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BmatEp) {
    auto ep = this->randEp();
    TypeParam expectedArray[4][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BmatEP_float(ep.data(), expectedArray);
    else
        BmatEP(ep.data(), expectedArray);
    auto expected = cArray43ToEigenMatrix43(expectedArray);
    EXPECT_LT((bmatEp(ep).transpose() * ep).norm(), this->accuracy);
    EXPECT_LT((bmatEp(ep) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BmatMrp) {
    auto mrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BmatMRP_float(mrp.data(), expectedArray);
    else
        BmatMRP(mrp.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((bmatMrp(mrp).transpose() * bmatMrp(mrp) -
               (1 + mrp.dot(mrp)) * (1 + mrp.dot(mrp)) * Eigen::Matrix<TypeParam, 3, 3>::Identity())
                  .norm(),
              this->accuracy);
    EXPECT_LT((bmatMrp(mrp) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BdotMrp) {
    auto mrp = this->randVec3();
    auto dmrp = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BdotmatMRP_float(mrp.data(), dmrp.data(), expectedArray);
    else
        BdotmatMRP(mrp.data(), dmrp.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((bmatDotMrp(mrp, dmrp) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BmatPrv) {
    auto prv = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BmatPRV_float(prv.data(), expectedArray);
    else
        BmatPRV(prv.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((bmatPrv(prv) - expected).norm(), this->accuracy);
}

TYPED_TEST(BMatrixTypedTest, BmatEuler321) {
    auto euler = this->randVec3();
    TypeParam expectedArray[3][3] = {};
    if constexpr (std::is_same_v<TypeParam, float>)
        BmatEuler321_float(euler.data(), expectedArray);
    else
        BmatEuler321(euler.data(), expectedArray);
    auto expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((bmatEulerAngles321(euler) - expected).norm(), this->accuracy);
}

// DCM to representations test
template <typename T>
class DcmToRepresentationTest : public ::testing::Test {
   public:
    using Mat3 = Eigen::Matrix<T, 3, 3>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-3.14, 3.14};
    const T accuracy = kinematicsAccuracy<T>();
};

TYPED_TEST_SUITE(DcmToRepresentationTest, FloatingPointTypes);

TYPED_TEST(DcmToRepresentationTest, dcmToMrp) {
    using T = TypeParam;
    typename TestFixture::Mat3 dcm1 = rotationMatrix(this->dist(this->generator), 1);
    typename TestFixture::Mat3 dcm2 = rotationMatrix(this->dist(this->generator), 2);
    typename TestFixture::Mat3 dcm3 = rotationMatrix(this->dist(this->generator), 3);
    typename TestFixture::Mat3 dcm = dcm3 * dcm2 * dcm1;

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        C2MRP_float(dcmArray, expectedArray);
    else
        C2MRP(dcmArray, expectedArray);
    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);

    EXPECT_LT((dcm * dcm.transpose() - TestFixture::Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((dcmToMrp(dcm) - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, mrpToDcm) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 mrp(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>)
        MRP2C_float(mrp.data(), expectedArray);
    else
        MRP2C(mrp.data(), expectedArray);

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = mrpToDcm<T>(mrp);

    EXPECT_LT((result * result.transpose() - Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((result - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, dcmToPrv) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        C2PRV_float(dcmArray, expectedArray);
    else
        C2PRV(dcmArray, expectedArray);
    auto expected = Eigen::Map<Eigen::Matrix<T, 3, 1>>(expectedArray);

    EXPECT_LT((dcm * dcm.transpose() - TestFixture::Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((dcmToPrv(dcm) - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, prvToDcm) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 prv(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>)
        PRV2C_float(prv.data(), expectedArray);
    else
        PRV2C(prv.data(), expectedArray);

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = prvToDcm<T>(prv);

    EXPECT_LT((result * result.transpose() - Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((result - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, dcmToEuler321) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        C2Euler321_float(dcmArray, expectedArray);
    else
        C2Euler321(dcmArray, expectedArray);
    auto expected = Eigen::Map<Eigen::Matrix<T, 3, 1>>(expectedArray);

    EXPECT_LT((dcm * dcm.transpose() - TestFixture::Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((dcmToEulerAngles321(dcm) - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, eulerAngles321ToDcm) {
    using T = TypeParam;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    Vec3 euler(this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>)
        Euler3212C_float(euler.data(), expectedArray);
    else
        Euler3212C(euler.data(), expectedArray);

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = eulerAngles321ToDcm<T>(euler);

    EXPECT_LT((result * result.transpose() - Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((result - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, dcmToEulerParameter) {
    using T = TypeParam;
    Eigen::Matrix<T, 3, 3> dcm = rotationMatrix(this->dist(this->generator), 3) *
                                 rotationMatrix(this->dist(this->generator), 2) *
                                 rotationMatrix(this->dist(this->generator), 1);

    T dcmArray[3][3];
    eigenMatrix3ToCArray(dcm, dcmArray);
    T expectedArray[4] = {};
    if constexpr (std::is_same_v<T, float>)
        C2EP_float(dcmArray, expectedArray);
    else
        C2EP(dcmArray, expectedArray);
    auto expected = Eigen::Map<Eigen::Matrix<T, 4, 1>>(expectedArray);

    EXPECT_LT((dcm * dcm.transpose() - TestFixture::Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((dcmToEp(dcm) - expected).norm(), this->accuracy);
}

TYPED_TEST(DcmToRepresentationTest, eulerParameterToDcm) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Mat3 = Eigen::Matrix<T, 3, 3>;

    T phi = this->dist(this->generator);
    Eigen::Matrix<T, 3, 1> unitVector(
        this->dist(this->generator), this->dist(this->generator), this->dist(this->generator));
    unitVector.normalize();
    Vec4 ep(std::cos(phi / 2),
            std::sin(phi / 2) * unitVector(0),
            std::sin(phi / 2) * unitVector(1),
            std::sin(phi / 2) * unitVector(2));

    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>)
        EP2C_float(ep.data(), expectedArray);
    else
        EP2C(ep.data(), expectedArray);

    Mat3 expected = cArray33ToEigenMatrix33(expectedArray);
    Mat3 result = epToDcm(ep);

    EXPECT_LT((result * result.transpose() - Mat3::Identity()).norm(), this->accuracy);
    EXPECT_LT((result - expected).norm(), this->accuracy);
}

// Representation-to-representation tests
template <typename T>
class RepresentationTransformTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-3.14, 3.14};
    const T accuracy = kinematicsAccuracy<T>();
};

TYPED_TEST_SUITE(RepresentationTransformTest, FloatingPointTypes);

TYPED_TEST(RepresentationTransformTest, epToMrp) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        EP2MRP_float(ep.data(), expectedArray);
    else
        EP2MRP(ep.data(), expectedArray);

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToMrp<T>(ep);

    EXPECT_LT((result - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationTransformTest, epToPrv) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        EP2PRV_float(ep.data(), expectedArray);
    else
        EP2PRV(ep.data(), expectedArray);

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToPrv<T>(ep);

    EXPECT_LT((result - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationTransformTest, epToEulerAngles321) {
    using T = TypeParam;
    using Vec4 = Eigen::Matrix<T, 4, 1>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    T phi = TestFixture::dist(TestFixture::generator);
    Vec3 axis(TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator),
              TestFixture::dist(TestFixture::generator));
    axis.normalize();
    Vec4 ep(std::cos(phi / 2), std::sin(phi / 2) * axis(0), std::sin(phi / 2) * axis(1), std::sin(phi / 2) * axis(2));

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        EP2Euler321_float(ep.data(), expectedArray);
    else
        EP2Euler321(ep.data(), expectedArray);

    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    Vec3 result = epToEulerAngles321<T>(ep);

    EXPECT_LT((result - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationTransformTest, mrpToPrv) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        MRP2PRV_float(rep.data(), expectedArray);
    else
        MRP2PRV(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((mrpToPrv<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

TYPED_TEST(RepresentationTransformTest, prvToMrp) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        PRV2MRP_float(rep.data(), expectedArray);
    else
        PRV2MRP(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((prvToMrp<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

TYPED_TEST(RepresentationTransformTest, eulerAngles321ToMrp) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        Euler3212MRP_float(rep.data(), expectedArray);
    else
        Euler3212MRP(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((eulerAngles321ToMrp<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

TYPED_TEST(RepresentationTransformTest, mrpToEulerAngles321) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        MRP2Euler321_float(rep.data(), expectedArray);
    else
        MRP2Euler321(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((mrpToEulerAngles321<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

TYPED_TEST(RepresentationTransformTest, prvToEulerAngles321) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        PRV2Euler321_float(rep.data(), expectedArray);
    else
        PRV2Euler321(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((prvToEulerAngles321<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

TYPED_TEST(RepresentationTransformTest, eulerAngles321ToPrv) {
    using T = TypeParam;
    using Vec3 = typename RepresentationTransformTest<T>::Vec3;
    Vec3 rep(RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator),
             RepresentationTransformTest<T>::dist(RepresentationTransformTest<T>::generator));
    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        Euler3212PRV_float(rep.data(), expectedArray);
    else
        Euler3212PRV(rep.data(), expectedArray);
    Vec3 expected = Eigen::Map<Vec3>(expectedArray);
    EXPECT_LT((eulerAngles321ToPrv<T>(rep) - expected).norm(), RepresentationTransformTest<T>::accuracy);
}

// Representation Derivatives Tests
template <typename T>
class RepresentationDerivativesTest : public ::testing::Test {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;

    std::default_random_engine generator{std::random_device{}()};
    std::uniform_real_distribution<T> dist{-3.14, 3.14};
    const T accuracy = kinematicsAccuracy<T>();

    Vec3 randVec3() { return Vec3(dist(generator), dist(generator), dist(generator)); }
};

TYPED_TEST_SUITE(RepresentationDerivativesTest, FloatingPointTypes);

TYPED_TEST(RepresentationDerivativesTest, dmrp) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        dMRP_float(rep.data(), rate.data(), expectedArray);
    else
        dMRP(rep.data(), rate.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((dmrp<T>(rep, rate) - expected).norm(), this->accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, dmrpToOmega) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        dMRP2Omega_float(rep.data(), rate.data(), expectedArray);
    else
        dMRP2Omega(rep.data(), rate.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((dmrpToOmega<T>(rep, rate) - expected).norm(), this->accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, dprv) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        dPRV_float(rep.data(), rate.data(), expectedArray);
    else
        dPRV(rep.data(), rate.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((dprv<T>(rep, rate) - expected).norm(), this->accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, deuler321) {
    using T = TypeParam;
    typename TestFixture::Vec3 rep = this->randVec3();
    typename TestFixture::Vec3 rate = this->randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        dEuler321_float(rep.data(), rate.data(), expectedArray);
    else
        dEuler321(rep.data(), rate.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((deuler321<T>(rep, rate) - expected).norm(), this->accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, ddmrp) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    typename TestFixture::Vec3 dmrp = TestFixture::randVec3();
    typename TestFixture::Vec3 omega = TestFixture::randVec3() / 10.0;
    typename TestFixture::Vec3 domega = TestFixture::randVec3() / 10.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        ddMRP_float(mrp.data(), dmrp.data(), omega.data(), domega.data(), expectedArray);
    else
        ddMRP(mrp.data(), dmrp.data(), omega.data(), domega.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((ddmrp<T>(mrp, dmrp, omega, domega) - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, ddmrpToOmega) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    typename TestFixture::Vec3 dmrp = TestFixture::randVec3() / 10.0;
    typename TestFixture::Vec3 ddmrp = TestFixture::randVec3() / 100.0;

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        ddMRP2dOmega_float(mrp.data(), dmrp.data(), ddmrp.data(), expectedArray);
    else
        ddMRP2dOmega(mrp.data(), dmrp.data(), ddmrp.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((ddmrpTodOmega<T>(mrp, dmrp, ddmrp) - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, mrpSwitch) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();
    T value = TestFixture::dist(TestFixture::generator);

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        MRPswitch_float(mrp.data(), value, expectedArray);
    else
        MRPswitch(mrp.data(), value, expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((mrpSwitch<T>(mrp, value) - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, mrpShadow) {
    using T = TypeParam;
    typename TestFixture::Vec3 mrp = TestFixture::randVec3();

    T expectedArray[3] = {};
    if constexpr (std::is_same_v<T, float>)
        MRPshadow_float(mrp.data(), expectedArray);
    else
        MRPshadow(mrp.data(), expectedArray);

    typename TestFixture::Vec3 expected = Eigen::Map<typename TestFixture::Vec3>(expectedArray);
    EXPECT_LT((mrpShadow<T>(mrp) - expected).norm(), TestFixture::accuracy);
}

TYPED_TEST(RepresentationDerivativesTest, rotationMatrix) {
    using T = TypeParam;
    T expectedArray[3][3] = {};

    for (int axis = 1; axis <= 3; ++axis) {
        T angle = TestFixture::dist(TestFixture::generator);
        if constexpr (std::is_same_v<T, float>)
            Mi_float(angle, axis, expectedArray);
        else
            Mi(angle, axis, expectedArray);

        Eigen::Matrix<T, 3, 3> expected = cArray33ToEigenMatrix33(expectedArray);
        EXPECT_LT((rotationMatrix<T>(angle, axis) - expected).norm(), TestFixture::accuracy);
    }
}

TYPED_TEST(RepresentationDerivativesTest, tildeMatrix) {
    using T = TypeParam;
    typename TestFixture::Vec3 vec = TestFixture::randVec3();
    typename TestFixture::Vec3 testVec = TestFixture::randVec3();

    T expectedArray[3][3] = {};
    if constexpr (std::is_same_v<T, float>)
        tilde_float(vec.data(), expectedArray);
    else
        tilde(vec.data(), expectedArray);

    Eigen::Matrix<T, 3, 3> expected = cArray33ToEigenMatrix33(expectedArray);
    EXPECT_LT((tildeMatrix<T>(vec) * vec).norm(), TestFixture::accuracy);
    EXPECT_LT((tildeMatrix<T>(vec) * testVec - vec.cross(testVec)).norm(), TestFixture::accuracy);
    EXPECT_LT((tildeMatrix<T>(vec) - expected).norm(), TestFixture::accuracy);
}
