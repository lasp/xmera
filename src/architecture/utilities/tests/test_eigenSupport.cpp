// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include "architecture/utilities/eigenSupport.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <type_traits>
#include <vector>

// -------- Compile-time sanity checks for traits --------
static_assert(is_fixed_v<Eigen::Matrix<float, 2, 3>>);
static_assert(!is_fixed_v<Eigen::Matrix<float, Eigen::Dynamic, 3>>);
static_assert(is_row_major_v<Eigen::Matrix<float, 2, 3, Eigen::RowMajor>>);
static_assert(!is_row_major_v<Eigen::Matrix<float, 2, 3>>);

// -------- Test utilities --------
template<typename T>
constexpr T GetTolerance() {
    if constexpr (std::is_same_v<T, float>) {
        return static_cast<T>(1e-5);
    } else {
        return static_cast<T>(1e-12);
    }
}

template<typename DerivedA, typename DerivedB>
void ExpectMatricesApproxEqual(
    Eigen::MatrixBase<DerivedA> const &expected,
    Eigen::MatrixBase<DerivedB> const &actual,
    typename DerivedA::Scalar tolerance
) {
    static_assert(std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>, "Scalar types must match");
    ASSERT_EQ(expected.rows(), actual.rows());
    ASSERT_EQ(expected.cols(), actual.cols());
    for (int i = 0; i < expected.rows(); ++i) {
        for (int j = 0; j < expected.cols(); ++j) { EXPECT_NEAR(expected(i, j), actual(i, j), tolerance); }
    }
}

using ScalarTypes = ::testing::Types<float, double>;

template<typename T>
class EigenSupportConversionsTest : public ::testing::Test {};

TYPED_TEST_SUITE(EigenSupportConversionsTest, ScalarTypes);

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixToCArrayCopiesInRowMajorOrder) {
    using Scalar = TypeParam;
    constexpr int rows = 2;
    constexpr int cols = 3;
    Eigen::Matrix<Scalar, rows, cols> input;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { input(i, j) = static_cast<Scalar>(i * cols + j + 1); }
    }

    Scalar output[rows * cols];
    eigenMatrixToCArray(input, output);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int const index = i * cols + j;
            EXPECT_EQ(output[index], input(i, j));
        }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixToCArrayRowMajorInput) {
    using Scalar = TypeParam;
    using RowMajorMatrix = Eigen::Matrix<Scalar, 2, 3, Eigen::RowMajor>;
    RowMajorMatrix input;
    for (int i = 0, value = 1; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j, ++value) { input(i, j) = static_cast<Scalar>(value); }
    }

    Scalar output[RowMajorMatrix::RowsAtCompileTime * RowMajorMatrix::ColsAtCompileTime] = {};
    eigenMatrixToCArray(input, output);

    for (int i = 0; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j) {
            int const index = i * input.cols() + j;
            EXPECT_EQ(output[index], input(i, j));
        }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXToCArrayCopiesInRowMajorOrder) {
    using Scalar = TypeParam;
    constexpr int rows = 3;
    constexpr int cols = 2;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { input(i, j) = static_cast<Scalar>((i + 1) * 10 + j); }
    }

    Scalar output[rows * cols];
    eigenMatrixXToCArray(input, output);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int const index = i * cols + j;
            EXPECT_EQ(output[index], input(i, j));
        }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenVectorToCArrayCopiesElements) {
    using Scalar = TypeParam;
    constexpr int size = 4;
    Eigen::Vector<Scalar, size> input;
    for (int i = 0; i < size; ++i) { input(i) = static_cast<Scalar>(i - 1); }

    Scalar output[size] = {};
    eigenVectorToCArray(input, output);

    for (int i = 0; i < size; ++i) { EXPECT_EQ(output[i], input(i)); }
}

TYPED_TEST(EigenSupportConversionsTest, EigenVectorToCArrayAcceptsExpression) {
    using Scalar = TypeParam;

    // Constant expression (Zero) - the original failure mode that prompted
    // widening the signature to MatrixBase<Derived>.
    Scalar zeroOut[3] = {static_cast<Scalar>(7), static_cast<Scalar>(7), static_cast<Scalar>(7)};
    eigenVectorToCArray(Eigen::Vector3<Scalar>::Zero(), zeroOut);
    for (int i = 0; i < 3; ++i) { EXPECT_EQ(zeroOut[i], static_cast<Scalar>(0)); }

    // Block expression - exercises the PlainObject evaluation path.
    Eigen::Matrix<Scalar, 4, 1> source;
    source << static_cast<Scalar>(1), static_cast<Scalar>(2), static_cast<Scalar>(3), static_cast<Scalar>(4);

    Scalar blockOut[3] = {};
    eigenVectorToCArray(source.template head<3>(), blockOut);
    for (int i = 0; i < 3; ++i) { EXPECT_EQ(blockOut[i], source(i)); }
}

TYPED_TEST(EigenSupportConversionsTest, cArrayToEigenMatrixPreservesColumnMajorOrdering) {
    using Scalar = TypeParam;
    constexpr int rows = 3;
    constexpr int cols = 2;
    Eigen::Matrix<Scalar, rows, cols> original;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { original(i, j) = static_cast<Scalar>(i + j * 0.5 + 1.0); }
    }

    Eigen::Matrix<Scalar, rows, cols> reconstructed = cArrayToEigenMatrix<Scalar, rows, cols>(original.data());
    ExpectMatricesApproxEqual(original, reconstructed, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenMatrixXPreservesColumnMajorOrdering) {
    using Scalar = TypeParam;
    int const rows = 4;
    int const cols = 1;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> original(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { original(i, j) = static_cast<Scalar>(i * 0.25 + j); }
    }

    auto reconstructed = cArrayToEigenMatrixX(original.data(), rows, cols);
    ExpectMatricesApproxEqual(original, reconstructed, GetTolerance<Scalar>());
}

// The output side writes row-major and the general-shape input side reads
// column-major (rule 3 at the top of eigenSupport.h), so the two are not
// inverses. The next two tests pin that asymmetry down. They are not asserting
// that it is desirable - they exist so that changing the layout on one side
// alone fails here, rather than silently transposing matrices in the modules
// that round-trip through a message buffer.
TYPED_TEST(EigenSupportConversionsTest, RoundTripThroughCArrayTransposesSquareInput) {
    using Scalar = TypeParam;
    constexpr int dim = 3;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(dim, dim);
    for (int i = 0; i < dim; ++i) {
        for (int j = 0; j < dim; ++j) { input(i, j) = static_cast<Scalar>(i * dim + j + 1); }
    }

    Scalar buffer[dim * dim] = {};
    eigenMatrixXToCArray(input, buffer);
    auto roundTripped = cArrayToEigenMatrixX(buffer, dim, dim);

    ExpectMatricesApproxEqual(input.transpose(), roundTripped, GetTolerance<Scalar>());
    // Stated the other way: the round trip is not the identity. A symmetric
    // matrix cannot tell the two layouts apart, which is why callers that only
    // ever exercise the identity matrix don't notice the difference.
    EXPECT_NE(input(0, 1), roundTripped(0, 1));
}

TYPED_TEST(EigenSupportConversionsTest, RoundTripThroughCArrayNeedsSwappedDimsForNonSquare) {
    using Scalar = TypeParam;
    constexpr int rows = 3;
    constexpr int cols = 2;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { input(i, j) = static_cast<Scalar>(i * cols + j + 1); }
    }

    Scalar buffer[rows * cols] = {};
    eigenMatrixXToCArray(input, buffer);

    // Reading back with the original dimensions reorders entries rather than
    // transposing: what was input(0, 1) lands at (1, 0) of a 3 x 2 result.
    auto sameDims = cArrayToEigenMatrixX(buffer, rows, cols);
    EXPECT_NEAR(sameDims(1, 0), input(0, 1), GetTolerance<Scalar>());

    // Reading back with the dimensions swapped recovers the exact transpose.
    auto swappedDims = cArrayToEigenMatrixX(buffer, cols, rows);
    ExpectMatricesApproxEqual(input.transpose(), swappedDims, GetTolerance<Scalar>());
}

// The pairing that is the identity: the output side writes row-major, and
// cArrayToEigenMatrix3 reads row-major. Both the flat and the strided writer
// are covered, because a C array can hold either a single 3 x 3 matrix or a
// sequence of them packed one after another.
TYPED_TEST(EigenSupportConversionsTest, RowMajorWriteRoundTripsThroughCArrayToEigenMatrix3) {
    using Scalar = TypeParam;
    constexpr int dim = 3;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(dim, dim);
    for (int i = 0; i < dim; ++i) {
        for (int j = 0; j < dim; ++j) { input(i, j) = static_cast<Scalar>(i * dim + j + 1); }
    }

    Scalar flat[dim * dim] = {};
    eigenMatrixXToCArray(input, flat);
    ExpectMatricesApproxEqual(input, cArrayToEigenMatrix3(flat), GetTolerance<Scalar>());

    // Second slot of a two-matrix buffer, as a strided write produces it.
    Scalar strided[2 * dim * dim] = {};
    eigenMatrixXInsertCArray(input, strided, dim * dim);
    ExpectMatricesApproxEqual(input, cArrayToEigenMatrix3(&strided[dim * dim]), GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenVectorCopiesElements) {
    using Scalar = TypeParam;
    constexpr std::size_t size = 5;
    Scalar raw[size] = {};
    for (std::size_t i = 0; i < size; ++i) { raw[i] = static_cast<Scalar>(i * 2 - 3); }

    auto vec = cArrayToEigenVector(raw);
    for (std::size_t i = 0; i < size; ++i) { EXPECT_EQ(vec(static_cast<int>(i)), raw[i]); }
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenVectorAcceptsConstInput) {
    using Scalar = TypeParam;
    Scalar const raw[3] = {static_cast<Scalar>(1.0), static_cast<Scalar>(2.0), static_cast<Scalar>(3.0)};
    auto vec = cArrayToEigenVector(raw);
    for (int i = 0; i < 3; ++i) { EXPECT_EQ(vec(i), raw[i]); }
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenVector3CreatesMatchingVector) {
    using Scalar = TypeParam;
    Scalar raw[3] = {static_cast<Scalar>(1.0), static_cast<Scalar>(-2.0), static_cast<Scalar>(0.5)};

    Eigen::Vector3<Scalar> vec = cArrayToEigenVector3(raw);
    for (int i = 0; i < 3; ++i) { EXPECT_EQ(vec(i), raw[i]); }
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenVector3AcceptsConstInput) {
    using Scalar = TypeParam;
    Scalar const raw[3] = {static_cast<Scalar>(4.0), static_cast<Scalar>(-5.0), static_cast<Scalar>(6.0)};
    Eigen::Vector3<Scalar> vec = cArrayToEigenVector3(raw);
    for (int i = 0; i < 3; ++i) { EXPECT_EQ(vec(i), raw[i]); }
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenMrpCopiesCoefficients) {
    using Scalar = TypeParam;
    Scalar raw[3] = {static_cast<Scalar>(0.1), static_cast<Scalar>(-0.2), static_cast<Scalar>(0.3)};

    Eigen::MRP<Scalar> mrp = cArrayToEigenMrp(raw);
    EXPECT_EQ(mrp.x(), raw[0]);
    EXPECT_EQ(mrp.y(), raw[1]);
    EXPECT_EQ(mrp.z(), raw[2]);
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenMrpAcceptsConstInput) {
    using Scalar = TypeParam;
    Scalar const raw[3] = {static_cast<Scalar>(0.1), static_cast<Scalar>(-0.2), static_cast<Scalar>(0.3)};
    Eigen::MRP<Scalar> mrp = cArrayToEigenMrp(raw);
    EXPECT_EQ(mrp.x(), raw[0]);
    EXPECT_EQ(mrp.y(), raw[1]);
    EXPECT_EQ(mrp.z(), raw[2]);
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenMatrix3AcceptsConstInput) {
    using Scalar = TypeParam;
    std::array<Scalar, 9> const raw = {
        static_cast<Scalar>(1),
        static_cast<Scalar>(2),
        static_cast<Scalar>(3),
        static_cast<Scalar>(4),
        static_cast<Scalar>(5),
        static_cast<Scalar>(6),
        static_cast<Scalar>(7),
        static_cast<Scalar>(8),
        static_cast<Scalar>(9)
    };

    Eigen::Matrix3<Scalar> expected;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { expected(i, j) = static_cast<Scalar>(i * 3 + j + 1); }
    }

    Eigen::Matrix3<Scalar> reconstructed = cArrayToEigenMatrix3(raw.data());
    ExpectMatricesApproxEqual(expected, reconstructed, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, CArrayToEigenMatrix3ReadsRowMajorDataCorrectly) {
    using Scalar = TypeParam;
    std::array<Scalar, 9> raw = {
        static_cast<Scalar>(1),
        static_cast<Scalar>(2),
        static_cast<Scalar>(3),
        static_cast<Scalar>(4),
        static_cast<Scalar>(5),
        static_cast<Scalar>(6),
        static_cast<Scalar>(7),
        static_cast<Scalar>(8),
        static_cast<Scalar>(9)
    };

    Eigen::Matrix3<Scalar> expected;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { expected(i, j) = static_cast<Scalar>(i * 3 + j + 1); }
    }

    Eigen::Matrix3<Scalar> reconstructed = cArrayToEigenMatrix3(raw.data());
    ExpectMatricesApproxEqual(expected, reconstructed, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, C2DArrayToEigenMatrix3CopiesEntries) {
    using Scalar = TypeParam;
    Scalar raw[3][3] = {
        {static_cast<Scalar>(1), static_cast<Scalar>(2), static_cast<Scalar>(3)},
        {static_cast<Scalar>(4), static_cast<Scalar>(5), static_cast<Scalar>(6)},
        {static_cast<Scalar>(7), static_cast<Scalar>(8), static_cast<Scalar>(9)}
    };

    Eigen::Matrix3<Scalar> reconstructed = c2DArrayToEigenMatrix3(raw);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { EXPECT_EQ(reconstructed(i, j), raw[i][j]); }
    }
}

TYPED_TEST(EigenSupportConversionsTest, C2DArrayToEigenMatrix3AcceptsConstInput) {
    using Scalar = TypeParam;
    Scalar const raw[3][3] = {
        {static_cast<Scalar>(1), static_cast<Scalar>(2), static_cast<Scalar>(3)},
        {static_cast<Scalar>(4), static_cast<Scalar>(5), static_cast<Scalar>(6)},
        {static_cast<Scalar>(7), static_cast<Scalar>(8), static_cast<Scalar>(9)}
    };

    Eigen::Matrix3<Scalar> reconstructed = c2DArrayToEigenMatrix3(raw);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { EXPECT_EQ(reconstructed(i, j), raw[i][j]); }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixToCArray2DColumnMajorInput) {
    using Scalar = TypeParam;
    using MatrixType = Eigen::Matrix<Scalar, 2, 3>;
    MatrixType input;
    for (int j = 0, value = 1; j < input.cols(); ++j) {
        for (int i = 0; i < input.rows(); ++i, ++value) { input(i, j) = static_cast<Scalar>(value); }
    }

    Scalar output[2][3] = {};
    eigenMatrixToCArray2D(input, output);

    for (int i = 0; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j) { EXPECT_EQ(output[i][j], input(i, j)); }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixToCArray2DRowMajorInput) {
    using Scalar = TypeParam;
    using RowMajorMatrix = Eigen::Matrix<Scalar, 2, 3, Eigen::RowMajor>;
    RowMajorMatrix input;
    for (int i = 0, value = 0; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j, ++value) { input(i, j) = static_cast<Scalar>(value + 0.5); }
    }

    Scalar output[2][3] = {};
    eigenMatrixToCArray2D(input, output);

    for (int i = 0; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j) { EXPECT_EQ(output[i][j], input(i, j)); }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMrpToVector3ExtractsComponents) {
    using Scalar = TypeParam;
    Eigen::MRP<Scalar> mrp(static_cast<Scalar>(0.05), static_cast<Scalar>(-0.1), static_cast<Scalar>(0.2));

    Eigen::Vector3<Scalar> vec = eigenMrpToVector3(mrp);
    EXPECT_EQ(vec(0), mrp.x());
    EXPECT_EQ(vec(1), mrp.y());
    EXPECT_EQ(vec(2), mrp.z());
}

TYPED_TEST(EigenSupportConversionsTest, EigenM1MatchesRotationAboutXAxis) {
    using Scalar = TypeParam;
    Scalar const angle = static_cast<Scalar>(0.37);
    Scalar const c = static_cast<Scalar>(std::cos(static_cast<double>(angle)));
    Scalar const s = static_cast<Scalar>(std::sin(static_cast<double>(angle)));

    Eigen::Matrix3<Scalar> expected;
    expected << static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0), static_cast<Scalar>(0), c, s,
        static_cast<Scalar>(0), -s, c;

    Eigen::Matrix3<Scalar> rotation = eigenM1(angle);
    ExpectMatricesApproxEqual(expected, rotation, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, EigenM2MatchesRotationAboutYAxis) {
    using Scalar = TypeParam;
    Scalar const angle = static_cast<Scalar>(-0.42);
    Scalar const c = static_cast<Scalar>(std::cos(static_cast<double>(angle)));
    Scalar const s = static_cast<Scalar>(std::sin(static_cast<double>(angle)));

    Eigen::Matrix3<Scalar> expected;
    expected << c, static_cast<Scalar>(0), -s, static_cast<Scalar>(0), static_cast<Scalar>(1), static_cast<Scalar>(0),
        s, static_cast<Scalar>(0), c;

    Eigen::Matrix3<Scalar> rotation = eigenM2(angle);
    ExpectMatricesApproxEqual(expected, rotation, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, EigenM3MatchesRotationAboutZAxis) {
    using Scalar = TypeParam;
    Scalar const angle = static_cast<Scalar>(1.1);
    Scalar const c = static_cast<Scalar>(std::cos(static_cast<double>(angle)));
    Scalar const s = static_cast<Scalar>(std::sin(static_cast<double>(angle)));

    Eigen::Matrix3<Scalar> expected;
    expected << c, s, static_cast<Scalar>(0), -s, c, static_cast<Scalar>(0), static_cast<Scalar>(0),
        static_cast<Scalar>(0), static_cast<Scalar>(1);

    Eigen::Matrix3<Scalar> rotation = eigenM3(angle);
    ExpectMatricesApproxEqual(expected, rotation, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, EigenTildeBuildsCorrectSkewMatrix) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, 3, 1> vec;
    vec << static_cast<Scalar>(1.5), static_cast<Scalar>(-0.7), static_cast<Scalar>(2.0);

    Eigen::Matrix3<Scalar> tilde = eigenTilde(vec);

    Eigen::Matrix<Scalar, 3, 1> other;
    other << static_cast<Scalar>(0.3), static_cast<Scalar>(-1.0), static_cast<Scalar>(2.5);

    Eigen::Matrix<Scalar, 3, 1> crossProduct = vec.cross(other);
    Eigen::Matrix<Scalar, 3, 1> matrixProduct = tilde * other;

    ExpectMatricesApproxEqual(crossProduct, matrixProduct, GetTolerance<Scalar>());

    Eigen::Matrix3<Scalar> skewCheck = tilde + tilde.transpose();
    ExpectMatricesApproxEqual(Eigen::Matrix3<Scalar>::Zero(), skewCheck, GetTolerance<Scalar>());
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXToCArrayDiesOnSizeMismatch) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(2, 3);
    // Fill with 1..6 (row-major reading order)
    {
        int k = 0;
        for (int i = 0; i < input.rows(); ++i) {
            for (int j = 0; j < input.cols(); ++j) { input(i, j) = static_cast<Scalar>(++k); }
        }
    }

    Scalar out_good[6];
    ASSERT_NO_FATAL_FAILURE(eigenMatrixXToCArray(input, out_good));

    Scalar out_bad[5];
    // std::terminate() → death test
    ASSERT_DEATH({ eigenMatrixXToCArray(input, out_bad); }, ".*");
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXToCArrayAllowsLargerDestination) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(2, 2);
    input << static_cast<Scalar>(1), static_cast<Scalar>(2), static_cast<Scalar>(3), static_cast<Scalar>(4);

    Scalar out_large[6];
    for (Scalar &v : out_large) { v = static_cast<Scalar>(-1); }

    ASSERT_NO_FATAL_FAILURE(eigenMatrixXToCArray(input, out_large));

    EXPECT_EQ(out_large[0], static_cast<Scalar>(1));
    EXPECT_EQ(out_large[1], static_cast<Scalar>(2));
    EXPECT_EQ(out_large[2], static_cast<Scalar>(3));
    EXPECT_EQ(out_large[3], static_cast<Scalar>(4));
    EXPECT_EQ(out_large[4], static_cast<Scalar>(-1));
    EXPECT_EQ(out_large[5], static_cast<Scalar>(-1));
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXToCArray2DCopiesInRowMajorOrder) {
    using Scalar = TypeParam;
    constexpr int rows = 2;
    constexpr int cols = 3;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { input(i, j) = static_cast<Scalar>(10 + i * cols + j); }
    }

    Scalar out[2][3] = {};
    ASSERT_NO_FATAL_FAILURE(eigenMatrixXToCArray2D(input, out));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) { EXPECT_EQ(out[i][j], input(i, j)); }
    }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXToCArray2DDiesOnShapeMismatch) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(2, 3);
    input.setOnes();

    Scalar out_wrong[3][3] = {};
    ASSERT_DEATH(
        {
            eigenMatrixXToCArray2D(input, out_wrong);  // rows mismatch
        },
        ".*"
    );
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_OffsetStride1) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(2, 3);
    // row-major flatten: [1 2 3 4 5 6]
    for (int i = 0, k = 1; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j, ++k) { input(i, j) = static_cast<Scalar>(k); }
    }

    Scalar out[10] = {};
    eigenMatrixXInsertCArray(input, out, /*offset=*/2, /*stride=*/1);

    // Expect out[2..7] == 1..6
    for (int i = 0; i < 6; ++i) { EXPECT_EQ(out[2 + i], static_cast<Scalar>(i + 1)); }
    // Ensure no other indices were touched
    EXPECT_EQ(out[0], static_cast<Scalar>(0));
    EXPECT_EQ(out[1], static_cast<Scalar>(0));
    for (int i = 8; i < 10; ++i) { EXPECT_EQ(out[i], static_cast<Scalar>(0)); }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_StrideGT1) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(1, 5);
    for (int j = 0; j < 5; ++j) {
        input(0, j) = static_cast<Scalar>(j + 1);  // [1 2 3 4 5]
    }

    Scalar out[20];
    for (auto &v : out) { v = static_cast<Scalar>(-1); }

    eigenMatrixXInsertCArray(input, out, /*offset=*/3, /*stride=*/2);
    // Writes to indices: 3,5,7,9,11
    for (int k = 0; k < 5; ++k) { EXPECT_EQ(out[3 + 2 * k], static_cast<Scalar>(k + 1)); }
    // untouched sentinel checks
    EXPECT_EQ(out[2], static_cast<Scalar>(-1));
    EXPECT_EQ(out[4], static_cast<Scalar>(-1));
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_DiesOnOverrun) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(2, 3);  // 6 elems
    input.setZero();
    Scalar out[8] = {};

    // last index = offset + (count-1)*stride
    // Choose offset=4, stride=1 → last index = 4+(6-1)*1 = 9 (>=8) → death
    ASSERT_DEATH({ eigenMatrixXInsertCArray(input, out, /*offset=*/4, /*stride=*/1); }, ".*");
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_DiesOnZeroStrideForMultiple) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(1, 3);  // 3 elems
    // Fill with 1..3
    for (int j = 0; j < input.cols(); ++j) { input(0, j) = static_cast<Scalar>(j + 1); }
    Scalar out[10] = {};

    ASSERT_DEATH({ eigenMatrixXInsertCArray(input, out, /*offset=*/0, /*stride=*/0); }, ".*");
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_NoOpOnZeroSize) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> empty(0, 0);
    Scalar out[4] = {static_cast<Scalar>(9), static_cast<Scalar>(9), static_cast<Scalar>(9), static_cast<Scalar>(9)};

    // Should return immediately, not touching output
    ASSERT_NO_FATAL_FAILURE(eigenMatrixXInsertCArray(empty, out, /*offset=*/1, /*stride=*/3));

    for (Scalar v : out) { EXPECT_EQ(v, static_cast<Scalar>(9)); }
}

TYPED_TEST(EigenSupportConversionsTest, EigenMatrixXInsertCArray_AllowsZeroStrideForSingleElement) {
    using Scalar = TypeParam;
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input(1, 1);
    input(0, 0) = static_cast<Scalar>(12.5);

    Scalar out[3] = {static_cast<Scalar>(-1), static_cast<Scalar>(-1), static_cast<Scalar>(-1)};
    ASSERT_NO_FATAL_FAILURE(eigenMatrixXInsertCArray(input, out, /*offset=*/1, /*stride=*/0));

    EXPECT_EQ(out[0], static_cast<Scalar>(-1));
    EXPECT_EQ(out[1], static_cast<Scalar>(12.5));
    EXPECT_EQ(out[2], static_cast<Scalar>(-1));
}
