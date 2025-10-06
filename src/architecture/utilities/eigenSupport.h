/*
 ISC License

 Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef EIGENSUPPORT
#define EIGENSUPPORT

#include "eigenMRP.h"

#include <Eigen/Core>
#include <cstring>
#include <exception>

template <class Derived>
constexpr bool is_row_major_v = (Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0;

template <class Derived>
inline constexpr bool is_fixed_v =
    (Derived::RowsAtCompileTime != Eigen::Dynamic) && (Derived::ColsAtCompileTime != Eigen::Dynamic);

/**
 * @brief Copy a fixed-size Eigen matrix into a row-major C array.
 *
 * Works for compile-time sized matrices or expressions. Values are flattened
 * in row-major order. Column-major inputs are internally transposed to produce
 * the desired layout.
 *
 * @tparam Derived Fixed-size Eigen expression type.
 * @tparam Size Extent of the destination array (rows × cols).
 * @param inMat Matrix to copy from.
 * @param out Destination array that receives the flattened entries.
 */
template <class Derived, std::size_t Size>
void eigenMatrixToCArray(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[Size]) {
    static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
                  "Input must be a fixed-size Eigen type.");

    using Scalar = typename Derived::Scalar;
    constexpr int Rows = Derived::RowsAtCompileTime;
    constexpr int Cols = Derived::ColsAtCompileTime;

    static_assert(static_cast<std::size_t>(Rows) * static_cast<std::size_t>(Cols) == Size,
                  "Output array size must equal rows*cols of input.");

    if constexpr ((Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0) {
        Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor> tmp = inMat;
        std::memcpy(out, tmp.data(), Size * sizeof(Scalar));
    } else {
        Eigen::Matrix<Scalar, Cols, Rows> tmpT = inMat.transpose();
        std::memcpy(out, tmpT.data(), Size * sizeof(Scalar));
    }
}

/**
 * @brief Copy a dynamic-size Eigen matrix into a row-major C array.
 *
 * Only the first `inMat.size()` elements of `out` are written, allowing the
 * destination buffer to be larger than the matrix. If the buffer is too small,
 * the function terminates the program.
 *
 * @tparam Derived Eigen dynamic expression type.
 * @tparam Size Compile-time extent of the destination buffer.
 * @param inMat Matrix to copy from.
 * @param out Destination array that must hold at least `inMat.size()` values.
 */
template <class Derived, std::size_t Size>
void eigenMatrixXToCArray(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[Size]) {
    using Scalar = typename Derived::Scalar;

    // Runtime capacity check against compile-time size
    if (static_cast<std::size_t>(inMat.size()) > Size) {
        std::terminate();
    }

    // Make a contiguous row-major buffer regardless of input layout
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
    const auto count = static_cast<std::size_t>(inMat.size());
    std::memcpy(out, rm.data(), count * sizeof(Scalar));
}

/**
 * @brief Copy a fixed-size Eigen matrix into a 2-D C array.
 *
 * Performs compile-time shape checks and ensures the row-major layout is
 * preserved. Column-major inputs incur a transpose so that the C array receives
 * data in row-major order.
 *
 * @tparam Derived Fixed-size Eigen expression type.
 * @tparam N Number of rows in the destination array.
 * @tparam M Number of columns in the destination array.
 * @param inMat Matrix to copy from.
 * @param out Destination 2-D array `out[N][M]`.
 */
template <class Derived, std::size_t N, std::size_t M>
void eigenMatrixToCArray2D(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[N][M]) {
    static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
                  "Input must be a fixed-size Eigen type.");

    using Scalar = typename Derived::Scalar;
    constexpr int R = Derived::RowsAtCompileTime;
    constexpr int C = Derived::ColsAtCompileTime;

    static_assert(static_cast<std::size_t>(R) == N && static_cast<std::size_t>(C) == M,
                  "2D output shape must match input rows x cols.");

    if constexpr ((Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0) {
        Eigen::Matrix<Scalar, R, C, Eigen::RowMajor> tmp = inMat;
        std::memcpy(out, tmp.data(), N * M * sizeof(Scalar));
    } else {
        Eigen::Matrix<Scalar, C, R> tmpT = inMat.transpose();
        std::memcpy(out, tmpT.data(), N * M * sizeof(Scalar));
    }
}

/**
 * @brief Copy a dynamic Eigen matrix into a 2-D C array with runtime checks.
 *
 * Validates that the requested 2-D output dimensions match `inMat.rows()` and
 * `inMat.cols()`. If they do not, the function terminates. Data are laid out in
 * row-major order in the destination buffer.
 *
 * @tparam Derived Eigen dynamic expression type.
 * @tparam Rows Expected number of rows in the destination array.
 * @tparam Cols Expected number of columns in the destination array.
 * @param inMat Matrix to copy from.
 * @param out Destination 2-D array `out[Rows][Cols]`.
 */
template <class Derived, std::size_t Rows, std::size_t Cols>
void eigenMatrixXToCArray2D(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[Rows][Cols]) {
    using Scalar = typename Derived::Scalar;

    // Enforce shape at runtime (safer for 2-D indexing)
    if (inMat.rows() != static_cast<int>(Rows) || inMat.cols() != static_cast<int>(Cols)) {
        std::terminate();
    }

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
    std::memcpy(&out[0][0], rm.data(), Rows * Cols * sizeof(Scalar));
}

/**
 * @brief Insert a flattened Eigen matrix into a strided C array slice.
 *
 * The matrix is flattened in row-major order and written starting at `offset`.
 * Elements are spaced by `stride`. If `stride` is zero and more than one value
 * would be written, the function terminates. Capacity violations also
 * terminate execution.
 *
 * @tparam Derived Eigen expression type.
 * @tparam Size Compile-time extent of the destination array.
 * @param inMat Matrix supplying the values.
 * @param out Destination array that receives the data.
 * @param offset Starting index in `out` for the first element.
 * @param stride Number of indices to skip between elements (default 1).
 */
template <class Derived, std::size_t Size>
void eigenMatrixXInsertCArray(const Eigen::MatrixBase<Derived>& inMat,
                              typename Derived::Scalar (&out)[Size],
                              std::size_t offset,
                              const std::size_t stride = 1) {
    using Scalar = typename Derived::Scalar;

    const auto count = static_cast<std::size_t>(inMat.size());
    if (count == 0) return;

    if (stride == 0 && count > 1) {
        std::terminate();
    }

    // Capacity check: last index must be < N
    const std::size_t last_index = offset + (count - 1) * stride;
    if (last_index >= Size) {
        std::terminate();
    }

    // Make a contiguous row-major buffer regardless of input layout
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();

    if (stride == 1) {
        std::memcpy(out + offset, rm.data(), count * sizeof(Scalar));
    } else {
        const Scalar* src = rm.data();
        std::size_t idx = offset;
        for (std::size_t i = 0; i < count; ++i) {
            out[idx] = src[i];
            idx += stride;
        }
    }
}

/**
 * @brief Copy a fixed-size Eigen vector into a contiguous C array.
 *
 * @tparam ScalarT Scalar type stored in the vector.
 * @tparam size Compile-time number of elements.
 * @param inVec Vector whose contents should be copied.
 * @param outArray Pointer to a C array with at least `size` elements.
 */
template <typename ScalarT, int size>
void eigenVectorToCArray(const Eigen::Vector<ScalarT, size>& inVec, ScalarT* outArray) {
    memcpy(outArray, inVec.data(), size * sizeof(ScalarT));
}

/**
 * @brief Map a row-major C array onto a fixed-size Eigen matrix.
 *
 * @tparam ScalarT Scalar type of the matrix.
 * @tparam rows Number of rows in the output matrix.
 * @tparam cols Number of columns in the output matrix.
 * @param inArray Pointer to `rows * cols` elements in row-major order.
 * @return Eigen matrix populated with the values from `inArray`.
 */
template <typename ScalarT, int rows, int cols>
Eigen::Matrix<ScalarT, rows, cols> cArrayAsEigenMatrix(ScalarT* inArray) {
    Eigen::Matrix<ScalarT, rows, cols> outMat;
    outMat = Eigen::Map<Eigen::Matrix<ScalarT, rows, cols>>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/**
 * @brief Map a row-major C array onto a dynamic Eigen matrix.
 *
 * @tparam ScalarT Scalar type of the matrix.
 * @param inArray Pointer to `nRows * nCols` elements in row-major order.
 * @param nRows Desired number of rows of the output matrix.
 * @param nCols Desired number of columns of the output matrix.
 * @return Eigen dynamic matrix containing the mapped values.
 */
template <typename ScalarT>
Eigen::MatrixX<ScalarT> cArrayAsEigenMatrixX(ScalarT* inArray, int nRows, int nCols) {
    Eigen::MatrixX<ScalarT> outMat;
    outMat.resize(nRows, nCols);
    outMat = Eigen::Map<Eigen::MatrixX<ScalarT>>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/**
 * @brief Map a C array onto a fixed-size Eigen vector.
 *
 * @tparam ScalarT Scalar type of the vector.
 * @tparam size Compile-time number of elements.
 * @param inArray Reference to the C array holding the coefficients.
 * @return Eigen vector whose contents match the input array.
 */
template <typename ScalarT, std::size_t size>
Eigen::Vector<ScalarT, size> cArrayAsEigenVector(ScalarT (&inArray)[size]) {
    return Eigen::Map<Eigen::Vector<ScalarT, size>>(inArray);
}

/**
 * @brief Convert a three-element C array into an Eigen 3-vector.
 *
 * @tparam ScalarT Scalar type of the vector.
 * @param inArray Pointer to three consecutive elements.
 * @return Eigen::Vector3 populated from the input data.
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> cArrayAsEigenVector3(ScalarT* inArray) {
    return Eigen::Map<Eigen::Vector3<ScalarT>>(inArray);
}

/**
 * @brief Convert a three-element C array into an Eigen modified Rodrigues parameter.
 *
 * @tparam ScalarT Scalar type of the MRP coefficients.
 * @param inArray Pointer to three elements representing the MRP components.
 * @return Eigen::MRP constructed from the input.
 */
template <typename ScalarT>
Eigen::MRP<ScalarT> cArrayAsEigenMrp(ScalarT* inArray) {
    Eigen::MRP<ScalarT> sigma_Eigen;
    sigma_Eigen = Eigen::Map<Eigen::Vector<ScalarT, 3>>(inArray);

    return sigma_Eigen;
}

/**
 * @brief Convert a row-major C array into an Eigen 3×3 matrix.
 *
 * @tparam ScalarT Scalar type of the matrix.
 * @param inArray Pointer to nine elements stored in row-major order.
 * @return Eigen::Matrix3 with the copied values.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> cArrayAsEigenMatrix3(ScalarT* inArray) {
    return Eigen::Map<Eigen::Matrix3<ScalarT>>(inArray, 3, 3).transpose();
}

/**
 * @brief Copy a 3×3 C two-dimensional array into an Eigen 3×3 matrix.
 *
 * @tparam ScalarT Scalar type of the matrix.
 * @param in2DArray Source array with bounds `[3][3]`.
 * @return Eigen::Matrix3 containing the same entries.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> c2DArrayAsEigenMatrix3(ScalarT in2DArray[3][3]) {
    Eigen::Matrix3<ScalarT> outMat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            outMat(i, j) = in2DArray[i][j];
        }
    }

    return outMat;
}

/**
 * @brief Convert an Eigen MRP to a three-component vector.
 *
 * @tparam ScalarT Scalar type of the MRP.
 * @param mrp Modified Rodrigues parameter to convert.
 * @return Eigen::Vector3 containing the MRP coefficients.
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> eigenMrpToVector3(const Eigen::MRP<ScalarT> mrp) {
    Eigen::Vector3<ScalarT> vec3;

    vec3[0] = mrp.x();
    vec3[1] = mrp.y();
    vec3[2] = mrp.z();

    return vec3;
}

/**
 * @brief Create the DCM for a positive rotation about the body X-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM1(const ScalarT angle) {
    Eigen::Matrix3<ScalarT> mOut = Eigen::Matrix3<ScalarT>::Identity();

    mOut(1, 1) = std::cos(angle);
    mOut(1, 2) = std::sin(angle);
    mOut(2, 1) = -std::sin(angle);
    mOut(2, 2) = std::cos(angle);

    return mOut;
}

/**
 * @brief Create the DCM for a positive rotation about the body Y-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM2(const ScalarT angle) {
    Eigen::Matrix3<ScalarT> mOut = Eigen::Matrix3<ScalarT>::Identity();

    mOut(0, 0) = std::cos(angle);
    mOut(0, 2) = -std::sin(angle);
    mOut(2, 0) = std::sin(angle);
    mOut(2, 2) = std::cos(angle);

    return mOut;
}

/**
 * @brief Create the DCM for a positive rotation about the body Z-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM3(const ScalarT angle) {
    Eigen::Matrix3<ScalarT> mOut = Eigen::Matrix3<ScalarT>::Identity();

    mOut(0, 0) = std::cos(angle);
    mOut(0, 1) = std::sin(angle);
    mOut(1, 0) = -std::sin(angle);
    mOut(1, 1) = std::cos(angle);

    return mOut;
}

/**
 * @brief Construct the skew-symmetric matrix such that `[tilde(vec)] * b = vec × b`.
 *
 * @tparam Derived Eigen vector expression type.
 * @param vec Vector whose associated tilde matrix is requested.
 * @return Eigen::Matrix3 representing the skew-symmetric cross-product matrix.
 */
template <typename Derived>
Eigen::Matrix3<typename Eigen::MatrixBase<Derived>::Scalar> eigenTilde(const Eigen::MatrixBase<Derived>& vec) {
    using Scalar = typename Eigen::MatrixBase<Derived>::Scalar;

    Eigen::Matrix3<Scalar> mOut = Eigen::Matrix3<Scalar>::Zero();

    mOut(0, 1) = -vec(2);
    mOut(1, 0) = vec(2);
    mOut(0, 2) = vec(1);
    mOut(2, 0) = -vec(1);
    mOut(1, 2) = -vec(0);
    mOut(2, 1) = vec(0);

    return mOut;
}

#endif  // EIGENSUPPORT
