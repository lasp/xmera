// SPDX-License-Identifier: ISC
// Copyright (c) 2015, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef EIGEN_SUPPORT
#define EIGEN_SUPPORT

#include <architecture/utilities/eigenMRP.h>

#include <Eigen/Core>
#include <cstring>
#include <exception>

// =============================================================================
// Design goals for this header
// =============================================================================
//
// This header bridges Eigen types and plain C arrays at the boundary with
// Ada/SWIG/message-buffer code. The functions below already follow a small
// set of conventions; the rules are stated here so that future contributions
// stay consistent and so deviations are easy to spot.
//
// 1. Two directions, two parameter conventions.
//    Output-side functions (Eigen -> C array) take the destination as a sized
//    array reference `T (&out)[N]` so the buffer length is deduced and bounds
//    are checked at compile time. Input-side functions (C array -> Eigen)
//    take a `const ScalarT*` because callers commonly hand them stride-indexed
//    offsets into larger buffers (e.g. `&GsMatrix_B[i * 3]`) or struct member
//    arrays - array-reference parameters would force those callers into
//    awkward casts. The `cArrayToEigenVector` overload is the exception (its
//    size is part of the contract) and accepts `const ScalarT (&)[size]`.
//
// 2. Compile-time shape enforcement when the type carries it.
//    Fixed-size variants (`eigenMatrixToCArray`, `eigenMatrixToCArray2D`,
//    `eigenVectorToCArray`, `cArrayToEigenMatrix`, `cArrayToEigenVector3`,
//    `cArrayToEigenMatrix3`, `c2DArrayToEigenMatrix3`, `eigenTilde`) use
//    `static_assert` on `RowsAtCompileTime` / `ColsAtCompileTime` and on
//    destination size. Compile-time fixed sizes are regularly valuable in
//    embedded / flight-software contexts - they enable static stack
//    reasoning, eliminate heap allocation, and surface shape mismatches
//    before the binary leaves the developer's machine - and the FSW side
//    of this codebase prefers them by default. Dynamic-size variants (named
//    with an `X` infix: `eigenMatrixXToCArray`, `eigenMatrixXToCArray2D`,
//    `eigenMatrixXInsertCArray`, `cArrayToEigenMatrixX`) fall back to runtime
//    checks and `std::terminate()` on shape or capacity violations. The
//    dynamic variants exist for host-PC / Xmera simulation modules where
//    shapes legitimately depend on runtime configuration (variable-length
//    sensor arrays, scenario-driven reaction wheel counts, etc.) and the
//    FSW compile-time guarantees aren't applicable.
//
// 3. Row-major C array convention, regardless of Eigen storage order.
//    All matrix <-> C-array conversions read and write row-major. Column-
//    major Eigen inputs are transposed internally; row-major inputs go
//    through unchanged. Callers don't need to reason about Eigen's default
//    storage order.
//
// 4. Accept any Eigen expression on the output side.
//    Output-side functions take `const Eigen::MatrixBase<Derived>&`, not
//    concrete `Eigen::Matrix`/`Vector`. This admits `Zero()`, `Ones()`,
//    `Constant(...)`, `transpose()`, `block<...>()`, and segment views in
//    addition to plain matrix/vector variables. Internal evaluation to a
//    `PlainObject` handles non-contiguous expressions safely.
//
// 5. Const correctness on inputs.
//    Input parameters are const-qualified (`const ScalarT*`,
//    `const ScalarT (&)[N]`, `const Eigen::MatrixBase<Derived>&`).
//    `const`-qualified C arrays must be acceptable - regression tests in
//    `tests/test_eigenSupport.cpp` enforce this for each input-side
//    function, so removing `const` somewhere will fail to compile.
//
// 6. Fail loudly.
//    Compile-time violations use `static_assert` with a message identifying
//    which constraint failed. Runtime violations on dynamic variants call
//    `std::terminate()` rather than throwing or silently truncating.
//
// =============================================================================

template <class Derived>
inline constexpr bool is_row_major_v = (Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0;

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
template <class Derived, std::size_t size>
void eigenMatrixToCArray(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[size]) {
    static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
                  "Input must be a fixed-size Eigen type.");

    using Scalar = Derived::Scalar;
    constexpr int Rows = Derived::RowsAtCompileTime;
    constexpr int Cols = Derived::ColsAtCompileTime;

    static_assert(static_cast<std::size_t>(Rows) * static_cast<std::size_t>(Cols) == size,
                  "Output array size must equal rows*cols of input.");

    if constexpr ((Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0) {
        Eigen::Matrix<Scalar, Rows, Cols, Eigen::RowMajor> tmp = inMat;
        std::copy(tmp.data(), tmp.data() + tmp.size(), out);
    } else {
        Eigen::Matrix<Scalar, Cols, Rows> tmpT = inMat.transpose();
        std::copy(tmpT.data(), tmpT.data() + tmpT.size(), out);
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
template <class Derived, std::size_t size>
void eigenMatrixXToCArray(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[size]) {
    using Scalar = Derived::Scalar;

    // Runtime capacity check against compile-time size
    if (static_cast<std::size_t>(inMat.size()) > size) {
        std::terminate();
    }

    // Make a contiguous row-major buffer regardless of input layout
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
    std::copy(rm.data(), rm.data() + rm.size(), out);
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

    using Scalar = Derived::Scalar;
    constexpr int R = Derived::RowsAtCompileTime;
    constexpr int C = Derived::ColsAtCompileTime;

    static_assert(static_cast<std::size_t>(R) == N && static_cast<std::size_t>(C) == M,
                  "2D output shape must match input rows x cols.");

    if constexpr ((Eigen::internal::traits<Derived>::Flags & Eigen::RowMajorBit) != 0) {
        Eigen::Matrix<Scalar, R, C, Eigen::RowMajor> tmp = inMat;
        std::copy(tmp.data(), tmp.data() + tmp.size(), &out[0][0]);
    } else {
        Eigen::Matrix<Scalar, C, R> tmpT = inMat.transpose();
        std::copy(tmpT.data(), tmpT.data() + tmpT.size(), &out[0][0]);
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
template <class Derived, std::size_t rows, std::size_t cols>
void eigenMatrixXToCArray2D(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[rows][cols]) {
    using Scalar = Derived::Scalar;

    // Enforce shape at runtime (safer for 2-D indexing)
    if (inMat.rows() != static_cast<int>(rows) || inMat.cols() != static_cast<int>(cols)) {
        std::terminate();
    }

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
    std::copy(rm.data(), rm.data() + rm.size(), &out[0][0]);
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
template <class Derived, std::size_t size>
void eigenMatrixXInsertCArray(const Eigen::MatrixBase<Derived>& inMat,
                              typename Derived::Scalar (&out)[size],
                              std::size_t offset,
                              const std::size_t stride = 1U) {
    using Scalar = Derived::Scalar;

    const auto count = static_cast<std::size_t>(inMat.size());
    if (count == 0U) {
        return;
    }

    if (stride == 0U && count > 1U) {
        std::terminate();
    }

    // Capacity check: last index must be < N
    if (const std::size_t last_index = offset + ((count - 1U) * stride); last_index >= size) {
        std::terminate();
    }

    // Make a contiguous row-major buffer regardless of input layout
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();

    if (stride == 1U) {
        std::copy(rm.data(), rm.data() + rm.size(), out + offset);
    } else {
        const Scalar* src = rm.data();
        std::size_t idx = offset;
        for (std::size_t i = 0U; i < count; ++i) {
            out[idx] = src[i];
            idx += stride;
        }
    }
}

/**
 * @brief Copy a fixed-size Eigen column-vector expression into a C array.
 *
 * Accepts any fixed-size Eigen expression that resolves to a column vector
 * (concrete `Eigen::Vector<T, N>`, `Vector::Zero()`, `block<N, 1>()`, etc.).
 * The destination is a sized C array; its length is enforced at compile time
 * to match the vector length.
 *
 * @tparam Derived Fixed-size Eigen column-vector expression type.
 * @tparam size Extent of the destination array (must equal vector length).
 * @param inVec Vector expression whose contents should be copied.
 * @param out Destination array that receives the entries.
 */
template <class Derived, std::size_t size>
void eigenVectorToCArray(const Eigen::MatrixBase<Derived>& inVec, typename Derived::Scalar (&out)[size]) {
    static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
                  "Input must be a fixed-size Eigen type.");
    static_assert(Derived::ColsAtCompileTime == 1, "Input must be a column vector.");
    static_assert(static_cast<std::size_t>(Derived::RowsAtCompileTime) == size,
                  "Output array size must equal vector length.");

    const typename Derived::PlainObject evaluated = inVec;
    std::copy(evaluated.data(), evaluated.data() + size, out);
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
Eigen::Matrix<ScalarT, rows, cols> cArrayToEigenMatrix(const ScalarT* inArray) {
    return Eigen::Map<const Eigen::Matrix<ScalarT, rows, cols>>(inArray);
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
Eigen::MatrixX<ScalarT> cArrayToEigenMatrixX(const ScalarT* inArray, int nRows, int nCols) {
    Eigen::MatrixX<ScalarT> outMat(nRows, nCols);
    outMat = Eigen::Map<const Eigen::MatrixX<ScalarT>>(inArray, outMat.rows(), outMat.cols());
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
template <typename ScalarT, int size>
Eigen::Vector<ScalarT, size> cArrayToEigenVector(const ScalarT (&inArray)[size]) {
    return Eigen::Map<const Eigen::Vector<ScalarT, size>>(inArray);
}

/**
 * @brief Convert a three-element C array into an Eigen 3-vector.
 *
 * @tparam ScalarT Scalar type of the vector.
 * @param inArray Pointer to three consecutive elements.
 * @return Eigen::Vector3 populated from the input data.
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> cArrayToEigenVector3(const ScalarT* inArray) {
    return Eigen::Map<const Eigen::Vector3<ScalarT>>(inArray);
}

/**
 * @brief Convert a three-element C array into an Eigen modified Rodrigues parameter.
 *
 * @tparam ScalarT Scalar type of the MRP coefficients.
 * @param inArray Pointer to three elements representing the MRP components.
 * @return Eigen::MRP constructed from the input.
 */
template <typename ScalarT>
Eigen::MRP<ScalarT> cArrayToEigenMrp(const ScalarT* inArray) {
    Eigen::MRP<ScalarT> sigma_Eigen;
    sigma_Eigen = Eigen::Map<const Eigen::Vector<ScalarT, 3>>(inArray);

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
Eigen::Matrix3<ScalarT> cArrayToEigenMatrix3(const ScalarT* inArray) {
    return Eigen::Map<const Eigen::Matrix3<ScalarT>>(inArray, 3, 3).transpose();
}

/**
 * @brief Copy a 3×3 C two-dimensional array into an Eigen 3×3 matrix.
 *
 * Both dimensions are enforced at compile time via the array reference
 * parameter, and the input is const-qualified for consistency with the rest
 * of the input-side conversion functions.
 *
 * @tparam ScalarT Scalar type of the matrix.
 * @param in2DArray Source array with bounds `[3][3]`.
 * @return Eigen::Matrix3 containing the same entries.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> c2DArrayToEigenMatrix3(const ScalarT (&in2DArray)[3][3]) {
    return Eigen::Map<const Eigen::Matrix<ScalarT, 3, 3, Eigen::RowMajor>>(&in2DArray[0][0]);
}

/**
 * @brief Convert an Eigen MRP to a three-component vector.
 *
 * @tparam ScalarT Scalar type of the MRP.
 * @param mrp Modified Rodrigues parameter to convert.
 * @return Eigen::Vector3 containing the MRP coefficients.
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> eigenMrpToVector3(const Eigen::MRP<ScalarT>& mrp) {
    return Eigen::Vector3<ScalarT>(mrp.x(), mrp.y(), mrp.z());
}

/**
 * @brief Create the DCM for a positive rotation about the body X-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM1(ScalarT angle) {
    const ScalarT c = std::cos(angle);
    const ScalarT s = std::sin(angle);

    return Eigen::Matrix3<ScalarT>{{ScalarT{1}, ScalarT{0}, ScalarT{0}}, {ScalarT{0}, c, s}, {ScalarT{0}, -s, c}};
}

/**
 * @brief Create the DCM for a positive rotation about the body Y-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM2(ScalarT angle) {
    const ScalarT c = std::cos(angle);
    const ScalarT s = std::sin(angle);

    return Eigen::Matrix3<ScalarT>{{c, ScalarT{0}, -s}, {ScalarT{0}, ScalarT{1}, ScalarT{0}}, {s, ScalarT{0}, c}};
}

/**
 * @brief Create the DCM for a positive rotation about the body Z-axis.
 *
 * @tparam ScalarT Scalar type representing the angle.
 * @param angle Rotation angle in radians.
 * @return Eigen::Matrix3 representing the rotation.
 */
template <typename ScalarT>
Eigen::Matrix3<ScalarT> eigenM3(ScalarT angle) {
    const ScalarT c = std::cos(angle);
    const ScalarT s = std::sin(angle);

    return Eigen::Matrix3<ScalarT>{{c, s, ScalarT{0}}, {-s, c, ScalarT{0}}, {ScalarT{0}, ScalarT{0}, ScalarT{1}}};
}

/**
 * @brief Construct the skew-symmetric matrix such that `[tilde(vec)] * b = vec × b`.
 *
 * Accepts either a fixed-size 3-element column vector (shape checked at
 * compile time via `static_assert`) or a dynamic-size expression that is
 * 3×1 at runtime (shape checked via `std::terminate()`). This mirrors the
 * suite-wide split between fixed and dynamic variants documented at the
 * top of this header.
 *
 * @tparam Derived Eigen 3-vector expression type.
 * @param vec Vector whose associated tilde matrix is requested.
 * @return Eigen::Matrix3 representing the skew-symmetric cross-product matrix.
 */
template <typename Derived>
Eigen::Matrix3<typename Eigen::MatrixBase<Derived>::Scalar> eigenTilde(const Eigen::MatrixBase<Derived>& vec) {
    static_assert((Derived::RowsAtCompileTime == 3 || Derived::RowsAtCompileTime == Eigen::Dynamic) &&
                      (Derived::ColsAtCompileTime == 1 || Derived::ColsAtCompileTime == Eigen::Dynamic),
                  "eigenTilde requires a 3-element column vector (fixed-size or dynamic).");

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic || Derived::ColsAtCompileTime == Eigen::Dynamic) {
        if (vec.rows() != 3 || vec.cols() != 1) {
            std::terminate();
        }
    }

    using Scalar = Eigen::MatrixBase<Derived>::Scalar;

    const Scalar vx = vec(0);
    const Scalar vy = vec(1);
    const Scalar vz = vec(2);

    return Eigen::Matrix3<Scalar>{{Scalar{0}, -vz, vy}, {vz, Scalar{0}, -vx}, {-vy, vx, Scalar{0}}};
}

#endif  // EIGEN_SUPPORT
