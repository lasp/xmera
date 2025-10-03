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
    (Derived::RowsAtCompileTime != Eigen::Dynamic) &&
    (Derived::ColsAtCompileTime != Eigen::Dynamic);

/*! This function provides a general conversion between an Eigen matrix and
an output C array. Note that this routine would convert an inbound type
to a MatrixXd and then transpose the matrix which would be inefficient
in a lot of cases.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy into
*/
template <class Derived, std::size_t Size>
void eigenMatrixToCArray(const Eigen::MatrixBase<Derived>& inMat,
                         typename Derived::Scalar (&out)[Size])
{
  static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::ColsAtCompileTime != Eigen::Dynamic,
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

/*! This function provides a general conversion between a dynamic Eigen matrix and
an output C array. Note that this routine would convert an inbound type
to a MatrixXd and then transpose the matrix which would be inefficient
in a lot of cases.
@return void
@param inMat The source Eigen matrix that we are converting
@param outArray The destination array (sized by the user!) we copy into
*/
template <class Derived, std::size_t Size>
void eigenMatrixXToCArray(const Eigen::MatrixBase<Derived>& inMat, typename Derived::Scalar (&out)[Size])
{
  using Scalar = typename Derived::Scalar;

  // Runtime capacity check against compile-time size
  if (static_cast<std::size_t>(inMat.size()) > Size) {
    std::terminate();  // or throw, or your project’s fatal handler
  }

  // Make a contiguous row-major buffer regardless of input layout
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
  const auto count = static_cast<std::size_t>(inMat.size());
  std::memcpy(out, rm.data(), count * sizeof(Scalar));
}


template <class Derived, std::size_t N, std::size_t M>
void eigenMatrixToCArray2D(const Eigen::MatrixBase<Derived>& inMat,
                           typename Derived::Scalar (&out)[N][M])
{
  static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic &&
                Derived::ColsAtCompileTime != Eigen::Dynamic,
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

template <class Derived, std::size_t Rows, std::size_t Cols>
void eigenMatrixXToCArray2D(const Eigen::MatrixBase<Derived>& inMat,
                            typename Derived::Scalar (&out)[Rows][Cols])
{
  using Scalar = typename Derived::Scalar;

  // Enforce shape at runtime (safer for 2-D indexing)
  if (inMat.rows() != static_cast<int>(Rows) || inMat.cols() != static_cast<int>(Cols)) {
    std::terminate();
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rm = inMat.derived();
  std::memcpy(&out[0][0], rm.data(), Rows * Cols * sizeof(Scalar));
}

/*! This function Writes the matrix flattened in row-major order into `out[offset + i*stride]`
 * @return void
 * @param inMat The source Eigen matrix that we are converting
 * @param out The destination array (sized by the user)
 * @param offset The offset at which writing into the array starts
 * @param stride The stride between elements in the array (default 1)
*/
template <class Derived, std::size_t Size>
void eigenMatrixXInsertCArray(const Eigen::MatrixBase<Derived>& inMat,
                              typename Derived::Scalar (&out)[Size],
                              std::size_t offset,
                              const std::size_t stride = 1)
{
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

/*! This function provides a direct conversion between an Eigen vector and an
output C array. We are providing this function to save on the inline conversion
and the transpose that would have been performed by the general case.
@return void
@param inVec The source Eigen matrix that we are converting
@param outArray The destination array we copy into
*/
template <typename ScalarT, int size>
void eigenVectorToCArray(const Eigen::Vector<ScalarT, size> &inVec, ScalarT *outArray) {
    memcpy(outArray, inVec.data(), size * sizeof(ScalarT));
}

/*! This function performs the general conversion between an input C array
and an Eigen matrix. Note that to use this function the user MUST size
the Eigen matrix ahead of time so that the internal map call has enough
information to ingest the C array.
@return Eigen::Matrix
@param inArray The input array (row-major)
*/
template <typename ScalarT, int rows, int cols>
Eigen::Matrix<ScalarT, rows, cols> cArrayAsEigenMatrix(ScalarT *inArray) {
    Eigen::Matrix<ScalarT, rows, cols> outMat;
    outMat = Eigen::Map<Eigen::Matrix<ScalarT, rows, cols>>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/*! This function performs the general conversion between an input C array
and a dynamic Eigen matrix. Note that to use this function the user MUST size
the Eigen matrix ahead of time so that the internal map call has enough
information to ingest the C array.
@return Eigen::MatrixX
@param inArray The input array (row-major)
*/
template <typename ScalarT>
Eigen::MatrixX<ScalarT> cArrayAsEigenMatrixX(ScalarT *inArray, int nRows, int nCols) {
    Eigen::MatrixX<ScalarT> outMat;
    outMat.resize(nRows, nCols);
    outMat = Eigen::Map<Eigen::MatrixX<ScalarT>>(inArray, outMat.rows(), outMat.cols());
    return outMat;
}

/*! This function performs the conversion between an input C array and an output Eigen vector. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Vector
@param inArray The input array (row-major)
*/
template <typename ScalarT, std::size_t size>
Eigen::Vector<ScalarT, size> cArrayAsEigenVector(ScalarT (&inArray)[size]) {
    return Eigen::Map<Eigen::Vector<ScalarT, size>>(inArray);
}

/*! This function performs the conversion between an input C array and an output Eigen vector3. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Vector
@param inArray The input array (row-major)
*/
template <typename ScalarT>
Eigen::Vector3<ScalarT> cArrayAsEigenVector3(ScalarT *inArray) {
    return Eigen::Map<Eigen::Vector3<ScalarT>>(inArray);
}

/*! This function performs the conversion between an input C array
and an output Eigen MRP. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::MRP
@param inArray The input array (row-major)
*/
template <typename ScalarT>
Eigen::MRP<ScalarT> cArrayAsEigenMrp(ScalarT *inArray) {
    Eigen::MRP<ScalarT> sigma_Eigen;
    sigma_Eigen = Eigen::Map<Eigen::Vector<ScalarT, 3>>(inArray);

    return sigma_Eigen;
}

/*! This function performs the conversion between an input C array
and an Eigen 3x3 matrix. This function is provided
in order to save an unnecessary conversion between types.
@return Eigen::Matrix3
@param inArray The input array (row-major)
*/
template <typename ScalarT>
Eigen::Matrix3<ScalarT> cArrayAsEigenMatrix3(ScalarT *inArray) {
    return Eigen::Map<Eigen::Matrix3<ScalarT>>(inArray, 3, 3).transpose();
}

/*! This function performs the conversion between an input C 3x3
2D-array and an output Eigen Matrix3. This function is provided
in order to save an unnecessary conversion between types
@return Eigen::Matrix3
@param in2DArray The input 2D-array
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

/*! This function converts the Eigen MRP to Vector3
 @return Eigen::Vector3
 @param mrp The input Vector3 variable
 */
template <typename ScalarT>
Eigen::Vector3<ScalarT> eigenMrpToVector3(const Eigen::MRP<ScalarT> mrp) {
    Eigen::Vector3<ScalarT> vec3;

    vec3[0] = mrp.x();
    vec3[1] = mrp.y();
    vec3[2] = mrp.z();

    return vec3;
}

/*! This function returns the Eigen DCM that corresponds to a 1-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3
 @param angle The input rotation angle
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

/*! This function returns the Eigen DCM that corresponds to a 2-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3
 @param angle The input rotation angle
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

/*! This function returns the Eigen DCM that corresponds to a 3-axis rotation
 by the angle theta.  The DCM is the positive theta rotation from the original
 frame to the final frame.
 @return Eigen::Matrix3
 @param angle The input rotation angle
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

/*! This function returns the tilde matrix version of a vector. The tilde
 matrix is the matrix equivalent of a vector cross product, where
 [tilde_a] b == a x b
 @return Eigen::Matrix3
 @param vec The input vector
 */
template <typename Derived>
Eigen::Matrix3<typename Eigen::MatrixBase<Derived>::Scalar> eigenTilde(const Eigen::MatrixBase<Derived> &vec) {
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

#endif // EIGENSUPPORT
