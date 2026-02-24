// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
#include "architecture/utilities/linearAlgebra.h"
#include "unitTestComparators.h"
}

namespace {
constexpr double kAccuracy = 1e-10;
}

TEST(LinearAlgebraVectors, GeneralOperations) {
    double v2_0[2] = {};
    double v2_1[2] = {};
    double v3_0[3] = {};
    double v3_1[3] = {};
    double v3_2[3] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};
    double a = 0.0;

    v3Set(4, 5, 16, v3_0);
    vCopy(v3_0, 3, v3_1);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "vCopy";

    v3Set(0, 0, 0, v3_0);
    vSetZero(v3_1, 3);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "vSetZero";

    v3Set(1, 2, 3, v3_0);
    v3Set(4, 5, 6, v3_1);
    v3Set(5, 7, 9, v3_2);
    vAdd(v3_0, 3, v3_1, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_2, kAccuracy)) << "vAdd";

    v3Set(4, 6, 8, v3_0);
    v3Set(1, 2, 3, v3_1);
    v3Set(3, 4, 5, v3_2);
    vSubtract(v3_0, 3, v3_1, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_2, kAccuracy)) << "vSubtract";

    v3Set(1, 2, 3, v3_0);
    v3Set(3, 6, 9, v3_2);
    vScale(3, v3_0, 3, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_2, kAccuracy)) << "vScale";

    v3Set(1, 2, 3, v3_1);
    v3Set(4, 5, 6, v3_2);
    a = vDot(v3_1, 3, v3_2);
    EXPECT_TRUE(isEqual(a, 32.0, kAccuracy)) << "vDot";

    v3Set(1, 2, 3, v3_0);
    v3Set(4, 5, 6, v3_1);
    m33Set(4, 5, 6, 8, 10, 12, 12, 15, 18, m33_1);
    vOuterProduct(v3_0, 3, v3_1, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "vOuterProduct";

    v3Set(1, 2, 3, v3_0);
    m33Set(4, 5, 6, 7, 8, 9, 10, 11, 12, m33_0);
    v3Set(48, 54, 60, v3_1);
    vtMultM(v3_0, m33_0, 3, 3, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "vtMultM";

    v3Set(1, 2, 3, v3_0);
    m33Set(4, 5, 6, 7, 8, 9, 10, 11, 12, m33_0);
    v3Set(32, 50, 68, v3_1);
    vtMultMt(v3_0, m33_0, 3, 3, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "vtMultMt";

    v3Set(1, 2, 3, v3_0);
    a = vNorm(v3_0, 3);
    EXPECT_TRUE(isEqual(a, 3.74165738677394, kAccuracy)) << "vNorm";

    v3Set(1, 2, 3, v3_0);
    v3Set(0.267261241912424, 0.534522483824849, 0.801783725737273, v3_1);
    vNormalize(v3_0, 3, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "vNormalize";

    v2_0[0] = 1;
    v2_0[1] = 2;
    v2Set(1, 2, v2_1);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_1, kAccuracy)) << "v2IsEqual";
}

TEST(LinearAlgebraVectors, Vector2Operations) {
    double v2_0[2] = {};
    double v2_1[2] = {};
    double v2_2[2] = {};

    v2Set(1, 2, v2_0);
    v2Copy(v2_0, v2_1);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_1, kAccuracy)) << "v2Copy";

    v2Set(0, 0, v2_0);
    v2SetZero(v2_1);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_1, kAccuracy)) << "v2SetZero";

    v2Set(1, 2, v2_0);
    v2Set(4, 5, v2_1);
    double a = v2Dot(v2_0, v2_1);
    EXPECT_TRUE(isEqual(a, 14.0, kAccuracy)) << "v2Dot";

    v2Set(1, 2, v2_0);
    v2Set(4, 5, v2_1);
    v2Set(5, 7, v2_2);
    v2Add(v2_0, v2_1, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_2, kAccuracy)) << "v2Add";

    v2Set(4, 6, v2_0);
    v2Set(1, 2, v2_1);
    v2Set(3, 4, v2_2);
    v2Subtract(v2_0, v2_1, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_2, kAccuracy)) << "v2Subtract";

    v2Set(3, 4, v2_0);
    double norm = v2Norm(v2_0);
    EXPECT_TRUE(isEqual(norm, 5.0, kAccuracy)) << "v2Norm";

    v2Set(1, 2, v2_0);
    v2Set(3, 6, v2_2);
    v2Scale(3, v2_0, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_2, kAccuracy)) << "v2Scale";

    v2Set(1, 1, v2_0);
    v2Set(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0), v2_2);
    v2Normalize(v2_0, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_2, kAccuracy)) << "v2Normalize";
}

TEST(LinearAlgebraVectors, Vector3BasicOperations) {
    double v3_0[3] = {};
    double v3_1[3] = {};
    double v3_2[3] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};
    double a = 0.0;

    v3_0[0] = 1;
    v3_0[1] = 2;
    v3_0[2] = 3;
    v3Set(1, 2, 3, v3_1);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3Set";

    v3Set(4, 5, 16, v3_0);
    v3Copy(v3_0, v3_1);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3Copy";

    v3Set(0, 0, 0, v3_0);
    v3SetZero(v3_1);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3SetZero";

    v3Set(1, 2, 3, v3_0);
    v3Set(4, 5, 6, v3_1);
    v3Set(5, 7, 9, v3_2);
    v3Add(v3_0, v3_1, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_2, kAccuracy)) << "v3Add";

    v3Set(4, 6, 8, v3_0);
    v3Set(1, 2, 3, v3_1);
    v3Set(3, 4, 5, v3_2);
    vSubtract(v3_0, 3, v3_1, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_2, kAccuracy)) << "vSubtract";

    v3Set(1, 2, 3, v3_0);
    v3Set(3, 6, 9, v3_2);
    v3Scale(3, v3_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_2, kAccuracy)) << "v3Scale";

    v3Set(1, 2, 3, v3_1);
    v3Set(4, 5, 6, v3_2);
    a = v3Dot(v3_1, v3_2);
    EXPECT_TRUE(isEqual(a, 32.0, kAccuracy)) << "v3Dot";

    v3Set(1, 2, 3, v3_0);
    v3Set(4, 5, 6, v3_1);
    m33Set(4, 5, 6, 8, 10, 12, 12, 15, 18, m33_1);
    v3OuterProduct(v3_0, v3_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "v3OuterProduct";

    v3Set(1, 2, 3, v3_0);
    m33Set(4, 5, 6, 7, 8, 9, 10, 11, 12, m33_0);
    v3Set(48, 54, 60, v3_1);
    v3tMultM33(v3_0, m33_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3tMultM33";

    v3Set(1, 2, 3, v3_0);
    m33Set(4, 5, 6, 7, 8, 9, 10, 11, 12, m33_0);
    v3Set(32, 50, 68, v3_1);
    v3tMultM33t(v3_0, m33_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3tMultM33t";

    v3Set(1, 2, 3, v3_0);
    a = v3Norm(v3_0);
    EXPECT_TRUE(isEqual(a, 3.74165738677394, kAccuracy)) << "v3Norm";

    v3Set(1, 2, 3, v3_0);
    v3Set(0.267261241912424, 0.534522483824849, 0.801783725737273, v3_1);
    v3Normalize(v3_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3Normalize";
}

TEST(LinearAlgebraVectors, Vector3AdvancedOperations) {
    double v3_0[3] = {};
    double v3_1[3] = {};
    double v3_2[3] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};

    v3Set(1, 2, 3, v3_0);
    v3Set(4, 5, 6, v3_1);
    v3Set(-3, 6, -3, v3_2);
    v3Cross(v3_0, v3_1, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_2, kAccuracy)) << "v3Cross";

    v3Set(2, 1, 1, v3_0);
    v3Set(-1, 1, 1, v3_1);
    v3Normalize(v3_1, v3_1);
    v3Perpendicular(v3_0, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_2, v3_1, kAccuracy)) << "v3Perpendicular";

    v3Set(1, 2, 3, v3_0);
    v3Tilde(v3_0, m33_0);
    m33Set(0, -3, 2, 3, 0, -1, -2, 1, 0, m33_1);
    EXPECT_TRUE(m33IsEqual(m33_1, m33_0, kAccuracy)) << "v3Tilde";

    const double permutations[][3] = {
        {1, 2, 3},
        {1, 3, 2},
        {2, 1, 3},
        {2, 3, 1},
        {3, 1, 2},
        {3, 2, 1},
    };

    for (const auto& perm : permutations) {
        v3Set(perm[0], perm[1], perm[2], v3_0);
        v3Sort(v3_0, v3_0);
        v3Set(3, 2, 1, v3_1);
        EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "v3Sort";
    }
}

TEST(LinearAlgebraVectors, Vector4Operations) {
    double v4_0[4] = {};
    double v4_1[4] = {};
    double v4_2[4] = {};

    v4_0[0] = 1;
    v4_0[1] = 2;
    v4_0[2] = 3;
    v4_0[3] = 4;
    v4Set(1, 2, 3, 4, v4_1);
    EXPECT_TRUE(v4IsEqual(v4_0, v4_1, kAccuracy)) << "v4Set";

    v4Set(4, 5, 16, 22, v4_0);
    v4Copy(v4_0, v4_1);
    EXPECT_TRUE(v4IsEqual(v4_0, v4_1, kAccuracy)) << "v4Copy";

    v4Set(0, 0, 0, 0, v4_0);
    v4SetZero(v4_1);
    EXPECT_TRUE(v4IsEqual(v4_0, v4_1, kAccuracy)) << "v4SetZero";

    v4Set(1, 2, 3, 4, v4_1);
    v4Set(4, 5, 6, 7, v4_2);
    double a = v4Dot(v4_1, v4_2);
    EXPECT_TRUE(isEqual(a, 60.0, kAccuracy)) << "v4Dot";

    v4Set(1, 2, 3, 4, v4_0);
    double norm = v4Norm(v4_0);
    EXPECT_TRUE(isEqual(norm, 5.47722557505166, kAccuracy)) << "v4Norm";
}

TEST(LinearAlgebraMatrices, GeneralOperations) {
    double v2_0[2] = {};
    double v2_1[2] = {};
    double v3_0[3] = {};
    double v3_1[3] = {};
    double v4_0[4] = {};
    double v4_1[4] = {};
    double m23_0[2][3] = {};
    double m23_1[2][3] = {};
    double m24_0[2][4] = {};
    double m24_1[2][4] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};
    double m33_2[3][3] = {};
    double m34_0[3][4] = {};
    double m42_0[4][2] = {};
    double m42_1[4][2] = {};
    double m43_0[4][3] = {};
    double m43_1[4][3] = {};
    double a = 0.0;

    m33_0[0][0] = 1;
    m33_0[0][1] = 2;
    m33_0[0][2] = 3;
    m33_0[1][0] = 4;
    m33_0[1][1] = 5;
    m33_0[1][2] = 6;
    m33_0[2][0] = 7;
    m33_0[2][1] = 8;
    m33_0[2][2] = 9;
    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_1);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mIsEqual";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    mCopy(m33_0, 3, 3, m33_1);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mCopy";

    m33Set(0, 0, 0, 0, 0, 0, 0, 0, 0, m33_0);
    mSetZero(m33_1, 3, 3);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mSetZero";

    m33Set(1, 0, 0, 0, 1, 0, 0, 0, 1, m33_0);
    mSetIdentity(m33_1, 3, 3);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mSetIdentity";

    m33Set(1, 0, 0, 0, 2, 0, 0, 0, 3, m33_0);
    v3Set(1, 2, 3, v3_0);
    mDiag(v3_0, 3, m33_1);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mDiag";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(1, 4, 7, 2, 5, 8, 3, 6, 9, m33_1);
    mTranspose(m33_0, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mTranspose square";

    m24_0[0][0] = 1;
    m24_0[0][1] = 2;
    m24_0[0][2] = 3;
    m24_0[0][3] = 4;
    m24_0[1][0] = 5;
    m24_0[1][1] = 6;
    m24_0[1][2] = 7;
    m24_0[1][3] = 8;

    m42_0[0][0] = 1;
    m42_0[0][1] = 5;
    m42_0[1][0] = 2;
    m42_0[1][1] = 6;
    m42_0[2][0] = 3;
    m42_0[2][1] = 7;
    m42_0[3][0] = 4;
    m42_0[3][1] = 8;
    mTranspose(m24_0, 2, 4, m42_1);
    EXPECT_TRUE(mIsEqual(m42_0, 4, 2, m42_1, kAccuracy)) << "mTranspose rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(11, 13, 15, 17, 19, 21, 23, 25, 27, m33_2);
    mAdd(m33_0, 3, 3, m33_1, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mAdd";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(-9, -9, -9, -9, -9, -9, -9, -9, -9, m33_2);
    mSubtract(m33_0, 3, 3, m33_1, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mSubtract";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(2, 4, 6, 8, 10, 12, 14, 16, 18, m33_1);
    mScale(2, m33_0, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mScale";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(84, 90, 96, 201, 216, 231, 318, 342, 366, m33_2);
    mMultM(m33_0, 3, 3, m33_1, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mMultM square";

    m43_0[0][0] = 1;
    m43_0[0][1] = 5;
    m43_0[0][2] = 9;
    m43_0[1][0] = 2;
    m43_0[1][1] = 6;
    m43_0[1][2] = 10;
    m43_0[2][0] = 3;
    m43_0[2][1] = 7;
    m43_0[2][2] = 11;
    m43_0[3][0] = 4;
    m43_0[3][1] = 8;
    m43_0[3][2] = 12;

    m23_0[0][0] = 30;
    m23_0[0][1] = 70;
    m23_0[0][2] = 110;
    m23_0[1][0] = 70;
    m23_0[1][1] = 174;
    m23_0[1][2] = 278;
    mMultM(m24_0, 2, 4, m43_0, 4, 3, m23_1);
    EXPECT_TRUE(mIsEqual(m23_1, 2, 3, m23_0, kAccuracy)) << "mMultM rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(174, 186, 198, 213, 228, 243, 252, 270, 288, m33_2);
    mtMultM(m33_0, 3, 3, m33_1, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mtMultM";

    m24_0[0][0] = 1;
    m24_0[0][1] = 2;
    m24_0[0][2] = 3;
    m24_0[0][3] = 4;
    m24_0[1][0] = 5;
    m24_0[1][1] = 6;
    m24_0[1][2] = 7;
    m24_0[1][3] = 8;

    m23_0[0][0] = 1;
    m23_0[0][1] = 2;
    m23_0[0][2] = 3;
    m23_0[1][0] = 4;
    m23_0[1][1] = 5;
    m23_0[1][2] = 6;

    m43_0[0][0] = 21;
    m43_0[0][1] = 27;
    m43_0[0][2] = 33;
    m43_0[1][0] = 26;
    m43_0[1][1] = 34;
    m43_0[1][2] = 42;
    m43_0[2][0] = 31;
    m43_0[2][1] = 41;
    m43_0[2][2] = 51;
    m43_0[3][0] = 36;
    m43_0[3][1] = 48;
    m43_0[3][2] = 60;
    mtMultM(m24_0, 2, 4, m23_0, 2, 3, m43_1);
    EXPECT_TRUE(mIsEqual(m43_0, 4, 3, m43_1, kAccuracy)) << "mtMultM rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(68, 86, 104, 167, 212, 257, 266, 338, 410, m33_2);
    mMultMt(m33_0, 3, 3, m33_1, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mMultMt";

    m23_0[0][0] = 1;
    m23_0[0][1] = 2;
    m23_0[0][2] = 3;
    m23_0[1][0] = 4;
    m23_0[1][1] = 5;
    m23_0[1][2] = 6;

    m43_0[0][0] = 1;
    m43_0[0][1] = 5;
    m43_0[0][2] = 9;
    m43_0[1][0] = 2;
    m43_0[1][1] = 6;
    m43_0[1][2] = 10;
    m43_0[2][0] = 3;
    m43_0[2][1] = 7;
    m43_0[2][2] = 11;
    m43_0[3][0] = 4;
    m43_0[3][1] = 8;
    m43_0[3][2] = 12;

    m24_0[0][0] = 38;
    m24_0[0][1] = 44;
    m24_0[0][2] = 50;
    m24_0[0][3] = 56;
    m24_0[1][0] = 83;
    m24_0[1][1] = 98;
    m24_0[1][2] = 113;
    m24_0[1][3] = 128;

    mMultMt(m23_0, 2, 3, m43_0, 4, 3, m24_1);
    EXPECT_TRUE(mIsEqual(m24_0, 2, 4, m24_1, kAccuracy)) << "mMultMt rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(138, 174, 210, 171, 216, 261, 204, 258, 312, m33_2);
    mtMultMt(m33_0, 3, 3, m33_1, 3, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_2, kAccuracy)) << "mtMultMt";

    double m32_0[3][2] = {};
    m32_0[0][0] = 1;
    m32_0[0][1] = 2;
    m32_0[1][0] = 3;
    m32_0[1][1] = 4;
    m32_0[2][0] = 5;
    m32_0[2][1] = 6;

    m43_0[0][0] = 1;
    m43_0[0][1] = 2;
    m43_0[0][2] = 3;
    m43_0[1][0] = 4;
    m43_0[1][1] = 5;
    m43_0[1][2] = 6;
    m43_0[2][0] = 7;
    m43_0[2][1] = 8;
    m43_0[2][2] = 9;
    m43_0[3][0] = 10;
    m43_0[3][1] = 11;
    m43_0[3][2] = 12;

    m24_0[0][0] = 22;
    m24_0[0][1] = 49;
    m24_0[0][2] = 76;
    m24_0[0][3] = 103;
    m24_0[1][0] = 28;
    m24_0[1][1] = 64;
    m24_0[1][2] = 100;
    m24_0[1][3] = 136;

    mtMultMt(m32_0, 3, 2, m43_0, 4, 3, m24_1);
    EXPECT_TRUE(mIsEqual(m24_0, 2, 4, m24_1, kAccuracy)) << "mtMultMt rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    v3Set(2, 3, 4, v3_0);
    v3Set(20, 47, 74, v3_1);
    mMultV(m33_0, 3, 3, v3_0, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "mMultV square";

    m23_0[0][0] = 1;
    m23_0[0][1] = 2;
    m23_0[0][2] = 3;
    m23_0[1][0] = 4;
    m23_0[1][1] = 5;
    m23_0[1][2] = 6;
    v3Set(2, 3, 4, v3_0);
    v2_1[0] = 20;
    v2_1[1] = 47;
    mMultV(m23_0, 2, 3, v3_0, v2_0);
    EXPECT_TRUE(vIsEqual(v2_0, 2, v2_1, kAccuracy)) << "mMultV rectangular";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    v3Set(2, 3, 4, v3_0);
    v3Set(42, 51, 60, v3_1);
    mtMultV(m33_0, 3, 3, v3_0, v3_0);
    EXPECT_TRUE(vIsEqual(v3_0, 3, v3_1, kAccuracy)) << "mtMultV square";

    m34_0[0][0] = 1;
    m34_0[0][1] = 2;
    m34_0[0][2] = 3;
    m34_0[0][3] = 4;
    m34_0[1][0] = 5;
    m34_0[1][1] = 6;
    m34_0[1][2] = 7;
    m34_0[1][3] = 8;
    m34_0[2][0] = 9;
    m34_0[2][1] = 10;
    m34_0[2][2] = 11;
    m34_0[2][3] = 12;
    v3Set(2, 3, 4, v3_0);
    v4Set(53, 62, 71, 80, v4_0);
    mtMultV(m34_0, 3, 4, v3_0, v4_1);
    EXPECT_TRUE(vIsEqual(v4_0, 4, v4_1, kAccuracy)) << "mtMultV rectangular";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    a = mTrace(m33_0, 3);
    EXPECT_TRUE(isEqual(a, 32.0, kAccuracy)) << "mTrace";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    a = mDeterminant(m33_0, 3);
    EXPECT_TRUE(isEqual(a, 500.0, kAccuracy)) << "mDeterminant";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    m33Set(-0.3, 0.0, 0.1, 0.68, -0.12, -0.08, -0.2, 0.1, 0.0, m33_1);
    mInverse(m33_0, 3, m33_0);
    EXPECT_TRUE(mIsEqual(m33_0, 3, 3, m33_1, kAccuracy)) << "mInverse 3x3";

    double m44_0[4][4] = {};
    double m44_1[4][4] = {};
    m44_0[0][0] = 4;
    m44_0[0][1] = 5;
    m44_0[0][2] = 6;
    m44_0[0][3] = 7;
    m44_0[1][0] = 8;
    m44_0[1][1] = 10;
    m44_0[1][2] = 22;
    m44_0[1][3] = 36;
    m44_0[2][0] = 22;
    m44_0[2][1] = 15;
    m44_0[2][2] = 18;
    m44_0[2][3] = 15;
    m44_0[3][0] = 1;
    m44_0[3][1] = 2;
    m44_0[3][2] = 3;
    m44_0[3][3] = 4;

    m44_1[0][0] = 11.0 / 40.0;
    m44_1[0][1] = 3.0 / 40.0;
    m44_1[0][2] = 1.0 / 40.0;
    m44_1[0][3] = -5.0 / 4.0;
    m44_1[1][0] = 169.0 / 120.0;
    m44_1[1][1] = -1.0 / 40.0;
    m44_1[1][2] = -7.0 / 40.0;
    m44_1[1][3] = -19.0 / 12.0;
    m44_1[2][0] = -277.0 / 120.0;
    m44_1[2][1] = -7.0 / 40.0;
    m44_1[2][2] = 11.0 / 40.0;
    m44_1[2][3] = 55.0 / 12.0;
    m44_1[3][0] = 23.0 / 24.0;
    m44_1[3][1] = 1.0 / 8.0;
    m44_1[3][2] = -1.0 / 8.0;
    m44_1[3][3] = -25.0 / 12.0;

    mInverse(m44_0, 4, m44_0);
    EXPECT_TRUE(mIsEqual(m44_0, 4, 4, m44_1, kAccuracy)) << "mInverse 4x4";
}

TEST(LinearAlgebraMatrices, Matrix22Operations) {
    double v2_0[2] = {};
    double v2_1[2] = {};
    double m22_0[2][2] = {};
    double m22_1[2][2] = {};
    double m22_2[2][2] = {};
    double a = 0.0;

    m22_0[0][0] = 1;
    m22_0[0][1] = 2;
    m22_0[1][0] = 3;
    m22_0[1][1] = 4;
    m22Set(1, 2, 3, 4, m22_1);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22Set";

    m22Set(1, 2, 3, 4, m22_0);
    m22Copy(m22_0, m22_1);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22Copy";

    m22Set(0, 0, 0, 0, m22_0);
    m22SetZero(m22_1);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22SetZero";

    m22Set(1, 0, 0, 1, m22_0);
    m22SetIdentity(m22_1);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22SetIdentity";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(1, 3, 2, 4, m22_1);
    m22Transpose(m22_0, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22Transpose";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(10, 11, 12, 13, m22_1);
    m22Set(11, 13, 15, 17, m22_2);
    m22Add(m22_0, m22_1, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_2, kAccuracy)) << "m22Add";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(10, 11, 12, 13, m22_1);
    m22Set(-9, -9, -9, -9, m22_2);
    m22Subtract(m22_0, m22_1, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_2, kAccuracy)) << "m22Subtract";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(2, 4, 6, 8, m22_1);
    m22Scale(2, m22_0, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22Scale";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(10, 11, 12, 13, m22_1);
    m22Set(34, 37, 78, 85, m22_2);
    m22MultM22(m22_0, m22_1, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_2, kAccuracy)) << "m22MultM22";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(10, 11, 12, 13, m22_1);
    m22Set(46, 50, 68, 74, m22_2);
    m22tMultM22(m22_0, m22_1, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_2, kAccuracy)) << "m22tMultM22";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(10, 11, 12, 13, m22_1);
    m22Set(32, 38, 74, 88, m22_2);
    m22MultM22t(m22_0, m22_1, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_2, kAccuracy)) << "m22MultM22t";

    m22Set(1, 2, 3, 4, m22_0);
    v2Set(2, 3, v2_0);
    v2Set(8, 18, v2_1);
    m22MultV2(m22_0, v2_0, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_1, kAccuracy)) << "m22MultV2";

    m22Set(1, 2, 3, 4, m22_0);
    v2Set(2, 3, v2_0);
    v2Set(11, 16, v2_1);
    m22tMultV2(m22_0, v2_0, v2_0);
    EXPECT_TRUE(v2IsEqual(v2_0, v2_1, kAccuracy)) << "m22tMultV2";

    m22Set(1, 2, 3, 4, m22_0);
    a = m22Trace(m22_0);
    EXPECT_TRUE(isEqual(a, 5.0, kAccuracy)) << "m22Trace";

    m22Set(1, 2, 3, 4, m22_0);
    a = m22Determinant(m22_0);
    EXPECT_TRUE(isEqual(a, -2.0, kAccuracy)) << "m22Determinant";

    m22Set(1, 2, 3, 4, m22_0);
    m22Set(-2.0, 1.0, 1.5, -0.5, m22_1);
    m22Inverse(m22_0, m22_0);
    EXPECT_TRUE(m22IsEqual(m22_0, m22_1, kAccuracy)) << "m22Inverse";
}

TEST(LinearAlgebraMatrices, Matrix33Operations) {
    double v3_0[3] = {};
    double v3_1[3] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};
    double m33_2[3][3] = {};
    double a = 0.0;

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Copy(m33_0, m33_1);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33Copy";

    m33Set(0, 0, 0, 0, 0, 0, 0, 0, 0, m33_0);
    m33SetZero(m33_1);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33SetZero";

    m33Set(1, 0, 0, 0, 1, 0, 0, 0, 1, m33_0);
    m33SetIdentity(m33_1);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33SetIdentity";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(1, 4, 7, 2, 5, 8, 3, 6, 9, m33_1);
    m33Transpose(m33_0, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33Transpose";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(11, 13, 15, 17, 19, 21, 23, 25, 27, m33_2);
    m33Add(m33_0, m33_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_2, kAccuracy)) << "m33Add";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(-9, -9, -9, -9, -9, -9, -9, -9, -9, m33_2);
    m33Subtract(m33_0, m33_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_2, kAccuracy)) << "m33Subtract";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(2, 4, 6, 8, 10, 12, 14, 16, 18, m33_1);
    m33Scale(2, m33_0, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33Scale";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(84, 90, 96, 201, 216, 231, 318, 342, 366, m33_2);
    m33MultM33(m33_0, m33_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_2, kAccuracy)) << "m33MultM33";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(174, 186, 198, 213, 228, 243, 252, 270, 288, m33_2);
    m33tMultM33(m33_0, m33_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_2, kAccuracy)) << "m33tMultM33";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    m33Set(10, 11, 12, 13, 14, 15, 16, 17, 18, m33_1);
    m33Set(68, 86, 104, 167, 212, 257, 266, 338, 410, m33_2);
    m33MultM33t(m33_0, m33_1, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_2, kAccuracy)) << "m33MultM33t";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    v3Set(2, 3, 4, v3_0);
    v3Set(20, 47, 74, v3_1);
    m33MultV3(m33_0, v3_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "m33MultV3";

    m33Set(1, 2, 3, 4, 5, 6, 7, 8, 9, m33_0);
    v3Set(2, 3, 4, v3_0);
    v3Set(42, 51, 60, v3_1);
    m33tMultV3(m33_0, v3_0, v3_0);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "m33tMultV3";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    a = m33Trace(m33_0);
    EXPECT_TRUE(isEqual(a, 32.0, kAccuracy)) << "m33Trace";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    a = m33Determinant(m33_0);
    EXPECT_TRUE(isEqual(a, 500.0, kAccuracy)) << "m33Determinant";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    v3Set(40.7786093479462, 9.66938737160798, 1.26805658603441, v3_0);
    m33SingularValues(m33_0, v3_1);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "m33SingularValues";

    m33Set(4, 5, 6, 7, 8, 9, 22, 15, 20, m33_0);
    v3Set(32.879131511069, 0.695395691157217, -1.5745272022262, v3_0);
    m33EigenValues(m33_0, v3_1);
    EXPECT_TRUE(v3IsEqual(v3_0, v3_1, kAccuracy)) << "m33EigenValues";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    a = m33ConditionNumber(m33_0);
    EXPECT_TRUE(isEqual(a, 32.1583514466598, kAccuracy)) << "m33ConditionNumber";

    m33Set(4, 5, 6, 8, 10, 22, 22, 15, 18, m33_0);
    m33Set(-0.3, 0.0, 0.1, 0.68, -0.12, -0.08, -0.2, 0.1, 0.0, m33_1);
    m33Inverse(m33_0, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m33Inverse";
}

TEST(LinearAlgebraMatrices, Matrix44Operations) {
    double v4_0[4] = {};
    double v4_1[4] = {};
    double m44_0[4][4] = {};
    double m44_1[4][4] = {};
    double a = 0.0;

    m44Set(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, m44_0);
    m44Copy(m44_0, m44_1);
    EXPECT_TRUE(m44IsEqual(m44_0, m44_1, kAccuracy)) << "m44Copy";

    m44Set(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, m44_0);
    m44SetZero(m44_1);
    EXPECT_TRUE(m44IsEqual(m44_0, m44_1, kAccuracy)) << "m44SetZero";

    m44Set(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, m44_0);
    v4Set(2, 3, 4, 5, v4_0);
    v4Set(40, 96, 152, 208, v4_1);
    m44MultV4(m44_0, v4_0, v4_0);
    EXPECT_TRUE(v4IsEqual(v4_0, v4_1, kAccuracy)) << "m44MultV4";

    m44Set(4, 5, 6, 7, 8, 10, 22, 24, 22, 15, 18, 19, 1, 4, 9, 3, m44_0);
    a = m44Determinant(m44_0);
    EXPECT_TRUE(isEqual(a, -3620.0, kAccuracy)) << "m44Determinant";

    m44Set(4, 5, 6, 7, 8, 10, 22, 24, 22, 15, 18, 19, 1, 4, 9, 3, m44_0);
    m44Set(-0.282872928176796,
           0.0116022099447514,
           0.0939226519337017,
           -0.0276243093922652,
           0.649171270718232,
           -0.140883977900553,
           -0.069060773480663,
           0.0497237569060773,
           -0.285635359116022,
           0.0419889502762431,
           0.0303867403314917,
           0.138121546961326,
           0.0856353591160222,
           0.0580110497237569,
           -0.0303867403314917,
           -0.138121546961326,
           m44_1);
    m44Inverse(m44_0, m44_0);
    EXPECT_TRUE(m44IsEqual(m44_0, m44_1, kAccuracy)) << "m44Inverse";
}

TEST(LinearAlgebraMatrices, Matrix66Operations) {
    double v6_0[6] = {};
    double v6_1[6] = {};
    double m33_0[3][3] = {};
    double m33_1[3][3] = {};
    double m66_0[6][6] = {};
    double m66_1[6][6] = {};
    double m66_2[6][6] = {};

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Copy(m66_0, m66_1);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66Copy";

    m66Set(0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           0,
           m66_0);
    m66SetZero(m66_1);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66SetZero";

    m66Set(1,
           0,
           0,
           0,
           0,
           0,
           0,
           1,
           0,
           0,
           0,
           0,
           0,
           0,
           1,
           0,
           0,
           0,
           0,
           0,
           0,
           1,
           0,
           0,
           0,
           0,
           0,
           0,
           1,
           0,
           0,
           0,
           0,
           0,
           0,
           1,
           m66_0);
    m66SetIdentity(m66_1);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66SetIdentity";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(1,
           7,
           13,
           19,
           25,
           31,
           2,
           8,
           14,
           20,
           26,
           32,
           3,
           9,
           15,
           21,
           27,
           33,
           4,
           10,
           16,
           22,
           28,
           34,
           5,
           11,
           17,
           23,
           29,
           35,
           6,
           12,
           18,
           24,
           30,
           36,
           m66_1);
    m66Transpose(m66_0, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66Transpose";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m33Set(4, 5, 6, 10, 11, 12, 16, 17, 18, m33_1);
    m66Get33Matrix(0, 1, m66_0, m33_0);
    EXPECT_TRUE(m33IsEqual(m33_0, m33_1, kAccuracy)) << "m66Get33Matrix";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m33Set(54, 55, 56, 57, 58, 59, 60, 61, 62, m33_0);
    m66Set(1,
           2,
           3,
           54,
           55,
           56,
           7,
           8,
           9,
           57,
           58,
           59,
           13,
           14,
           15,
           60,
           61,
           62,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_1);
    m66Set33Matrix(0, 1, m33_0, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66Set33Matrix";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(2,
           4,
           6,
           8,
           10,
           12,
           14,
           16,
           18,
           20,
           22,
           24,
           26,
           28,
           30,
           32,
           34,
           36,
           38,
           40,
           42,
           44,
           46,
           48,
           50,
           52,
           54,
           56,
           58,
           60,
           62,
           64,
           66,
           68,
           70,
           72,
           m66_1);
    m66Scale(2.0, m66_0, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_1, kAccuracy)) << "m66Scale";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           37,
           38,
           39,
           40,
           41,
           42,
           43,
           44,
           45,
           m66_1);
    m66Set(11,
           13,
           15,
           17,
           19,
           21,
           23,
           25,
           27,
           29,
           31,
           33,
           35,
           37,
           39,
           41,
           43,
           45,
           47,
           49,
           51,
           53,
           55,
           57,
           59,
           61,
           63,
           65,
           67,
           69,
           71,
           73,
           75,
           77,
           79,
           81,
           m66_2);
    m66Add(m66_0, m66_1, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_2, kAccuracy)) << "m66Add";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           37,
           38,
           39,
           40,
           41,
           42,
           43,
           44,
           45,
           m66_1);
    m66Set(-9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           -9,
           m66_2);
    m66Subtract(m66_0, m66_1, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_2, kAccuracy)) << "m66Subtract";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           37,
           38,
           39,
           40,
           41,
           42,
           43,
           44,
           45,
           m66_1);
    m66Set(630,
           651,
           672,
           693,
           714,
           735,
           1530,
           1587,
           1644,
           1701,
           1758,
           1815,
           2430,
           2523,
           2616,
           2709,
           2802,
           2895,
           3330,
           3459,
           3588,
           3717,
           3846,
           3975,
           4230,
           4395,
           4560,
           4725,
           4890,
           5055,
           5130,
           5331,
           5532,
           5733,
           5934,
           6135,
           m66_2);
    m66MultM66(m66_0, m66_1, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_2, kAccuracy)) << "m66MultM66";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           37,
           38,
           39,
           40,
           41,
           42,
           43,
           44,
           45,
           m66_1);
    m66Set(3030,
           3126,
           3222,
           3318,
           3414,
           3510,
           3180,
           3282,
           3384,
           3486,
           3588,
           3690,
           3330,
           3438,
           3546,
           3654,
           3762,
           3870,
           3480,
           3594,
           3708,
           3822,
           3936,
           4050,
           3630,
           3750,
           3870,
           3990,
           4110,
           4230,
           3780,
           3906,
           4032,
           4158,
           4284,
           4410,
           m66_2);
    m66tMultM66(m66_0, m66_1, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_2, kAccuracy)) << "m66tMultM66";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    m66Set(10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           37,
           38,
           39,
           40,
           41,
           42,
           43,
           44,
           45,
           m66_1);
    m66Set(280,
           406,
           532,
           658,
           784,
           910,
           730,
           1072,
           1414,
           1756,
           2098,
           2440,
           1180,
           1738,
           2296,
           2854,
           3412,
           3970,
           1630,
           2404,
           3178,
           3952,
           4726,
           5500,
           2080,
           3070,
           4060,
           5050,
           6040,
           7030,
           2530,
           3736,
           4942,
           6148,
           7354,
           8560,
           m66_2);
    m66MultM66t(m66_0, m66_1, m66_0);
    EXPECT_TRUE(m66IsEqual(m66_0, m66_2, kAccuracy)) << "m66MultM66t";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    v6Set(10, 11, 12, 13, 14, 15, v6_0);
    v6Set(280, 730, 1180, 1630, 2080, 2530, v6_1);
    m66MultV6(m66_0, v6_0, v6_0);
    EXPECT_TRUE(v6IsEqual(v6_0, v6_1, kAccuracy)) << "m66MultV6";

    m66Set(1,
           2,
           3,
           4,
           5,
           6,
           7,
           8,
           9,
           10,
           11,
           12,
           13,
           14,
           15,
           16,
           17,
           18,
           19,
           20,
           21,
           22,
           23,
           24,
           25,
           26,
           27,
           28,
           29,
           30,
           31,
           32,
           33,
           34,
           35,
           36,
           m66_0);
    v6Set(10, 11, 12, 13, 14, 15, v6_0);
    v6Set(1305, 1380, 1455, 1530, 1605, 1680, v6_1);
    m66tMultV6(m66_0, v6_0, v6_0);
    EXPECT_TRUE(v6IsEqual(v6_0, v6_1, kAccuracy)) << "m66tMultV6";
}

TEST(LinearAlgebraPolynomials, CubicRoots) {
    double coefficients[3] = {};
    double expected[3] = {};
    double result[3] = {};

    v3Set(-27, -72, -6, coefficients);
    v3Set(12.1228937846324, -5.73450994222507, -0.38838384240732, expected);
    cubicRoots(coefficients, result);
    EXPECT_TRUE(v3IsEqual(expected, result, kAccuracy)) << "cubicRoots";
}
