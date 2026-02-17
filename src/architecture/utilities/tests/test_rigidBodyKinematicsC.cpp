// SPDX-License-Identifier: ISC
// Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#include <gtest/gtest.h>

extern "C" {
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "unitTestComparators.h"
}

#include <numbers>

constexpr double kAccuracy = 1e-10;

TEST(RigidBodyKinematicsC, AllTests) {
    double C[3][3];
    double C2[3][3];
    double v3_1[4];
    double v3_2[4];
    double v3[4];
    double om[3];
    const double accuracy = kAccuracy;

    v4Set(0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665, v3_1);
    v4Set(-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381, v3_2);
    addEP(v3_1, v3_2, v3);
    v4Set(-0.46986547690254, -0.34044145332460, 0.71745926113861, 0.38545850500388, v3_1);
    EXPECT_TRUE(vIsEqual(v3, 4, v3_1, accuracy)) << "addEP failed";
    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler121(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler121 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler123(v3_1, v3_2, v3);
    v3Set(2.65556257351773, -0.34257634487528, -2.38843896474589, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler123 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler131(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler123 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler132(v3_1, v3_2, v3);
    v3Set(2.93168877067466, -0.89056295435594, -2.11231276758895, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler132 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler212(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler212 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler213(v3_1, v3_2, v3);
    v3Set(2.93168877067466, -0.89056295435594, -2.11231276758895, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler213 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler231(v3_1, v3_2, v3);
    v3Set(2.65556257351773, -0.34257634487528, -2.38843896474589, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler231 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler232(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler232 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler312(v3_1, v3_2, v3);
    v3Set(2.65556257351773, -0.34257634487528, -2.38843896474589, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler312 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler313(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler313 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler321(v3_1, v3_2, v3);
    v3Set(2.93168877067466, -0.89056295435594, -2.11231276758895, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler321 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    addEuler323(v3_1, v3_2, v3);
    v3Set(-2.96705972839036, 2.44346095279206, 1.41371669411541, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addEuler323 failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    addGibbs(v3_1, v3_2, v3);
    v3Set(0.61290322580645, 0.17741935483871, 0.82258064516129, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addGibbs failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    addMRP(v3_1, v3_2, v3);
    v3Set(0.58667769962764, -0.34919321472900, 0.43690525444766, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addMRP failed";

    v3Set(0.0, 0.0, 1.0, v3_1);
    v3Set(0.0, 0.0, 1.0, v3_2);
    addMRP(v3_1, v3_2, v3);
    v3Set(0.0, 0.0, 0.0, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addMRP 360 addition test failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    addPRV(v3_1, v3_2, v3);
    v3Set(1.00227389370983, 0.41720669426711, 0.86837149207759, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "addPRV failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler121(v3_1, C);
    v3Set(0.76604444311898, 0.0, 1.0, C2[0]);
    v3Set(-0.16636567534280, 0.96592582628907, 0., C2[1]);
    v3Set(-0.62088515301485, -0.25881904510252, 0., C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler121 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler123(v3_1, C);
    v3Set(0.73994211169385, 0.25881904510252, 0, C2[0]);
    v3Set(-0.19826689127415, 0.96592582628907, 0, C2[1]);
    v3Set(-0.64278760968654, 0, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler123 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler131(v3_1, C);
    v3Set(0.76604444311898, 0, 1.00000000000000, C2[0]);
    v3Set(0.62088515301485, 0.25881904510252, 0, C2[1]);
    v3Set(-0.16636567534280, 0.96592582628907, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler131 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler132(v3_1, C);
    v3Set(0.73994211169385, -0.25881904510252, 0, C2[0]);
    v3Set(0.64278760968654, 0, 1.00000000000000, C2[1]);
    v3Set(0.19826689127415, 0.96592582628907, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler132 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler212(v3_1, C);
    v3Set(-0.16636567534280, 0.96592582628907, 0, C2[0]);
    v3Set(0.76604444311898, 0, 1.00000000000000, C2[1]);
    v3Set(0.62088515301485, 0.25881904510252, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler212 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler213(v3_1, C);
    v3Set(0.19826689127415, 0.96592582628907, 0, C2[0]);
    v3Set(0.73994211169385, -0.25881904510252, 0, C2[1]);
    v3Set(0.64278760968654, 0, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler213 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler231(v3_1, C);
    v3Set(-0.64278760968654, 0, 1.00000000000000, C2[0]);
    v3Set(0.73994211169385, 0.25881904510252, 0, C2[1]);
    v3Set(-0.19826689127415, 0.96592582628907, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler231 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler232(v3_1, C);
    v3Set(-0.62088515301485, -0.25881904510252, 0, C2[0]);
    v3Set(0.76604444311898, 0, 1.00000000000000, C2[1]);
    v3Set(-0.16636567534280, 0.96592582628907, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler232 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler312(v3_1, C);
    v3Set(-0.19826689127415, 0.96592582628907, 0, C2[0]);
    v3Set(-0.64278760968654, 0, 1.00000000000000, C2[1]);
    v3Set(0.73994211169385, 0.25881904510252, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler312 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler313(v3_1, C);
    v3Set(-0.16636567534280, 0.96592582628907, 0, C2[0]);
    v3Set(-0.62088515301485, -0.25881904510252, 0, C2[1]);
    v3Set(0.76604444311898, 0, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler313 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler321(v3_1, C);
    v3Set(0.64278760968654, 0, 1.00000000000000, C2[0]);
    v3Set(0.19826689127415, 0.96592582628907, 0, C2[1]);
    v3Set(0.73994211169385, -0.25881904510252, 0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler321 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BinvEuler323(v3_1, C);
    v3Set(0.62088515301485, 0.25881904510252, 0, C2[0]);
    v3Set(-0.16636567534280, 0.96592582628907, 0, C2[1]);
    v3Set(0.76604444311898, 0, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvEuler323 failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BinvGibbs(v3_1, C);
    v3Set(0.64, -0.32, -0.32, C2[0]);
    v3Set(0.32, 0.64, 0.16, C2[1]);
    v3Set(0.32, -0.16, 0.64, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvGibbs failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BinvMRP(v3_1, C);
    v3Set(0.2304, -0.3072, -0.512, C2[0]);
    v3Set(0.512, 0.384, 0, C2[1]);
    v3Set(0.3072, -0.4096, 0.3840, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvMRP failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BinvPRV(v3_1, C);
    v3Set(0.91897927113877, -0.21824360100796, -0.25875396543858, C2[0]);
    v3Set(0.25875396543858, 0.94936204446173, 0.07873902718102, C2[1]);
    v3Set(0.21824360100796, -0.15975975604225, 0.94936204446173, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BinvPRV failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler121(v3_1, C);
    v3Set(0, -0.40265095531125, -1.50271382293774, C2[0]);
    v3Set(0, 0.96592582628907, -0.25881904510252, C2[1]);
    v3Set(1.00000000000000, 0.30844852683273, 1.15114557365953, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler121 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler123(v3_1, C);
    v3Set(1.26092661459205, -0.33786426809485, 0, C2[0]);
    v3Set(0.25881904510252, 0.96592582628907, 0, C2[1]);
    v3Set(0.81050800458377, -0.21717496528718, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler123 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler131(v3_1, C);
    v3Set(0, 1.50271382293774, -0.40265095531125, C2[0]);
    v3Set(0, 0.25881904510252, 0.96592582628907, C2[1]);
    v3Set(1.00000000000000, -1.15114557365953, 0.30844852683273, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler131 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler132(v3_1, C);
    v3Set(1.26092661459205, 0, 0.33786426809485, C2[0]);
    v3Set(-0.25881904510252, 0, 0.96592582628907, C2[1]);
    v3Set(-0.81050800458377, 1.00000000000000, -0.21717496528718, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler132 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler212(v3_1, C);
    v3Set(-0.40265095531125, 0, 1.50271382293774, C2[0]);
    v3Set(0.96592582628907, 0, 0.25881904510252, C2[1]);
    v3Set(0.30844852683273, 1.00000000000000, -1.15114557365953, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler212 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler213(v3_1, C);
    v3Set(0.33786426809485, 1.26092661459205, 0, C2[0]);
    v3Set(0.96592582628907, -0.25881904510252, 0, C2[1]);
    v3Set(-0.21717496528718, -0.81050800458377, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler213 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler231(v3_1, C);
    v3Set(0, 1.26092661459205, -0.33786426809485, C2[0]);
    v3Set(0, 0.25881904510252, 0.96592582628907, C2[1]);
    v3Set(1.00000000000000, 0.81050800458377, -0.21717496528718, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler231 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler232(v3_1, C);
    v3Set(-1.50271382293774, 0, -0.40265095531125, C2[0]);
    v3Set(-0.25881904510252, 0, 0.96592582628907, C2[1]);
    v3Set(1.15114557365953, 1.00000000000000, 0.30844852683273, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler232 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler312(v3_1, C);
    v3Set(-0.33786426809485, 0, 1.26092661459205, C2[0]);
    v3Set(0.96592582628907, 0, 0.25881904510252, C2[1]);
    v3Set(-0.21717496528718, 1.00000000000000, 0.81050800458377, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler312 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler313(v3_1, C);
    v3Set(-0.40265095531125, -1.50271382293774, 0, C2[0]);
    v3Set(0.96592582628907, -0.25881904510252, 0, C2[1]);
    v3Set(0.30844852683273, 1.15114557365953, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler313 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler321(v3_1, C);
    v3Set(0, 0.33786426809485, 1.26092661459205, C2[0]);
    v3Set(0, 0.96592582628907, -0.25881904510252, C2[1]);
    v3Set(1.00000000000000, -0.21717496528718, -0.81050800458377, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler321 failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    BmatEuler323(v3_1, C);
    v3Set(1.50271382293774, -0.40265095531125, 0, C2[0]);
    v3Set(0.25881904510252, 0.96592582628907, 0, C2[1]);
    v3Set(-1.15114557365953, 0.30844852683273, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatEuler323 failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BmatGibbs(v3_1, C);
    v3Set(1.06250000000000, 0.62500000000000, 0.37500000000000, C2[0]);
    v3Set(-0.37500000000000, 1.25000000000000, -0.50000000000000, C2[1]);
    v3Set(-0.62500000000000, 0, 1.25000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatGibbs failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BmatMRP(v3_1, C);
    v3Set(0.56250000000000, 1.25000000000000, 0.75000000000000, C2[0]);
    v3Set(-0.75000000000000, 0.93750000000000, -1.00000000000000, C2[1]);
    v3Set(-1.25000000000000, 0, 0.93750000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatMRP failed";

    v3Set(0.2, 0.1, -0.5, v3_1);
    v3Set(0.015, 0.045, -0.005, v3_2);
    v3Scale(1 / D2R, v3_2, v3_2);
    BdotmatMRP(v3_1, v3_2, C);
    v3Set(-0.4583662361046585, 1.7761691649055522, 4.1825919044550091, C2[0]);
    v3Set(0.6302535746439056, -0.1145915590261646, -4.3544792429942563, C2[1]);
    v3Set(-6.1306484078998089, -0.9167324722093173, -0.5729577951308232, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BdotmatMRP failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    BmatPRV(v3_1, C);
    v3Set(0.95793740211924, 0.26051564947019, 0.23948435052981, C2[0]);
    v3Set(-0.23948435052981, 0.97371087632453, -0.14603129894038, C2[1]);
    v3Set(-0.26051564947019, 0.10396870105962, 0.97371087632453, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "BmatPRV failed";

    v3Set(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C[0]);
    v3Set(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C[1]);
    v3Set(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C[2]);
    C2EP(C, v3_1);
    v4Set(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2EP failed";

    v3Set(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C[0]);
    v3Set(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C[1]);
    v3Set(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C[2]);
    C2Euler121(C, v3_1);
    v3Set(-3.081087141428621, 2.102046098550739, -1.127921895439695, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler121 failed";
    C2Euler123(C, v3_1);
    v3Set(1.395488250243478, 0.3784438476398376, 2.147410157986089, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler123 failed";
    C2Euler131(C, v3_1);
    v3Set(1.631301838956069, 2.102046098550739, 0.4428744313552013, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler131 failed";
    C2Euler132(C, v3_1);
    v3Set(-2.262757475208626, 0.8930615653924096, 2.511467464302149, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler132 failed";
    C2Euler212(C, v3_1);
    v3Set(-2.125637903992466, 1.982395614047245, -0.05691616561213509, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler212 failed";
    C2Euler213(C, v3_1);
    v3Set(1.157420789791818, 1.155503238813826, -3.012011225795042, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler213 failed";
    C2Euler231(C, v3_1);
    v3Set(-2.102846464319881, -0.05215813778076988, 1.982990154077466, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler231 failed";
    C2Euler232(C, v3_1);
    v3Set(-0.5548415771975691, 1.982395614047245, -1.627712492407032, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler232 failed";
    C2Euler312(C, v3_1);
    v3Set(2.045248068737305, -0.5038614866151004, -1.384653359078797, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler312 failed";
    C2Euler313(C, v3_1);
    v3Set(0.3837766626244829, 1.408008028147626, 2.082059614484753, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler313 failed";
    C2Euler321(C, v3_1);
    v3Set(-3.039045355374235, -1.036440549977791, -1.246934586231547, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler321 failed";
    C2Euler323(C, v3_1);
    v3Set(-1.187019664170414, 1.408008028147626, -2.630329365899936, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Euler323 failed";

    v3Set(0.25, 0.5, -0.5, v3_1);
    C2Gibbs(C, v3_1);
    v3Set(1.692307692307693, -1.923076923076923, 2.846153846153846, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2Gibbs failed";
    C2MRP(C, v3_1);
    v3Set(0.3413551595269481, -0.3879035903715318, 0.5740973137498672, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2MRP failed";
    C2PRV(C, v3_1);
    v3Set(1.162634795241009, -1.321175903682964, 1.955340337450788, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2PRV failed";
    m33SetIdentity(C);
    C2PRV(C, v3_1);
    v3Set(0.0, 0.0, 0.0, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2PRV failed";
    m33SetIdentity(C);
    C[0][0] = -1.0;
    C[1][1] = -1.0;
    C2PRV(C, v3_1);
    v3Set(0.0, 0.0, std::numbers::pi, v3_2);
    EXPECT_TRUE(v3IsEqual(v3_1, v3_2, accuracy)) << "C2PRV failed";

    v4Set(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733, v3_1);
    v3Set(0.2, 0.1, -0.5, om);
    dEP(v3_1, om, v3);
    v4Set(0.1613247949317332, 0.1107893170013107, 0.1914517144671774, 0.006802852798326098, v3_1);
    EXPECT_TRUE(vIsEqual(v3_1, 4, v3, accuracy)) << "dEP failed";

    v3Set(30 * D2R, -40 * D2R, 15 * D2R, v3_1);
    v3Set(0.2, 0.1, -0.5, om);
    dEuler121(v3_1, om, v3);
    v3Set(0.7110918159377425, 0.2260021051801672, -0.3447279341464908, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler121 failed";
    dEuler123(v3_1, om, v3);
    v3Set(0.2183988961089258, 0.148356391649411, -0.3596158956119647, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler123 failed";
    dEuler131(v3_1, om, v3);
    v3Set(0.3515968599493992, -0.4570810086342821, -0.06933882078231876, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler131 failed";
    dEuler132(v3_1, om, v3);
    v3Set(0.08325318887098565, -0.5347267221650382, 0.04648588172683711, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler132 failed";
    dEuler212(v3_1, om, v3);
    v3Set(-0.8318871025311179, 0.06377564270655334, 0.7372624921963103, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler212 failed";
    dEuler213(v3_1, om, v3);
    v3Set(0.1936655150781755, 0.1673032607475616, -0.6244857935158128, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler213 failed";
    dEuler231(v3_1, om, v3);
    v3Set(0.2950247955066306, -0.4570810086342821, 0.3896382831019671, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler231 failed";
    dEuler232(v3_1, om, v3);
    v3Set(-0.09921728693192147, -0.5347267221650384, 0.1760048513155397, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler232 failed";
    dEuler312(v3_1, om, v3);
    v3Set(-0.6980361609149971, 0.06377564270655331, -0.3486889953493196, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler312 failed";
    dEuler313(v3_1, om, v3);
    v3Set(-0.2308015733560238, 0.1673032607475616, -0.3231957372675008, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler312 failed";
    dEuler321(v3_1, om, v3);
    v3Set(-0.596676880486542, 0.2260021051801672, 0.5835365057631652, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler312 failed";
    dEuler323(v3_1, om, v3);
    v3Set(0.260277669056422, 0.148356391649411, -0.6993842620486324, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dEuler312 failed";

    dGibbs(v3_1, om, v3);
    v3Set(0.236312018677072, 0.2405875488560276, -0.1665723597065136, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dGibbs failed";
    dMRP(v3_1, om, v3);
    v3Set(0.144807895231133, 0.1948354871330581, 0.062187948908334, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dMRP failed";
    dPRV(v3_1, om, v3);
    v3Set(0.34316538031149, 0.255728121815202, -0.3710557691157747, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "dPRV failed";

    double w[3];
    v3Set(0.0124791041517595, 0.0042760566673861, -0.0043633231299858, v3_1);
    dMRP2Omega(om, v3_1, v3);
    v3Set(0.0174532925199433, 0.0349065850398866, -0.0174532925199433, w);
    EXPECT_TRUE(v3IsEqual(v3, w, accuracy)) << "dMRP2Omega failed";

    double dw[3];
    v3Set(0.0022991473184427, 0.0035194052312667, -0.0070466757773158, dw);
    ddMRP(om, v3_1, w, dw, v3);
    v3Set(0.0015, 0.0010, -0.0020, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "ddMRP failed";

    ddMRP2dOmega(om, v3_1, v3_2, v3);
    EXPECT_TRUE(v3IsEqual(v3, dw, accuracy)) << "ddMRP2dOmega failed";

    v4Set(0.9110886174894189, 0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    elem2PRV(v3_1, v3);
    v3Set(0.5235987755982988, -0.6981317007977318, 0.2617993877991494, v3_2);
    EXPECT_TRUE(v3IsEqual(v3, v3_2, accuracy)) << "elem2PRV failed";

    v4Set(0.2526773896521122, 0.4276078901804977, -0.4859180570232927, 0.7191587243944733, v3_1);
    EP2C(v3_1, C);
    v3Set(-0.506611258027956, -0.05213449187759728, 0.860596902153381, C2[0]);
    v3Set(-0.7789950887797505, -0.4000755572346052, -0.4828107291273137, C2[1]);
    v3Set(0.3694748772194938, -0.9149981110691346, 0.1620702682281828, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "EP2C failed";
    EP2Euler121(v3_1, v3_2);
    v3Set(3.202098165750965, 2.102046098550739, -1.127921895439695, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler121 failed";
    EP2Euler123(v3_1, v3_2);
    v3Set(1.395488250243478, 0.3784438476398376, 2.147410157986089, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler123 failed";
    EP2Euler131(v3_1, v3_2);
    v3Set(1.631301838956069, 2.102046098550739, 0.4428744313552013, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler131 failed";
    EP2Euler132(v3_1, v3_2);
    v3Set(-2.262757475208626, 0.8930615653924096, 2.511467464302149, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler132 failed";
    EP2Euler212(v3_1, v3_2);
    v3Set(-2.125637903992466, 1.982395614047245, -0.05691616561213508, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler212 failed";
    EP2Euler213(v3_1, v3_2);
    v3Set(1.157420789791818, 1.155503238813826, -3.012011225795042, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler213 failed";
    EP2Euler231(v3_1, v3_2);
    v3Set(-2.102846464319881, -0.05215813778076988, 1.982990154077466, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler231 failed";
    EP2Euler232(v3_1, v3_2);
    v3Set(-0.5548415771975691, 1.982395614047245, -1.627712492407032, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler232 failed";
    EP2Euler312(v3_1, v3_2);
    v3Set(2.045248068737305, -0.5038614866151004, -1.384653359078797, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler312 failed";
    EP2Euler313(v3_1, v3_2);
    v3Set(0.3837766626244828, 1.408008028147627, 2.082059614484753, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler313 failed";
    EP2Euler321(v3_1, v3_2);
    v3Set(-3.039045355374235, -1.036440549977791, -1.246934586231547, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler321 failed";
    EP2Euler323(v3_1, v3_2);
    v3Set(-1.187019664170414, 1.408008028147627, 3.65285594127965, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Euler323 failed";
    EP2Gibbs(v3_1, v3_2);
    v3Set(1.692307692307693, -1.923076923076923, 2.846153846153846, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2Gibbs failed";
    EP2MRP(v3_1, v3_2);
    v3Set(0.3413551595269481, -0.3879035903715319, 0.5740973137498672, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2MRP failed";
    EP2PRV(v3_1, v3_2);
    v3Set(1.162634795241009, -1.321175903682965, 1.955340337450788, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2PRV failed";

    v4Set(1.0, 0.0, 0.0, 0.0, v3_1);
    EP2PRV(v3_1, v3_2);
    v3Set(0.0, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2PRV failed";
    v4Set(-1.0, 0.0, 0.0, 0.0, v3_1);
    EP2PRV(v3_1, v3_2);
    v3Set(0.0, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2PRV failed";
    v4Set(0.0, 1.0, 0.0, 0.0, v3_1);
    EP2PRV(v3_1, v3_2);
    v3Set(std::numbers::pi, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "EP2PRV failed";

    Euler1(1.3, C);
    v3Set(1, 0, 0, C2[0]);
    v3Set(0, 0.2674988286245874, 0.963558185417193, C2[1]);
    v3Set(0, -0.963558185417193, 0.2674988286245874, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler1 failed";
    Euler2(1.3, C);
    v3Set(0.2674988286245874, 0, -0.963558185417193, C2[0]);
    v3Set(0, 1, 0, C2[1]);
    v3Set(0.963558185417193, 0, 0.2674988286245874, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler2 failed";
    Euler3(1.3, C);
    v3Set(0.2674988286245874, 0.963558185417193, 0, C2[0]);
    v3Set(-0.963558185417193, 0.2674988286245874, 0, C2[1]);
    v3Set(0, 0, 1, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3 failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler1212C(v3_1, C);
    v3Set(0.7205084754311385, -0.3769430728235922, 0.5820493593177511, C2[0]);
    v3Set(-0.1965294640304305, 0.6939446195986547, 0.692688266609151, C2[1]);
    v3Set(-0.6650140649638986, -0.6134776155495705, 0.4259125598286639, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler1212C failed";
    Euler1212EP(v3_1, v3_2);
    v4Set(0.8426692196316502, 0.3875084824890354, -0.3699741829975614, -0.05352444488005169, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler1212EP failed";
    Euler1212Gibbs(v3_1, v3_2);
    v3Set(0.4598583565902931, -0.4390503110571495, -0.06351774057138154, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1212Gibbs failed";
    Euler1212MRP(v3_1, v3_2);
    v3Set(0.2102973655610845, -0.2007816590497557, -0.02904723447366817, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1212MRP failed";
    Euler1212PRV(v3_1, v3_2);
    v3Set(0.8184049632304388, -0.7813731087574279, -0.1130418386266624, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1212PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler1232C(v3_1, C);
    v3Set(0.6909668228739537, -0.1236057418710468, 0.7122404581768593, C2[0]);
    v3Set(-0.2041991989591971, 0.9117724894309838, 0.3563335721781613, C2[1]);
    v3Set(-0.6934461311680212, -0.391653607277317, 0.6047643467291773, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler1232C failed";
    Euler1232EP(v3_1, v3_2);
    v4Set(0.8954752451958283, 0.2088240806958052, -0.3924414987701519, 0.02250019124496444, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler1232EP failed";
    Euler1232Gibbs(v3_1, v3_2);
    v3Set(0.2331991663824702, -0.4382494109977661, 0.02512653628972619, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1232Gibbs failed";
    Euler1232MRP(v3_1, v3_2);
    v3Set(0.1101697746911123, -0.2070412155288303, 0.01187047485953311, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1232MRP failed";
    Euler1232PRV(v3_1, v3_2);
    v3Set(0.4328366663508259, -0.8134266388215754, 0.04663690000825693, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1232PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler1312C(v3_1, C);
    v3Set(0.7205084754311385, -0.5820493593177511, -0.3769430728235922, C2[0]);
    v3Set(0.6650140649638986, 0.4259125598286639, 0.6134776155495705, C2[1]);
    v3Set(-0.1965294640304305, -0.692688266609151, 0.6939446195986547, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler1312C failed";
    Euler1312EP(v3_1, v3_2);
    v4Set(0.8426692196316502, 0.3875084824890354, 0.05352444488005169, -0.3699741829975614, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler1312EP failed";
    Euler1312Gibbs(v3_1, v3_2);
    v3Set(0.4598583565902931, 0.06351774057138154, -0.4390503110571495, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1312Gibbs failed";
    Euler1312MRP(v3_1, v3_2);
    v3Set(0.2102973655610845, 0.02904723447366817, -0.2007816590497557, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1312MRP failed";
    Euler1312PRV(v3_1, v3_2);
    v3Set(0.8184049632304388, 0.1130418386266624, -0.7813731087574279, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1312PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler1322C(v3_1, C);
    v3Set(0.6909668228739537, -0.404128912281835, -0.5993702294453531, C2[0]);
    v3Set(0.6934461311680212, 0.6047643467291773, 0.391653607277317, C2[1]);
    v3Set(0.2041991989591971, -0.6862506154337003, 0.6981137299618809, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler1322C failed";
    Euler1322EP(v3_1, v3_2);
    v4Set(0.8651365354042408, 0.3114838463640192, 0.2322088466732818, -0.3171681574333834, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler1322EP failed";
    Euler1322Gibbs(v3_1, v3_2);
    v3Set(0.3600401018996109, 0.2684071671586273, -0.3666105226791566, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1322Gibbs failed";
    Euler1322MRP(v3_1, v3_2);
    v3Set(0.1670032410235906, 0.1244996504360223, -0.1700509058789317, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1322MRP failed";
    Euler1322PRV(v3_1, v3_2);
    v3Set(0.6525765328552258, 0.4864908592507521, -0.6644854907437873, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler1322PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler2122C(v3_1, C);
    v3Set(0.6939446195986547, -0.1965294640304305, -0.692688266609151, C2[0]);
    v3Set(-0.3769430728235922, 0.7205084754311385, -0.5820493593177511, C2[1]);
    v3Set(0.6134776155495705, 0.6650140649638986, 0.4259125598286639, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler2122C failed";
    Euler2122EP(v3_1, v3_2);
    v4Set(0.8426692196316502, -0.3699741829975614, 0.3875084824890354, 0.05352444488005169, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler2122EP failed";
    Euler2122Gibbs(v3_1, v3_2);
    v3Set(-0.4390503110571495, 0.4598583565902931, 0.06351774057138154, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2122Gibbs failed";
    Euler2122MRP(v3_1, v3_2);
    v3Set(-0.2007816590497557, 0.2102973655610845, 0.02904723447366817, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2122MRP failed";
    Euler2122PRV(v3_1, v3_2);
    v3Set(-0.7813731087574279, 0.8184049632304388, 0.1130418386266624, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2122PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler2132C(v3_1, C);
    v3Set(0.6981137299618809, 0.2041991989591971, -0.6862506154337003, C2[0]);
    v3Set(-0.5993702294453531, 0.6909668228739537, -0.404128912281835, C2[1]);
    v3Set(0.391653607277317, 0.6934461311680212, 0.6047643467291773, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler2132C failed";
    Euler2132EP(v3_1, v3_2);
    v4Set(0.8651365354042408, -0.3171681574333834, 0.3114838463640192, 0.2322088466732818, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler2132EP failed";
    Euler2132Gibbs(v3_1, v3_2);
    v3Set(-0.3666105226791566, 0.3600401018996109, 0.2684071671586273, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2132Gibbs failed";
    Euler2132MRP(v3_1, v3_2);
    v3Set(-0.1700509058789317, 0.1670032410235906, 0.1244996504360223, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2132MRP failed";
    Euler2132PRV(v3_1, v3_2);
    v3Set(-0.6644854907437873, 0.6525765328552258, 0.4864908592507521, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2132PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler2312C(v3_1, C);
    v3Set(0.6047643467291773, -0.6934461311680212, -0.391653607277317, C2[0]);
    v3Set(0.7122404581768593, 0.6909668228739537, -0.1236057418710468, C2[1]);
    v3Set(0.3563335721781613, -0.2041991989591971, 0.9117724894309838, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler2312C failed";
    Euler2312EP(v3_1, v3_2);
    v4Set(0.8954752451958283, 0.02250019124496444, 0.2088240806958052, -0.3924414987701519, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler2312EP failed";
    Euler2312Gibbs(v3_1, v3_2);
    v3Set(0.02512653628972619, 0.2331991663824702, -0.4382494109977661, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2312Gibbs failed";
    Euler2312MRP(v3_1, v3_2);
    v3Set(0.01187047485953311, 0.1101697746911123, -0.2070412155288303, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2312MRP failed";
    Euler2312PRV(v3_1, v3_2);
    v3Set(0.04663690000825693, 0.4328366663508259, -0.8134266388215754, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2312PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler2322C(v3_1, C);
    v3Set(0.4259125598286639, -0.6650140649638986, -0.6134776155495705, C2[0]);
    v3Set(0.5820493593177511, 0.7205084754311385, -0.3769430728235922, C2[1]);
    v3Set(0.692688266609151, -0.1965294640304305, 0.6939446195986547, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler2322C failed";
    Euler2322EP(v3_1, v3_2);
    v4Set(0.8426692196316502, -0.05352444488005169, 0.3875084824890354, -0.3699741829975614, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler2322EP failed";
    Euler2322Gibbs(v3_1, v3_2);
    v3Set(-0.06351774057138154, 0.4598583565902931, -0.4390503110571495, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2322Gibbs failed";
    Euler2322MRP(v3_1, v3_2);
    v3Set(-0.02904723447366817, 0.2102973655610845, -0.2007816590497557, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2322MRP failed";
    Euler2322PRV(v3_1, v3_2);
    v3Set(-0.1130418386266624, 0.8184049632304388, -0.7813731087574279, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler2322PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler3122C(v3_1, C);
    v3Set(0.9117724894309838, 0.3563335721781613, -0.2041991989591971, C2[0]);
    v3Set(-0.391653607277317, 0.6047643467291773, -0.6934461311680212, C2[1]);
    v3Set(-0.1236057418710468, 0.7122404581768593, 0.6909668228739537, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3122C failed";
    Euler3122EP(v3_1, v3_2);
    v4Set(0.8954752451958283, -0.3924414987701519, 0.02250019124496444, 0.2088240806958052, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler3122EP failed";
    Euler3122Gibbs(v3_1, v3_2);
    v3Set(-0.4382494109977661, 0.02512653628972619, 0.2331991663824702, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3122Gibbs failed";
    Euler3122MRP(v3_1, v3_2);
    v3Set(-0.2070412155288303, 0.01187047485953311, 0.1101697746911123, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3122MRP failed";
    Euler3122PRV(v3_1, v3_2);
    v3Set(-0.8134266388215754, 0.04663690000825693, 0.4328366663508259, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3122PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler3132C(v3_1, C);
    v3Set(0.6939446195986547, 0.692688266609151, -0.1965294640304305, C2[0]);
    v3Set(-0.6134776155495705, 0.4259125598286639, -0.6650140649638986, C2[1]);
    v3Set(-0.3769430728235922, 0.5820493593177511, 0.7205084754311385, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3132C failed";
    Euler3132EP(v3_1, v3_2);
    v4Set(0.8426692196316502, -0.3699741829975614, -0.05352444488005169, 0.3875084824890354, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler3132EP failed";
    Euler3132Gibbs(v3_1, v3_2);
    v3Set(-0.4390503110571495, -0.06351774057138154, 0.4598583565902931, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3132Gibbs failed";
    Euler3132MRP(v3_1, v3_2);
    v3Set(-0.2007816590497557, -0.02904723447366817, 0.2102973655610845, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3132MRP failed";
    Euler3132PRV(v3_1, v3_2);
    v3Set(-0.7813731087574279, -0.1130418386266624, 0.8184049632304388, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3132PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler3212C(v3_1, C);
    v3Set(0.6047643467291773, 0.391653607277317, 0.6934461311680212, C2[0]);
    v3Set(-0.6862506154337003, 0.6981137299618809, 0.2041991989591971, C2[1]);
    v3Set(-0.404128912281835, -0.5993702294453531, 0.6909668228739537, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3212C failed";
    Euler3212EP(v3_1, v3_2);
    v4Set(0.8651365354042408, 0.2322088466732818, -0.3171681574333834, 0.3114838463640192, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler3212EP failed";
    Euler3212Gibbs(v3_1, v3_2);
    v3Set(0.2684071671586273, -0.3666105226791566, 0.3600401018996109, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3212Gibbs failed";
    Euler3212MRP(v3_1, v3_2);
    v3Set(0.1244996504360223, -0.1700509058789317, 0.1670032410235906, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3212MRP failed";
    Euler3212PRV(v3_1, v3_2);
    v3Set(0.4864908592507521, -0.6644854907437873, 0.6525765328552258, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3212PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Euler3232C(v3_1, C);
    v3Set(0.4259125598286639, 0.6134776155495705, 0.6650140649638986, C2[0]);
    v3Set(-0.692688266609151, 0.6939446195986547, -0.1965294640304305, C2[1]);
    v3Set(-0.5820493593177511, -0.3769430728235922, 0.7205084754311385, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3232C failed";
    Euler3232EP(v3_1, v3_2);
    v4Set(0.8426692196316502, 0.05352444488005169, -0.3699741829975614, 0.3875084824890354, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Euler3232EP failed";
    Euler3232Gibbs(v3_1, v3_2);
    v3Set(0.06351774057138154, -0.4390503110571495, 0.4598583565902931, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3232Gibbs failed";
    Euler3232MRP(v3_1, v3_2);
    v3Set(0.02904723447366817, -0.2007816590497557, 0.2102973655610845, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3232MRP failed";
    Euler3232PRV(v3_1, v3_2);
    v3Set(0.1130418386266624, -0.7813731087574279, 0.8184049632304388, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Euler3232PRV failed";

    v3Set(0.5746957711326909, -0.7662610281769212, 0.2873478855663454, v3_1);
    Gibbs2C(v3_1, C);
    v3Set(0.3302752293577981, -0.1530190869107189, 0.9313986428558203, C2[0]);
    v3Set(-0.7277148580434096, 0.5871559633027522, 0.3545122848941588, C2[1]);
    v3Set(-0.6011234134980221, -0.794879257371223, 0.08256880733944938, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Gibbs2C failed";
    Gibbs2EP(v3_1, v3_2);
    v4Set(0.7071067811865475, 0.4063712768871578, -0.5418283691828771, 0.2031856384435789, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "Gibbs2EP failed";
    Gibbs2Euler121(v3_1, v3_2);
    v3Set(3.304427597008361, 1.234201174364066, -2.26121636963008, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler121 failed";
    Gibbs2Euler123(v3_1, v3_2);
    v3Set(1.467291629150036, -0.6449061163953342, 1.144743256726005, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler123 failed";
    Gibbs2Euler131(v3_1, v3_2);
    v3Set(1.733631270213465, 1.234201174364066, -0.6904200428351842, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler131 failed";
    Gibbs2Euler132(v3_1, v3_2);
    v3Set(0.54319335066115, 0.8149843403384446, -1.068390851022488, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler132 failed";
    Gibbs2Euler212(v3_1, v3_2);
    v3Set(-1.117474807766432, 0.9432554204540935, -0.1901795897648197, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler212 failed";
    Gibbs2Euler213(v3_1, v3_2);
    v3Set(-1.434293025994105, 0.9188085603647974, -0.2549399408440935, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler213 failed";
    Gibbs2Euler231(v3_1, v3_2);
    v3Set(-1.230028192223063, -0.1536226209659692, 0.9345839026955233, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler231 failed";
    Gibbs2Euler232(v3_1, v3_2);
    v3Set(0.4533215190284649, 0.9432554204540935, -1.760975916559716, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler232 failed";
    Gibbs2Euler312(v3_1, v3_2);
    v3Set(0.8918931304028546, 0.3623924238788913, -1.482377127697951, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler312 failed";
    Gibbs2Euler313(v3_1, v3_2);
    v3Set(-0.6474859022891233, 1.488133410155628, 1.207104533714101, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler313 failed";
    Gibbs2Euler321(v3_1, v3_2);
    v3Set(-0.4338654111289937, -1.198236565236741, 1.341967642658489, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler321 failed";
    Gibbs2Euler323(v3_1, v3_2);
    v3Set(-2.21828222908402, 1.488133410155628, 2.777900860508998, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2Euler321 failed";
    Gibbs2MRP(v3_1, v3_2);
    v3Set(0.2380467826416248, -0.3173957101888331, 0.1190233913208124, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2MRP failed";
    Gibbs2PRV(v3_1, v3_2);
    v3Set(0.9027300063197914, -1.203640008426389, 0.4513650031598956, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2PRV failed";
    v3Set(0.0, 0.0, 0.0, v3_1);
    Gibbs2PRV(v3_1, v3_2);
    v3Set(0.0, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "Gibbs2PRV failed";

    v3Set(0.2, -0.25, 0.3, v3_1);
    MRP2C(v3_1, C);
    v3Set(0.1420873822677549, 0.4001248192538094, 0.9053790945330048, C2[0]);
    v3Set(-0.9626904702257736, 0.2686646537364468, 0.03234752493088797, C2[1]);
    v3Set(-0.2303003133666478, -0.876196001388834, 0.4233702077537369, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "MRP2C failed";
    MRP2EP(v3_1, v3_2);
    v4Set(0.6771488469601677, 0.3354297693920336, -0.419287211740042, 0.5031446540880503, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "MRP2EP failed";
    MRP2Euler121(v3_1, v3_2);
    v3Set(2.725460144813494, 1.428226451915784, -1.805609061169705, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler121 failed";
    MRP2Euler123(v3_1, v3_2);
    v3Set(1.120685944613971, -0.2323862804943196, 1.424260216144192, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler123 failed";
    MRP2Euler131(v3_1, v3_2);
    v3Set(1.154663818018597, 1.428226451915784, -0.2348127343748092, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler131 failed";
    MRP2Euler132(v3_1, v3_2);
    v3Set(0.1198243320629901, 1.296774918090265, -1.017995395279125, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler132 failed";
    MRP2Euler212(v3_1, v3_2);
    v3Set(-1.537207795170527, 1.298789879764913, 0.4283796513241308, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler212 failed";
    MRP2Euler213(v3_1, v3_2);
    v3Set(-0.4982011776145131, 1.067911809027856, 0.979488037955722, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler213 failed";
    MRP2Euler231(v3_1, v3_2);
    v3Set(-1.415129132201094, 0.4116530390866675, 1.273271587093173, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler231 failed";
    MRP2Euler232(v3_1, v3_2);
    v3Set(0.03358853162436948, 1.298789879764913, -1.142416675470766, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler232 failed";
    MRP2Euler312(v3_1, v3_2);
    v3Set(1.298643836753137, 0.03235316879424937, -1.133389474325039, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler312 failed";
    MRP2Euler313(v3_1, v3_2);
    v3Set(-0.257027406977469, 1.133634172515794, 1.535083362165219, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler313 failed";
    MRP2Euler321(v3_1, v3_2);
    v3Set(1.22957853325386, -1.13227169191098, 0.0762566635156139, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler321 failed";
    MRP2Euler323(v3_1, v3_2);
    v3Set(-1.827823733772366, 1.133634172515794, 3.105879688960115, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Euler321 failed";
    MRP2Gibbs(v3_1, v3_2);
    v3Set(0.4953560371517029, -0.6191950464396285, 0.7430340557275542, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2Gibbs failed";
    MRP2PRV(v3_1, v3_2);
    v3Set(0.7538859486650076, -0.9423574358312593, 1.130828922997511, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2PRV failed";

    MRPswitch(v3_1, 1, v3_2);
    v3Set(0.2, -0.25, 0.3, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2PRV failed";
    MRPswitch(v3_1, 0.4, v3_2);
    v3Set(-1.038961038961039, 1.298701298701299, -1.558441558441558, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2PRV failed";
    v3Set(0.0, 0.0, 0.0, v3_1);
    MRP2PRV(v3_1, v3_2);
    v3Set(0.0, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2PRV failed";
    v3Set(1.0, 0.0, 0.0, v3_1);
    MRP2PRV(v3_1, v3_2);
    v3Set(std::numbers::pi, 0.0, 0.0, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "MRP2PRV failed";

    EXPECT_TRUE(isEqual(wrapToPi(1.2), 1.2, accuracy)) << "wrapToPi(1.2) failed";
    EXPECT_TRUE(isEqual(wrapToPi(4.2), -2.083185307179586, accuracy)) << "wrapToPi(4.2) failed";
    EXPECT_TRUE(isEqual(wrapToPi(-4.2), 2.083185307179586, accuracy)) << "wrapToPi(-4.2) failed";

    v3Set(0.2, -0.25, 0.3, v3_1);
    PRV2C(v3_1, C);
    v3Set(0.9249653552860658, 0.2658656942983466, 0.2715778417245783, C2[0]);
    v3Set(-0.3150687400124018, 0.9360360405717283, 0.1567425271513747, C2[1]);
    v3Set(-0.212534186867712, -0.2305470957224576, 0.9495668781430935, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "PRV2C failed";
    PRV2EP(v3_1, v3_2);
    v4Set(0.9760338459808767, 0.09919984446969178, -0.1239998055871147, 0.1487997667045377, v3);
    EXPECT_TRUE(vIsEqual(v3_2, 4, v3, accuracy)) << "PRV2EP failed";
    PRV2Euler121(v3_1, v3_2);
    v3Set(2.366822457545908, 0.3898519008736288, -2.164246748437291, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler121 failed";
    PRV2Euler123(v3_1, v3_2);
    v3Set(0.2381830975647435, -0.2141676691157164, 0.3283009769818029, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler123 failed";
    PRV2Euler131(v3_1, v3_2);
    v3Set(0.796026130751012, 0.3898519008736288, -0.5934504216423945, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler131 failed";
    PRV2Euler132(v3_1, v3_2);
    v3Set(0.1659141638227202, 0.3205290820781828, -0.2258549616703266, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler132 failed";
    PRV2Euler212(v3_1, v3_2);
    v3Set(-1.109161329065078, 0.3596045976550934, 0.8564261174295806, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler212 failed";
    PRV2Euler213(v3_1, v3_2);
    v3Set(-0.2201931522496843, 0.2326398873102022, 0.2767451364802878, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler213 failed";
    PRV2Euler231(v3_1, v3_2);
    v3Set(-0.2855829177825758, 0.269101825006778, 0.2414947191533679, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler231 failed";
    PRV2Euler232(v3_1, v3_2);
    v3Set(0.4616349977298192, 0.3596045976550934, -0.714370209365316, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler232 failed";
    PRV2Euler312(v3_1, v3_2);
    v3Set(0.3246867163622526, 0.1573915425330904, -0.2785654591200913, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler312 failed";
    PRV2Euler313(v3_1, v3_2);
    v3Set(-0.7447668031423726, 0.3189446151924337, 1.047343966000315, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler313 failed";
    PRV2Euler321(v3_1, v3_2);
    v3Set(0.2798880637473677, -0.2750321114914171, 0.1635922230133545, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler321 failed";
    PRV2Euler323(v3_1, v3_2);
    v3Set(-2.315563129937269, 0.3189446151924337, 2.618140292795212, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Euler321 failed";
    PRV2Gibbs(v3_1, v3_2);
    v3Set(0.1016356603597079, -0.1270445754496348, 0.1524534905395618, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2Gibbs failed";
    PRV2MRP(v3_1, v3_2);
    v3Set(0.05020149056224809, -0.06275186320281011, 0.07530223584337212, v3);
    EXPECT_TRUE(v3IsEqual(v3_2, v3, accuracy)) << "PRV2MRP failed";

    v4Set(0.45226701686665, 0.75377836144441, 0.15075567228888, 0.45226701686665, v3_1);
    v4Set(-0.18663083698528, 0.46657709246321, 0.83983876643378, -0.20529392068381, v3_2);
    subEP(v3_1, v3_2, v3);
    v4Set(0.3010515331052196, -0.762476312817895, -0.0422034859493331, 0.5711538431809339, v3_1);
    EXPECT_TRUE(vIsEqual(v3, 4, v3_1, accuracy)) << "subEP failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler121(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler121 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler123(v3_1, v3_2, v3);
    v3Set(3.116108453572625, -0.6539785291371149, -0.9652248604105184, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler123 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler131(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler131 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler132(v3_1, v3_2, v3);
    v3Set(2.932019083757663, 0.6246626379494424, -1.519867235625338, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler132 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler212(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler212 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler213(v3_1, v3_2, v3);
    v3Set(2.932019083757663, 0.6246626379494424, -1.519867235625338, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler213 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler231(v3_1, v3_2, v3);
    v3Set(3.116108453572625, -0.6539785291371149, -0.9652248604105185, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler231 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler232(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler232 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler312(v3_1, v3_2, v3);
    v3Set(3.116108453572625, -0.653978529137115, -0.9652248604105184, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler312 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler313(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler313 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler321(v3_1, v3_2, v3);
    v3Set(2.932019083757663, 0.6246626379494424, -1.519867235625338, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler321 failed";

    v3Set(10 * D2R, 20 * D2R, 30 * D2R, v3_1);
    v3Set(-30 * D2R, 200 * D2R, 81 * D2R, v3_2);
    subEuler323(v3_1, v3_2, v3);
    v3Set(2.969124082346242, 2.907100217278789, 2.423943306316236, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subEuler323 failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    subGibbs(v3_1, v3_2, v3);
    v3Set(4.333333333333333, -0.5, 2.166666666666667, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subGibbs failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    subMRP(v3_1, v3_2, v3);
    v3Set(-0.005376344086021518, 0.04301075268817203, -0.4408602150537635, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subMRP failed";

    v3Set(0.0, 0.0, 1.0, v3_1);
    v3Set(0.0, 0.0, -1.0, v3_2);
    subMRP(v3_1, v3_2, v3);
    v3Set(0.0, 0.0, 0.0, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subMRP 360 subtraction failed";

    v3Set(1.5, 0.5, 0.5, v3_1);
    v3Set(-0.5, 0.25, 0.15, v3_2);
    subPRV(v3_1, v3_2, v3);
    v3Set(1.899971363060601, 0.06138537390284331, 0.7174863730592785, v3_1);
    EXPECT_TRUE(v3IsEqual(v3, v3_1, accuracy)) << "subPRV failed";

    v3Set(30 * D2R, 30 * D2R, 45 * D2R, v3_1);
    Euler3132C(v3_1, C);
    v3Set(0.306186217848, 0.883883476483, 0.353553390593, C2[0]);
    v3Set(-0.918558653544, 0.176776695297, 0.353553390593, C2[1]);
    v3Set(0.250000000000, -0.433012701892, 0.866025403784, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3132C failed";

    v3Set(60 * D2R, 30 * D2R, 45 * D2R, v3_1);
    Euler3212C(v3_1, C);
    v3Set(0.433012701892, 0.750000000000, -0.500000000000, C2[0]);
    v3Set(-0.435595740399, 0.659739608441, 0.612372435696, C2[1]);
    v3Set(0.789149130992, -0.0473671727454, 0.612372435696, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Euler3212C failed";

    Mi(30 * D2R, 3, C);
    v3Set(0.8660254037844387, 0.4999999999999999, 0, C2[0]);
    v3Set(-0.4999999999999999, 0.8660254037844387, 0, C2[1]);
    v3Set(0, 0, 1.00000000000000, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Mi(30 deg, 3, C) failed";

    Mi(30 * D2R, 2, C);
    v3Set(0.8660254037844387, 0, -0.4999999999999999, C2[0]);
    v3Set(0, 1.00000000000000, 0, C2[1]);
    v3Set(0.4999999999999999, 0, 0.8660254037844387, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Mi(30 deg, 2, C) failed";

    Mi(30 * D2R, 1, C);
    v3Set(1.0000000000000000, 0, 0, C2[0]);
    v3Set(0, 0.8660254037844387, 0.4999999999999999, C2[1]);
    v3Set(0, -0.4999999999999999, 0.8660254037844387, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "Mi(30 deg, 1, C) failed";

    v3Set(1.0, 2.0, 3.0, v3_1);
    tilde(v3_1, C);
    v3Set(0.0, -3.0, 2.0, C2[0]);
    v3Set(3.0, 0.0, -1.0, C2[1]);
    v3Set(-2.0, 1.0, 0.0, C2[2]);
    EXPECT_TRUE(m33IsEqual(C, C2, accuracy)) << "tilde() failed";
}
