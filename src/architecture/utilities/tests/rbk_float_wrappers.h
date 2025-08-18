#ifndef RBK_FLOAT_WRAPPERS_H
#define RBK_FLOAT_WRAPPERS_H

#include "architecture/utilities/rigidBodyKinematics.h"

// Generic float wrapper macros
#define DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(name)                          \
    inline void name##_float(const float* in, float* out) {                \
        double in_d[3], out_d[3];                                          \
        for (int i = 0; i < 3; ++i) in_d[i] = static_cast<double>(in[i]);  \
        name(in_d, out_d);                                                 \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]); \
    }

#define DEFINE_FLOAT_WRAPPER_VEC3IN_VEC4OUT(name)                          \
    inline void name##_float(const float* in, float* out) {                \
        double in_d[3], out_d[4];                                          \
        for (int i = 0; i < 3; ++i) in_d[i] = static_cast<double>(in[i]);  \
        name(in_d, out_d);                                                 \
        for (int i = 0; i < 4; ++i) out[i] = static_cast<float>(out_d[i]); \
    }

#define DEFINE_FLOAT_WRAPPER_VEC4IN_VEC3OUT(name)                          \
    inline void name##_float(const float* in, float* out) {                \
        double in_d[4], out_d[3];                                          \
        for (int i = 0; i < 4; ++i) in_d[i] = static_cast<double>(in[i]);  \
        name(in_d, out_d);                                                 \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]); \
    }

#define DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(name)                         \
    inline void name##_float(float* in1, float* in2, float* out) {         \
        double in1_d[3], in2_d[3], out_d[3];                               \
        for (int i = 0; i < 3; ++i) {                                      \
            in1_d[i] = static_cast<double>(in1[i]);                        \
            in2_d[i] = static_cast<double>(in2[i]);                        \
        }                                                                  \
        name(in1_d, in2_d, out_d);                                         \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]); \
    }

#define DEFINE_FLOAT_WRAPPER_2VEC4IN_VEC4OUT(name)                         \
    inline void name##_float(float* in1, float* in2, float* out) {         \
        double in1_d[4], in2_d[4], out_d[4];                               \
        for (int i = 0; i < 4; ++i) {                                      \
            in1_d[i] = static_cast<double>(in1[i]);                        \
            in2_d[i] = static_cast<double>(in2[i]);                        \
        }                                                                  \
        name(in1_d, in2_d, out_d);                                         \
        for (int i = 0; i < 4; ++i) out[i] = static_cast<float>(out_d[i]); \
    }

#define DEFINE_FLOAT_WRAPPER_3VEC3IN_VEC3OUT(name)                                               \
    inline void name##_float(const float* in1, const float* in2, const float* in3, float* out) { \
        double in1_d[3], in2_d[3], in3_d[3], out_d[3];                                           \
        for (int i = 0; i < 3; ++i) {                                                            \
            in1_d[i] = static_cast<double>(in1[i]);                                              \
            in2_d[i] = static_cast<double>(in2[i]);                                              \
            in3_d[i] = static_cast<double>(in3[i]);                                              \
        }                                                                                        \
        name(in1_d, in2_d, in3_d, out_d);                                                        \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]);                       \
    }

#define DEFINE_FLOAT_WRAPPER_4VEC3IN_VEC3OUT(name)                                                                 \
    inline void name##_float(const float* in1, const float* in2, const float* in3, const float* in4, float* out) { \
        double in1_d[3], in2_d[3], in3_d[3], in4_d[3], out_d[3];                                                   \
        for (int i = 0; i < 3; ++i) {                                                                              \
            in1_d[i] = static_cast<double>(in1[i]);                                                                \
            in2_d[i] = static_cast<double>(in2[i]);                                                                \
            in3_d[i] = static_cast<double>(in3[i]);                                                                \
            in4_d[i] = static_cast<double>(in4[i]);                                                                \
        }                                                                                                          \
        name(in1_d, in2_d, in3_d, in4_d, out_d);                                                                   \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]);                                         \
    }

#define DEFINE_FLOAT_WRAPPER_MAT33IN_VEC4OUT(name)                                  \
    inline void name##_float(const float in[3][3], float* out) {                    \
        double in_d[3][3], out_d[4];                                                \
        for (int i = 0; i < 3; ++i)                                                 \
            for (int j = 0; j < 3; ++j) in_d[i][j] = static_cast<double>(in[i][j]); \
        name(in_d, out_d);                                                          \
        for (int i = 0; i < 4; ++i) out[i] = static_cast<float>(out_d[i]);          \
    }

#define DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(name)                                  \
    inline void name##_float(const float in[3][3], float* out) {                    \
        double in_d[3][3], out_d[3];                                                \
        for (int i = 0; i < 3; ++i)                                                 \
            for (int j = 0; j < 3; ++j) in_d[i][j] = static_cast<double>(in[i][j]); \
        name(in_d, out_d);                                                          \
        for (int i = 0; i < 3; ++i) out[i] = static_cast<float>(out_d[i]);          \
    }

#define DEFINE_FLOAT_WRAPPER_VEC4IN_MAT33OUT(name)                                   \
    inline void name##_float(const float* in, float out[3][3]) {                     \
        double in_d[4], out_d[3][3];                                                 \
        for (int i = 0; i < 4; ++i) in_d[i] = static_cast<double>(in[i]);            \
        name(in_d, out_d);                                                           \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(name)                                   \
    inline void name##_float(const float* in, float out[3][3]) {                     \
        double in_d[3], out_d[3][3];                                                 \
        for (int i = 0; i < 3; ++i) in_d[i] = static_cast<double>(in[i]);            \
        name(in_d, out_d);                                                           \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_VEC4IN_MAT34OUT(name)                                   \
    inline void name##_float(const float* in, float out[3][4]) {                     \
        double in_d[4], out_d[3][4];                                                 \
        for (int i = 0; i < 4; ++i) in_d[i] = static_cast<double>(in[i]);            \
        name(in_d, out_d);                                                           \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 4; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_VEC4IN_MAT43OUT(name)                                   \
    inline void name##_float(const float* in, float out[4][3]) {                     \
        double in_d[4], out_d[4][3];                                                 \
        for (int i = 0; i < 4; ++i) in_d[i] = static_cast<double>(in[i]);            \
        name(in_d, out_d);                                                           \
        for (int i = 0; i < 4; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_2VEC3IN_MAT33OUT(name)                                  \
    inline void name##_float(float* in1, float* in2, float out[3][3]) {              \
        double in1_d[3], in2_d[3], out_d[3][3];                                      \
        for (int i = 0; i < 3; ++i) {                                                \
            in1_d[i] = static_cast<double>(in1[i]);                                  \
            in2_d[i] = static_cast<double>(in2[i]);                                  \
        }                                                                            \
        name(in1_d, in2_d, out_d);                                                   \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_SCALAR_MATRIX(name)                                     \
    inline void name##_float(const float x, int axis, float out[3][3]) {             \
        double out_d[3][3];                                                          \
        name(static_cast<double>(x), axis, out_d);                                   \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_TILDE(name)                                             \
    inline void name##_float(const float* v, float out[3][3]) {                      \
        double v_d[3], out_d[3][3];                                                  \
        for (int i = 0; i < 3; ++i) v_d[i] = static_cast<double>(v[i]);              \
        name(v_d, out_d);                                                            \
        for (int i = 0; i < 3; ++i)                                                  \
            for (int j = 0; j < 3; ++j) out[i][j] = static_cast<float>(out_d[i][j]); \
    }

#define DEFINE_FLOAT_WRAPPER_SCALAR(name) \
    inline float name##_float(float x) { return static_cast<float>(name(static_cast<double>(x))); }

inline void MRPswitch_float(float* q, const float s2, float* s) {
    double tmp_q[3], tmp_out[3];
    for (int i = 0; i < 3; ++i) tmp_q[i] = static_cast<double>(q[i]);
    MRPswitch(tmp_q, s2, tmp_out);
    for (int i = 0; i < 3; ++i) s[i] = static_cast<float>(tmp_out[i]);
}

DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addMRP)
DEFINE_FLOAT_WRAPPER_2VEC4IN_VEC4OUT(addEP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addEuler321)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler312)
DEFINE_FLOAT_WRAPPER_2VEC4IN_VEC4OUT(subEP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subGibbs)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subMRP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subPRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(MRPshadow)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvMRP)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvPRV)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(dMRP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(dMRP2Omega)
DEFINE_FLOAT_WRAPPER_4VEC3IN_VEC3OUT(ddMRP)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC4OUT(C2EP)
DEFINE_FLOAT_WRAPPER_TILDE(tilde)
DEFINE_FLOAT_WRAPPER_SCALAR_MATRIX(Mi)
DEFINE_FLOAT_WRAPPER_SCALAR(wrapToPi)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addGibbs)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addPRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatGibbs)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatPRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatMRP)
DEFINE_FLOAT_WRAPPER_VEC4IN_MAT43OUT(BmatEP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_MAT33OUT(BdotmatMRP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(dPRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(Gibbs2C)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(PRV2C)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC4OUT(MRP2EP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(MRP2PRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(MRP2Gibbs)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(MRP2Euler321)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(MRP2C)
DEFINE_FLOAT_WRAPPER_VEC4IN_VEC3OUT(EP2MRP)
DEFINE_FLOAT_WRAPPER_VEC4IN_VEC3OUT(EP2Gibbs)
DEFINE_FLOAT_WRAPPER_VEC4IN_VEC3OUT(EP2PRV)
DEFINE_FLOAT_WRAPPER_VEC4IN_VEC3OUT(EP2Euler321)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC4OUT(Euler3212EP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(Euler3212MRP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(Euler3212PRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(Euler3212C)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC4OUT(Gibbs2EP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(Gibbs2PRV)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(Gibbs2MRP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC4OUT(PRV2EP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(PRV2Gibbs)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(PRV2MRP)
DEFINE_FLOAT_WRAPPER_VEC3IN_VEC3OUT(PRV2Euler321)
DEFINE_FLOAT_WRAPPER_2VEC4IN_VEC4OUT(dEP)
DEFINE_FLOAT_WRAPPER_VEC4IN_MAT33OUT(EP2C)
DEFINE_FLOAT_WRAPPER_VEC4IN_MAT34OUT(BinvEP)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addEuler123)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(addEuler313)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler121)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler123)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler131)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler132)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler212)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler213)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler231)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler232)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler313)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler321)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(subEuler323)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler121)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler123)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler131)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler132)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler212)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler213)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler231)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler232)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler312)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler313)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler321)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BinvEuler323)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler121)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler123)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler131)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler132)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler212)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler213)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler231)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler232)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler312)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler313)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler321)
DEFINE_FLOAT_WRAPPER_VEC3IN_MAT33OUT(BmatEuler323)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler121)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler123)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler131)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler132)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler212)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler213)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler231)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler232)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler312)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler313)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler321)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2Euler323)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2MRP)
DEFINE_FLOAT_WRAPPER_MAT33IN_VEC3OUT(C2PRV)
DEFINE_FLOAT_WRAPPER_2VEC3IN_VEC3OUT(dEuler321)
DEFINE_FLOAT_WRAPPER_3VEC3IN_VEC3OUT(ddMRP2dOmega)

#endif
