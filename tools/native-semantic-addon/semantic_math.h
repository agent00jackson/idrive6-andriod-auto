// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
/* Self-contained single-precision trig/sqrt helpers (no QNX libm.so.2 stub
 * dependency for cos/cosf/sinf/atan2/atan2f). Accuracy < 1e-5 rad (< 0.001 deg). */
#ifndef NBT_SEMANTIC_MATH_H
#define NBT_SEMANTIC_MATH_H

static inline float m_fabsf(float x) { return x < 0.0f ? -x : x; }

static inline float m_sqrtf(float x) {
    if (x <= 0.0f) return 0.0f;
    double d = (double)x;
    double g = d > 1.0 ? d * 0.5 : 1.0;
    for (int i = 0; i < 16; ++i)
        g = 0.5 * (g + d / g);
    return (float)g;
}

static inline float m_sinf(float x) {
    const float PI = 3.14159265358979323846f;
    const float TWO_PI = 6.28318530717958647692f;
    const float HALF_PI = 1.57079632679489661923f;
    /* Reduce to [-PI, PI] */
    int k = (int)(x / TWO_PI);
    x -= (float)k * TWO_PI;
    if (x > PI) x -= TWO_PI;
    else if (x < -PI) x += TWO_PI;
    /* Fold to [-PI/2, PI/2] */
    if (x > HALF_PI) x = PI - x;
    else if (x < -HALF_PI) x = -PI - x;
    float x2 = x * x;
    /* Degree-9 Taylor on [-pi/2, pi/2]: max error < 1e-6 */
    return x * (1.0f + x2 * (-0.16666667f + x2 * (0.0083333310f +
                x2 * (-0.0001984090f + x2 * 0.0000027526f))));
}

static inline float m_cosf(float x) {
    return m_sinf(x + 1.57079632679489661923f);
}

static inline float m_atan_unit(float z) {
    /* Minimax approximation for atan(z) on [-1, 1], max error < 5e-5 rad */
    float z2 = z * z;
    return z * (0.9998660f + z2 * (-0.3302995f + z2 * (0.1801410f +
                z2 * (-0.0851330f + z2 * 0.0208351f))));
}

static inline float m_atan2f(float y, float x) {
    const float PI = 3.14159265358979323846f;
    const float HALF_PI = 1.57079632679489661923f;
    if (x == 0.0f) {
        if (y > 0.0f) return HALF_PI;
        if (y < 0.0f) return -HALF_PI;
        return 0.0f;
    }
    float ax = m_fabsf(x), ay = m_fabsf(y);
    float a;
    if (ax >= ay) {
        a = m_atan_unit(ay / ax);
    } else {
        a = HALF_PI - m_atan_unit(ax / ay);
    }
    if (x < 0.0f) a = PI - a;
    return y < 0.0f ? -a : a;
}

#endif /* NBT_SEMANTIC_MATH_H */
