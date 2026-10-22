#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

class TDC
{
public:
    TDC() = default;
	~TDC() = default;

    void reset();

    void setParams(const matrix::Vector3f &k1, const matrix::Vector3f &k2,
                                   const matrix::Vector3f &g_bar, float limit);




    matrix::Vector3f update(const matrix::Vector3f &att_target_eul,
                            const matrix::Vector3f &att_target_rate,
                            const matrix::Vector3f &att_eul,
                            const matrix::Vector3f &rates_eul,
                            const matrix::Vector3f &rates_eul_old,
                            float dt);


    // void setTDCGains(const matrix::Vector3f &P);

private:
    matrix::Vector3f _gain_p;

    matrix::Vector3f _k1_;
    matrix::Vector3f _k2_;
    matrix::Vector3f _g_bar_inverse;
    float _limit;

    matrix::Vector3f _u_curr;
    matrix::Vector3f _delta_u;
    matrix::Vector3f _x;
    matrix::Vector3f _x_r;
    matrix::Vector3f _x_dot;
    matrix::Vector3f _x_r_dot;
    matrix::Vector3f _x_r_dot_dot;
    matrix::Vector3f _x_dot_dot_old;
    matrix::Vector3f temp;
};
