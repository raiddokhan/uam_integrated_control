#include "TDC.hpp"

// using namespace matrix;

void TDC::reset() {
        _u_curr.setZero();
        _delta_u.setZero();
        temp.setZero();
        _x_dot_dot_old.setZero();
    }

void TDC::setParams(const matrix::Vector3f &k1, const matrix::Vector3f &k2,
                                   const matrix::Vector3f &g_bar, float limit)
{
    // _k1_matrix.setZero();
    _k1_(0) = k1(0);
    _k1_(1) = k1(1);
    _k1_(2) = k1(2);

    // _k2_matrix.setZero();
    _k2_(0) = k2(0);
    _k2_(1) = k2(1);
    _k2_(2) = k2(2);

    _g_bar_inverse = matrix::Vector3f(
        (g_bar(0) > 0.0f) ? (1.0f / g_bar(0)) : 1e-6f,
        (g_bar(1) > 0.0f) ? (1.0f / g_bar(1)) : 1e-6f,
        (g_bar(2) > 0.0f) ? (1.0f / g_bar(2)) : 1e-6f
    );

    _limit = limit;

}

matrix::Vector3f TDC::update(const matrix::Vector3f &att_target_eul,
                            const matrix::Vector3f &att_target_rate,
                            const matrix::Vector3f &att_eul,
                            const matrix::Vector3f &rates_eul,
                            const matrix::Vector3f &rates_eul_old,
                            float dt)
{
    // matrix::Eulerf euler_angles(matrix::Quatf(att_target_eul.q));
    if (!_k1_.isAllFinite() || !_k2_.isAllFinite()) {
        PX4_ERR("Invalid gain matrices!");
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    if (dt <= 0.0f || dt >= 1.0f) {
        return matrix::Vector3f(0.0f, 0.0f, 0.0f);
    }

    _x = att_eul;
    _x_dot = rates_eul;
    _x_r = att_target_eul;
    _x_r_dot = att_target_rate;


    // low pass filter
    float alpha = 0.5f; //


    matrix::Vector3f x_dot_dot_raw = (rates_eul - rates_eul_old) / dt;
    matrix::Vector3f e1 = _x - _x_r;
    e1(0) = matrix::wrap_pi(e1(0));
    e1(1) = matrix::wrap_pi(e1(1));
    e1(2) = matrix::wrap_pi(e1(2));
    matrix::Vector3f e1_dot = _x_dot - _x_r_dot;

    matrix::Vector3f temp_raw = - _k1_.emult(e1) - _k2_.emult(e1_dot) - x_dot_dot_raw;

    temp = alpha * temp + (1.0f - alpha) * temp_raw;


    _delta_u = matrix::Vector3f(
        temp(0) * _g_bar_inverse(0),
        temp(1) * _g_bar_inverse(1),
        temp(2) * _g_bar_inverse(2)
    );

    _u_curr = _u_curr + _delta_u;



    _u_curr(0) = math::constrain(_u_curr(0), -_limit, _limit);
    _u_curr(1) = math::constrain(_u_curr(1), -_limit, _limit);
    _u_curr(2) = math::constrain(_u_curr(2), -_limit, _limit);

    return _u_curr;

}
