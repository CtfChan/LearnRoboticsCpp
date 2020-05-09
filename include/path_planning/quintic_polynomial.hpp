# pragma once

#include <vector>

struct State {
    float x;
    float y;
    float yaw;
    float v;
    float a;
};

struct QuinticPolynomialTrajectory {
    std::vector<float> time; 
    std::vector<float> x; 
    std::vector<float> y; 
    std::vector<float> yaw; 
    std::vector<float> v; 
    std::vector<float> a; 
    std::vector<float> j; 
};

class QuinticPolynomial {
public:
    // boundary conditions and time
    QuinticPolynomial(float xs, float vs, float as, float xe, float ve, float ae, float T);

    float calculatePoint(float t);

    float calculateFirstDerivative(float t);

    float calculateSecondDerivative(float t);

    float calculateThirdDerivative(float t);

private:
    float a0_, a1_, a2_, a3_, a4_, a5_;

};


QuinticPolynomialTrajectory quinticPolynomialPlanner(State& start, State& goal,
    float min_T, float max_T, float T_res, float max_accel, float max_jerk, float dt);

