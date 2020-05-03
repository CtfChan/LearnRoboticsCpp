# pragma once

# include <vector>

struct State {
    float x;
    float y;
    float yaw;
    float v;
    float a;
};

struct StateWithTime : State {
    float time;
};

class QuinticPolynomial {
public:
    // boundary conditions and time
    QuinticPolynomial(float xs, float vs, float as, float xe, float ve, float ae, float time);

    float calculatePoint(float t);

    float calculateFirstDerivative(float t);

    float calculateSecondDerivative(float t);

    float calculateThirdDerivative(float t);

private:
    float a0, a1, a2, a3, a4, a5;

};


std::vector<StateWithTime> quinticPolynomialPlanner(State& start, State& goal,
    float min_T, float max_T, float T_res, float max_accel, float max_jerk, float dt);

