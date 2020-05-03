# pragma once

# include <Eigen/Core>

class CubicSpline {
public:
    CubicSpline(const std::vector<float>& x, const std::vector<float>& y);

    float calculatePosition(float t);

    float calculateFirstDerivative(float t);

    float calculateSecondDerivative(float t);

private:

    size_t searchIndex(float x);

    Eigen::MatrixXf generateA();

    Eigen::VectorXf generateB();

    Eigen::VectorXf a_, b_, c_, d_;

};

class CubicSpline2D {
public:
    CubicSpline2D(const std::vector<float>& x, const std::vector<float>& y);

    std::pair<float, float> calculatePosition(float s);

    float calculateCurvature(float s);

    float calculateYaw(float s);

private:
    std::vector<float> calculateS();

    std::vector<float> s_;

};

