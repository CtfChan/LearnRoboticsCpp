# pragma once

# include <Eigen/Eigen>


struct SplineCourse {
    std::vector<float> rx, ry, ryaw, rk, s;
};

SplineCourse calculateSplineCourse(const std::vector<float>& x, 
    const std::vector<float>& y, float ds=0.1);


class CubicSpline {
public:
    CubicSpline(const std::vector<float>& x, const std::vector<float>& y);

    float calculatePosition(float t);

    float calculateFirstDerivative(float t);

    float calculateSecondDerivative(float t);

private:

    size_t searchIndex(float x);

    Eigen::MatrixXf generateA(const std::vector<float>& h);

    Eigen::VectorXf generateB(const std::vector<float>& h);

    Eigen::VectorXf a_, b_, c_, d_;

    std::vector<float> x_, y_, h_;

    size_t nx_;

};

class CubicSpline2D {
public:
    CubicSpline2D(const std::vector<float>& x, const std::vector<float>& y);

    std::pair<float, float> calculatePosition(float s);

    float calculateCurvature(float s);

    float calculateYaw(float s);

    const std::vector<float>& getS() const {
        return s_;
    }

private:
    std::vector<float> calculateS(const std::vector<float>& x, 
                                  const std::vector<float>& y);

    std::vector<float> s_;
    CubicSpline sx_;
    CubicSpline sy_;

};

