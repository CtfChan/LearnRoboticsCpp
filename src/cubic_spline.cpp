#include "cubic_spline.hpp"

#include <numeric>



#include <iostream>




CubicSpline::CubicSpline(const std::vector<float>& x, 
    const std::vector<float>& y) : x_(x), y_(y) {
    //
    nx_ = x.size();
    h_.resize(nx_);
    std::adjacent_difference(x.begin(), x.end(), h_.begin());
    h_.erase(h_.begin());

    // caculate ai
    a_.resize(y_.size());
    for (size_t i = 0; i < y_.size(); ++i)
        a_(i) = y_[i];

    // calculate ci with Ac=B
    Eigen::MatrixXf A = generateA(h_);
    Eigen::VectorXf B = generateB(h_);
    c_ = A.colPivHouseholderQr().solve(B);

    // cacluate bi, di
    d_.resize(nx_);
    b_.resize(nx_);
    for (size_t i = 0; i < nx_ -1; ++i) {
        d_(i) = (c_(i+1) - c_(i)) / (3.f * h_[i]);
        b_(i) = (a_(i+1) - a_(i)) / h_[i] -
                (1/3.f) * (c_(i+1) + 2 * c_(i)) * h_[i];
    }

    std::cout << a_ << std::endl;
}

Eigen::MatrixXf CubicSpline::generateA(const std::vector<float>& h) {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx_, nx_);
    
    // fill accordingly
    A(0, 0) = 1.f;
    for (size_t i = 0; i < nx_ - 1; ++i) {
        if (i != nx_ - 2)
            A(i+1, i+1) = 2.f * (h[i] + h[i+1]);
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
    }

    A(0, 1) = 0.f;
    A(nx_-1, nx_-2) = 0.f;
    A(nx_-1, nx_-1) = 1.f;

    return A;
}

Eigen::VectorXf CubicSpline::generateB(const std::vector<float>& h) {
    Eigen::VectorXf B = Eigen::VectorXf::Zero(nx_);

    for (size_t i = 0; i < nx_ - 2; ++i ) {
        B(i+1) = 3.f * (a_(i+2) - a_(i+1)) / h[i+1]   -
                 -3.f * (a_(i+1) - a_(i)) / h[i];
    }

    return B;
}






CubicSpline2D::CubicSpline2D(const std::vector<float>& x, const std::vector<float>& y) :
    s_(calculateS(x, y)), sx_(CubicSpline(s_, x)), sy_(CubicSpline(s_, y)) {
}


std::vector<float> CubicSpline2D::calculateS(
    const std::vector<float>& x, const std::vector<float>& y) {
    // find adjacent diff, first elem is diff w/ 0, so erase it
    std::vector<float> dx(x.size());
    std::vector<float> dy(x.size());
    std::adjacent_difference(x.begin(), x.end(), dx.begin());
    std::adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    // cumsum of arc length
    std::vector<float> s(x.size(), 0.0f);
    for (size_t i = 0; i < dx.size(); ++i) {
        float r = std::hypot(dx[i], dy[i]);
        s[i+1] = s[i] + r;
    }

    return s;
}



int main() {
    std::vector<float> x = {-2.5f, 0.0f, 2.5f, 5.0f, 7.5f, 3.0f, -1.0f};
    std::vector<float> y = {0.7f, -6.0f, 5.0f, 6.5f, 0.0f, 5.0f, -2.0f};
    float ds = 0.1f; // [m], distance between each interpolated points

    CubicSpline2D sp(x, y);


    return 0;
}