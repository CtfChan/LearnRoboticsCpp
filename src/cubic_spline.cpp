#include "cubic_spline.hpp"

#include <numeric>
#include <iostream>



SplineCourse calculateSplineCourse(const std::vector<float>& x, 
    const std::vector<float>& y, float ds) {
    
    CubicSpline2D sp(x, y);
    std::vector<float> s = sp.getS();

    // generate what you need for plotting
    std::vector<float> rx, ry, ryaw, rk, rs;

    for (float is = 0; is < s.back(); is += ds) {
        rs.push_back(is);
        auto [ix, iy] = sp.calculatePosition(is);
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(sp.calculateYaw(is));
        rk.push_back(sp.calculateCurvature(is));
    }

    return {rx, ry, ryaw, rk, rs};
}




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
                 3.f * (a_(i+1) - a_(i)) / h[i];
    }
       
    return B;
}

size_t CubicSpline::searchIndex(float x) {
    // first elem, greater than x
    auto it = std::upper_bound(x_.begin(), x_.end(), x);
    it--;
    return std::distance(x_.begin(), it);
}

float CubicSpline::calculatePosition(float t) {
    // check bounds
    if (t < x_.front() || t > x_.back())
        throw std::out_of_range("Outside range of defined domain of spline!");

    size_t i = searchIndex(t);
    float dx = t - x_[i];
    return a_(i) + b_(i)*dx + c_(i)*std::pow(dx,2) + d_(i)*std::pow(dx,3);

}

float CubicSpline::calculateFirstDerivative(float t) {
    // check bounds
    if (t < x_.front() || t > x_.back())
        throw std::out_of_range("Outside defined domain of spline!");

    size_t i = searchIndex(t);
    float dx = t - x_[i];
    return b_(i) + 2.f*c_(i)*dx + 3.f*d_(i)*std::pow(dx, 2);
}

float CubicSpline::calculateSecondDerivative(float t) {
   // check bounds
    if (t < x_.front() || t > x_.back())
        throw std::out_of_range("Outside defined domain of spline!");

    size_t i = searchIndex(t);
    float dx = t - x_[i];
    return 2.f*c_(i) + 6.f*d_(i)*dx;
}



// CubicSpline2D
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

std::pair<float, float> CubicSpline2D::calculatePosition(float s) {
    return {sx_.calculatePosition(s), sy_.calculatePosition(s)};
}

float CubicSpline2D::calculateCurvature(float s) {
    float dx = sx_.calculateFirstDerivative(s);
    float ddx = sx_.calculateSecondDerivative(s);
    
    float dy = sy_.calculateFirstDerivative(s);
    float ddy = sy_.calculateSecondDerivative(s);

    float k = (ddy * dx - ddx * dy) /  
        std::pow(std::pow(dx, 2) + std::pow(dy, 2), 3.f/2.f) ;
    
    return k;
}   

float CubicSpline2D::calculateYaw(float s) {
    float dx = sx_.calculateFirstDerivative(s);
    float dy = sy_.calculateFirstDerivative(s);
    float yaw = std::atan2(dy, dx);
    return yaw;
}


