#include "ekf.hpp"


// x is state, u is control
Eigen::Vector4f motionModel(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt) {
    const static Eigen::Matrix4f F = (Eigen::Matrix4f() << 
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0
    ).finished();

    Eigen::Matrix<float, 4, 2> B;
    B << dt * std::cos(x(2)),  0,
         dt * std::sin(x(2)),  0,
                        0.0,  dt,
                        1.0,  0.0;

    return F * x + B * u;
}

// depends only on state
Eigen::Vector2f observationModel(const Eigen::Vector4f& x) {
    Eigen::Matrix<float, 2, 4> H;
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;
    return H * x;
}




Eigen::Matrix4f jacobianF(const Eigen::Vector4f& x, const Eigen::Vector2f& u, const float dt ) {
    Eigen::Matrix4f J_F = Eigen::Matrix4f::Identity();
    float v = u(0);
    float phi = x(2);
    J_F(0, 2) = -v * std::sin(phi) * dt;
    J_F(0, 3) = std::cos(phi) * dt;
    J_F(1, 2) = v * std::cos(phi) * dt;
    J_F(1, 3) = std::sin(phi) * dt;

    return J_F;
}

Eigen::Matrix<float, 2, 4> jacobianH() {
    Eigen::Matrix<float, 2, 4> J_H;
    J_H << 1.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0;
    return J_H;
}



void extendedKalmanFilter(Eigen::Vector4f& x_est, Eigen::Matrix4f& P_est,
                       const Eigen::Vector2f& u, const Eigen::Vector2f& z, 
                       const Eigen::Matrix4f& Q, const Eigen::Matrix2f& R,
                       const float dt ) {
    // state predict
    Eigen::Vector4f x_pred = motionModel(x_est, u, dt); // 4x1
    Eigen::Matrix4f J_F = jacobianF(x_est, u, dt); //4x4
    Eigen::Matrix4f P_pred = J_F * P_est * J_F.transpose() + Q; // 4x4

    // state update
    Eigen::Vector2f z_pred = observationModel(x_pred);
    Eigen::Vector2f y = z - z_pred;

    Eigen::Matrix<float, 2, 4> J_H = jacobianH();
    Eigen::Matrix2f S = J_H * P_pred * J_H.transpose() + R;
    Eigen::Matrix<float, 4, 2> K  = P_pred *  J_H.transpose() * S.inverse();

    x_est = x_pred + K * y;
    P_est = (Eigen::Matrix4f::Identity()  - K * J_H) * P_pred;
}



Ellipse generateEllipse(const Eigen::Vector4f& x_est, const Eigen::Matrix4f& P_est) {
    Ellipse ep;

    Eigen::Matrix2f p_xy = P_est.block(0, 0, 2, 2); // get cov of x,y
    Eigen::EigenSolver<Eigen::Matrix2f> ces(p_xy);
    Eigen::Matrix2f evals = ces.pseudoEigenvalueMatrix();
    Eigen::Matrix2f evecs = ces.pseudoEigenvectors(); // row major
 
    // biger eval is 0th idx
    float a = std::sqrt(evals(0,0));
    float b = std::sqrt(evals(1,1));

    // use angle of dominant direction w/ x-axis to determine rotation
    float angle = std::atan2(evecs(0, 1), evecs(0, 0));
    Eigen::Matrix2f rot; // rotation matrix
    rot << std::cos(angle), std::sin(angle),
           -std::sin(angle), std::cos(angle);


    for (float t = 0.0; t < 2 * M_PI + 0.1; t += 0.1) {
        Eigen::Vector2f x;
        x << a * std::cos(t), b * std::sin(t);
        Eigen::Vector2f fx = rot * x;
        ep.emplace_back(fx(0) + x_est(0), fx(1) + x_est(1));
    }


    return ep;
}




int main(){

    // simulation parameters
    float sim_time = 50.0f; // s
    float dt = 0.1;

    // control input (v, w) will be constant 
    Eigen::Vector2f u = Eigen::Vector2f(1.0f, 0.1f);

    // noisy control for dr
    Eigen::Vector2f ud = Eigen::Vector2f(1.0f, 0.1f);

    // dead reckoning state (x, y, v, w)
    Eigen::Vector4f x_DR = Eigen::Vector4f::Zero(4);

    // ground truth state (x, y, v, w)
    Eigen::Vector4f x_GT = Eigen::Vector4f::Zero(4);

    // observation z (x, y) coordinates
    Eigen::Vector2f z;

    // estimation of state, covariance of state
    Eigen::Vector4f x_est = Eigen::Vector4f::Zero(4);
    Eigen::Matrix4f P_est = Eigen::Matrix4f::Identity();

    // motion model covariance
    Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
    Q(0,0) = 0.1 * 0.1;
    Q(1,1) = 0.1 * 0.1;
    Q(2,2) =std::pow(deg2rad(1.0), 2)  ;
    Q(3,3) = 0.1 * 0.1;

    // observation model covariance
    Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
    R(0,0)=1.0;
    R(1,1)=1.0;

    float control_noise_v = 1.0;
    float control_noise_w = std::pow(deg2rad(30.0), 2); 

    float obs_noise_x =  0.5 * 0.5;
    float obs_noise_y = 0.5 * 0.5;

    // random number generator
    std::random_device random_device{};
    std::mt19937 generator{random_device()};
    std::normal_distribution<> gaussian(0, 1);

    // initialize plot
    Gnuplot gp;
    Path gt_path;
    Path dr_path;
    Path obs_path;
    Path est_path;

    gp << "set size ratio 1.0\n";
    gp << "set term gif animate\n";
    gp << "set output '../animations/ekf.gif'\n";


    float time = 0.0f;
    while(time <= sim_time){
        time += dt;

        // propagate true motion
        x_GT = motionModel(x_GT, u, dt);
        gt_path.emplace_back(x_GT(0), x_GT(1));

        // propogate noisy dead reckoning
        ud(0) = u(0) + gaussian(generator) * control_noise_v;
        ud(1) = u(1) + gaussian(generator) * control_noise_w;

        x_DR = motionModel(x_DR, ud, dt);
        dr_path.emplace_back(x_DR(0), x_DR(1));

        // generate observation using ground truth + noise
        z(0) = x_GT(0) + gaussian(generator) * obs_noise_x;
        z(1) = x_GT(1) + gaussian(generator) * obs_noise_y;
        obs_path.emplace_back(z(0), z(1));


        // do ekf
        extendedKalmanFilter(x_est, P_est, ud, z, Q, R, dt);
        est_path.emplace_back(x_est(0), x_est(1));

        // generate error ellipse
        Ellipse error_ellipse = generateEllipse(x_est, P_est);

        gp << "plot '-' title 'ground truth' with lines,"
                    "'-' title 'odometry' with lines,"
                    "'-' title 'observation',"
                    "'-' title 'covariance' with lines,"
                    "'-' title 'filter' with lines\n";
        gp.send1d(gt_path);
        gp.send1d(dr_path);
        gp.send1d(obs_path);
        gp.send1d(error_ellipse);
        gp.send1d(est_path);
        sleep(0.2);        

    }

    gp << "set output\n";


}

