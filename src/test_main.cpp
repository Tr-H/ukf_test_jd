#define _USE_MATH_DEFINES
#include <cmath>


#include "ukf_test/SystemModel.hpp"
#include "ukf_test/VioMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>

#include "ros/ros.h"

typedef float T;

typedef Test1::State<T> State;
typedef Test1::Control<T> Control;
typedef Test1::SystemModel<T> SystemModel;

typedef Test1::VioMeasurement<T> VioMeasurement;
typedef Test1::VioMeasurementModel<T> VioModel;

int main(int argc, char** argv) {
    State x;
    x.setZero();
    x.qw() = 1.0f;

    Control u;
    SystemModel sys;
    VioModel vm;

    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<T> noise(0, 1);

    Kalman::UnscentedKalmanFilter<State> ukf(2);

    ukf.init(x);

    T pos_systemNoise = 0.1f;
    T vel_systemNoise = 0.4f;
    T att_systemNoise = 0.05f;
    T measureNoise = 0.05;

    const size_t N = 100;

    int _j = 0;
    for (size_t i = 1; i <= N; i++) {
        u.ax() = std::sin( T(2) * T(M_PI) / T(N) ) * 2.0f;
        u.ay() = std::sin( T(2) * T(M_PI) / T(N) ) * 1.5f;
        u.az() = std::sin( T(2) * T(M_PI) / T(N) ) * 1.0f + 9.8f;
        u.wx() = std::sin( T(2) * T(M_PI) / T(N) ) * 0.1f;
        u.wy() = std::sin( T(2) * T(M_PI) / T(N) ) * 0.2f;
        u.wz() = std::sin( T(2) * T(M_PI) / T(N) ) * 0.1f;
        u.dt() = 0.005f;
        std::cout << "***********" << i << "**************" <<std::endl;
        x = sys.f(x,u);

        x.x() += pos_systemNoise*noise(generator);
        x.y() += pos_systemNoise*noise(generator);
        x.z() += pos_systemNoise*noise(generator);
        x.vx() += vel_systemNoise*noise(generator);
        x.vy() += vel_systemNoise*noise(generator);
        x.vz() += vel_systemNoise*noise(generator);
        x.qw() += att_systemNoise*noise(generator);
        x.qx() += att_systemNoise*noise(generator);
        x.qy() += att_systemNoise*noise(generator);
        x.qz() += att_systemNoise*noise(generator);
        // x.bax() += pos_systemNoise*noise(generator);
        // x.bay() += pos_systemNoise*noise(generator);
        // x.baz() += pos_systemNoise*noise(generator);
        // x.bwx() += pos_systemNoise*noise(generator);
        // x.bwy() += pos_systemNoise*noise(generator);
        // x.bwz() += pos_systemNoise*noise(generator);

        auto x_ukf = ukf.predict(sys, u);

        if ( _j > 10 ) {
            _j = 0;
            VioMeasurement  vio_state = vm.h(x);
            vio_state.vio_x() += measureNoise*noise(generator);
            vio_state.vio_y() += measureNoise*noise(generator);
            vio_state.vio_z() += measureNoise*noise(generator);
            vio_state.vio_qw() += measureNoise*noise(generator);
            vio_state.vio_qx() += measureNoise*noise(generator);
            vio_state.vio_qy() += measureNoise*noise(generator);
            vio_state.vio_qz() += measureNoise*noise(generator);

            x_ukf = ukf.update(vm, vio_state);
        } else {
            _j ++;
        }
        std::cout << "q:  [" << x.qw() <<", "<< x.qx() << ", " << x.qy() << ", " << x.qz() << "]" << std::endl;
        std::cout << "qu: [" << x_ukf.qw() <<", "<< x_ukf.qx() << ", " << x_ukf.qy() << ", " << x_ukf.qz() << "]" << std::endl;
        std::cout << "x:  [" << x.vx() <<", "<< x_ukf.vx() << "]"<< std::endl;
        // std::cout << "y:[" << x.y() <<", "<< x_ukf.y() << "]"<< std::endl;
        // std::cout << "z:[" << x.z() <<", "<< x_ukf.z() << "]"<<std::endl;
    }

}