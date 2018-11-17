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

    Kalman::UnscentedKalmanFilter<State> ukf(1,2,0);
}