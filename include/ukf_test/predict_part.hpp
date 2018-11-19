#ifndef PREDICT_PART_HPP_
#define PREDICT_PART_HPP_
// #include "ros/ros.h"
// #include "ukf_test/SystemModel.hpp"
// #include "ukf_test/VioMeasurementModel.hpp"
// #include "geometry_math_type.h"
#include "ukf_test/update_part.hpp"
#include "sensor_msgs/Imu.h"

// #include <kalman/UnscentedKalmanFilter.hpp>

template<typename T, class State, class Control, class SystemModel, class VioMeasurement, class VioModel>
class Filter_predict_part : public Filter_update_part<T, State, Control, SystemModel, VioMeasurement, VioModel>{
    public:
        Filter_predict_part():
        _first_predict(true),
        nh_predict_("~filter_predict") {
            this->set_predict_valid(false);
            imu_sub = nh_predict_.subscribe("/imu", 10, &Filter_predict_part::imu_update_cb, this);
        }

        ~Filter_predict_part() {

        }

        void imu_update_cb(const sensor_msgs::Imu& msg) {
            if (_first_predict) {
                _last_timestamp = msg.header.stamp;
                _first_predict = false;
            } else {
                double dt = (msg.header.stamp - _last_timestamp).toSec();
                if ( dt > 0.005f && dt < 1.0f) {
                    _last_timestamp = msg.header.stamp;
                    if (this->get_update_init_state()) {
                        Control _C;
                        _C.ax() = msg.linear_acceleration.x;
                        _C.ay() = msg.linear_acceleration.y;
                        _C.az() = msg.linear_acceleration.z;
                        _C.wx() = msg.angular_velocity.x;
                        _C.wy() = msg.angular_velocity.y;
                        _C.wz() = msg.angular_velocity.z;
                        _C.dt() = dt;
                        this->predict_process(_C);
                        this->set_predict_valid(true);
                    }
                } else if (dt >= 1.0f) {
                    _first_predict = true;
                    this->reinit();
                }
            }
        }


    private:
        ros::NodeHandle nh_predict_;
        ros::Subscriber imu_sub;
        bool _first_predict;
        ros::Time _last_timestamp;
};

#endif