#ifndef UPDATA_PART_HPP_
#define UPDATA_PART_HPP_
#include "ros/ros.h"
// #include "ukf_test/SystemModel.hpp"
// #include "ukf_test/VioMeasurementModel.hpp"
#include "geometry_math_type.h"

#include "nav_msgs/Odometry.h"

#include <kalman/UnscentedKalmanFilter.hpp>

template<typename T, class State, class Control, class SystemModel, class VioMeasurement, class VioModel>
class Filter_update_part {
    public:
        // typedef float T;
        // typedef Test1::State<T> State;

        // typedef Test1::Control<T> Control;
        // typedef Test1::SystemModel<T> SystemModel;

        // typedef Test1::VioMeasurement<T> VioMeasurement;
        // typedef Test1::VioMeasurementModel<T> VioModel;

        Filter_update_part():
        nh_("~filter_update") {
            _has_init = false;
            _predict_has_init = false;
            T _alpha = T(1);
            T _beta = T(2);
            T _kappa = T(0);
            _ukf_ptr = new Kalman::UnscentedKalmanFilter<State>(_alpha, _beta, _kappa);
            
            Vio_measure_sub = nh_.subscribe("/vio_odom", 10, &Filter_update_part::vio_update_cb, this);

        }

        ~Filter_update_part() {
            delete _ukf_ptr;
        }

        void vio_update_cb(const nav_msgs::Odometry& msg) {
            if (!_has_init) {
                State x;
                x.setZero();
                x.x() = msg.pose.pose.position.x;
                x.y() = msg.pose.pose.position.y;
                x.z() = msg.pose.pose.position.z;
                Eigen::Quaterniond vio_q;
                vio_q.w() = msg.pose.pose.orientation.w;
                vio_q.x() = msg.pose.pose.orientation.x;
                vio_q.y() = msg.pose.pose.orientation.y;
                vio_q.z() = msg.pose.pose.orientation.z;
                Eigen::Vector3d vio_euler;
                get_euler_from_q(vio_euler, vio_q);
                x.qx() = vio_euler(0);
                x.qy() = vio_euler(1);
                x.qz() = vio_euler(2);
                init_process(x);
                _has_init = true;
            } else {
                VioMeasurement vio_state;
                vio_state.vio_x() = msg.pose.pose.position.x;
                vio_state.vio_y() = msg.pose.pose.position.y;
                vio_state.vio_z() = msg.pose.pose.position.z;
                Eigen::Quaterniond vio_q;
                vio_q.w() = msg.pose.pose.orientation.w;
                vio_q.x() = msg.pose.pose.orientation.x;
                vio_q.y() = msg.pose.pose.orientation.y;
                vio_q.z() = msg.pose.pose.orientation.z;
                Eigen::Vector3d vio_euler;
                get_euler_from_q(vio_euler, vio_q);
                vio_state.vio_qx() = vio_euler(0);
                vio_state.vio_qy() = vio_euler(1);
                vio_state.vio_qz() = vio_euler(2);

                vio_state.vio_vx() = msg.twist.twist.linear.x;
                vio_state.vio_vy() = msg.twist.twist.linear.y;
                vio_state.vio_vz() = msg.twist.twist.linear.z;
                // vio_state.vio_wx() = msg.twist.twist.angular.x
                // vio_state.vio_wy() = msg.twist.twist.angular.y;
                // vio_state.vio_wz() = msg.twist.twist.angular.z;

                update_process(vio_state);
            }
        }

        void reinit() {
            ROS_WARN("update reinit");
            _has_init = false;
            _predict_has_init = false;
        }

        void init_process(State& init_state) {
            _ukf_ptr->init(init_state);
        }

        void update_process(VioMeasurement& vio_state) {
            _x_ukf = _ukf_ptr->update(_vm, vio_state);
        }

        void predict_process(Control& imu_state) {
            _x_ukf = _ukf_ptr->predict(_sys, imu_state);
        }

        bool get_state( State & now_state) {
            now_state = _x_ukf;
            return _predict_has_init&_has_init;
        }

        bool get_update_init_state() {
            return _has_init;
        }

        void set_predict_valid(bool temp) {
            _predict_has_init = temp;
        }


    private:
        ros::NodeHandle nh_;
        Kalman::UnscentedKalmanFilter<State> * _ukf_ptr;
        bool _has_init;
        bool _predict_has_init;
        ros::Subscriber Vio_measure_sub;
        State _x_ukf;
        SystemModel _sys;
        VioModel _vm;

};

#endif