#ifndef UPDATA_PART_HPP_
#define UPDATA_PART_HPP_
#include "ros/ros.h"
// #include "ukf_test/SystemModel.hpp"
// #include "ukf_test/VioMeasurementModel.hpp"
#include "geometry_math_type.h"

#include "nav_msgs/Odometry.h"

#include <kalman/UnscentedKalmanFilter.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <pthread.h>

#ifdef LOG_FLAG
#include <string>
#include <iostream>
#include <fstream>
#endif

template<typename T, class State, class Control, class SystemModel, class VioMeasurement, class VioModel>
class Filter_update_part {
    public:
 
        Filter_update_part():
        nh_("~update_part") {
            _has_init = false;
            _predict_has_init = false;
            float alpha, beta, kappa;
            std::string vio_topic_name;
            std::string _postopic, _veltopic, _acctopic, _atttopic;
            pthread_mutex_init(&_ukf_core_mutex, NULL);

            nh_.param<std::string>("vio_topic", vio_topic_name, "/vio_odom" );
            nh_.param<float>("alpha", alpha, 1.0f);
            nh_.param<float>("beta", beta, 2.0f);
            nh_.param<float>("kappa", kappa, 0.0f);
            nh_.param<bool>("verbose", _verbose, false);
            nh_.param<std::string>("postopic", _postopic, "/vio_data_rigid1/pos");
            nh_.param<std::string>("veltopic", _veltopic, "/vio_data_rigid1/vel");
            nh_.param<std::string>("acctopic", _acctopic, "/vio_data_rigid1/acc");
            nh_.param<std::string>("atttopic", _atttopic, "/vio_data_rigid1/att");

            T _alpha = T(alpha);
            T _beta = T(beta);
            T _kappa = T(kappa);
            std::cout << "********* param list **********" << std::endl;
            std::cout << "alpha: " << _alpha << std::endl;
            std::cout << "beta: " << _beta << std::endl;
            std::cout << "kappa: " << _kappa << std::endl;
            _ukf_ptr = new Kalman::UnscentedKalmanFilter<State>(_alpha, _beta, _kappa);
            
            Vio_measure_sub = nh_.subscribe(vio_topic_name, 1, &Filter_update_part::vio_update_cb, this);
            
            _rigid_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>(_postopic, 2);
            _rigid_vel_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_veltopic, 2);
            _rigid_acc_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_acctopic, 2);
            _rigid_att_pub = nh_.advertise<geometry_msgs::PoseStamped>(_atttopic, 2);
            
    #ifdef LOG_FLAG
            start_logger();
    #endif

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
                x.qx() = vio_euler(0)/T(M_PI)*T(180);
                x.qy() = vio_euler(1)/T(M_PI)*T(180);
                x.qz() = vio_euler(2)/T(M_PI)*T(180);
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
                vio_state.vio_qx() = vio_euler(0)/T(M_PI)*T(180);
                vio_state.vio_qy() = vio_euler(1)/T(M_PI)*T(180);
                vio_state.vio_qz() = vio_euler(2)/T(M_PI)*T(180);

                vio_state.vio_vx() = msg.twist.twist.linear.x;
                vio_state.vio_vy() = msg.twist.twist.linear.y;
                vio_state.vio_vz() = msg.twist.twist.linear.z;
                // vio_state.vio_wx() = msg.twist.twist.angular.x
                // vio_state.vio_wy() = msg.twist.twist.angular.y;
                // vio_state.vio_wz() = msg.twist.twist.angular.z;

                update_process(vio_state, msg.header.stamp);
            }
        }

        void reinit() {
            ROS_WARN("update reinit");
            _has_init = false;
            _predict_has_init = false;
        }

        void init_process(State& init_state) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _ukf_ptr->init(init_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
        }

        void update_process(VioMeasurement& vio_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->update(_vm, vio_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            if (_verbose) {
                std::cout << "x(vio): [" << vio_state << "]" << std::endl;
                std::cout << "x(ukf): [" << _x_ukf << "]" << std::endl;
            }
    #ifdef LOG_FLAG
            // record_predict(_x_ukf, _time_stamp);
            record_update(vio_state, _time_stamp);
    #endif
        }

        void predict_process(Control& imu_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->predict(_sys, imu_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            ros_publish_state(_x_ukf, _time_stamp);
    #ifdef LOG_FLAG
            record_predict(_x_ukf, _time_stamp);
    #endif
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

        void ros_publish_state( State & now_state, ros::Time & _timestamp) {
            geometry_msgs::PoseStamped _pos_msg;
            geometry_msgs::Vector3Stamped _vel_msg;
            geometry_msgs::Vector3Stamped _acc_msg;
            geometry_msgs::PoseStamped _att_msg;

            _pos_msg.header.stamp = _timestamp;
            _pos_msg.pose.position.x = now_state.x();
            _pos_msg.pose.position.y = now_state.y();
            _pos_msg.pose.position.z = now_state.z();

            _vel_msg.header.stamp = _timestamp;
            _vel_msg.vector.x = now_state.vx();
            _vel_msg.vector.y = now_state.vy();
            _vel_msg.vector.z = now_state.vz();

            _acc_msg.header.stamp = _timestamp;
            _acc_msg.vector.x = now_state.ax();
            _acc_msg.vector.y = now_state.ay();
            _acc_msg.vector.z = now_state.az();

            Eigen::Vector3d _eular_att;
            _eular_att(0) = now_state.qx()*T(M_PI)/T(180);
            _eular_att(1) = now_state.qy()*T(M_PI)/T(180);
            _eular_att(2) = now_state.qz()*T(M_PI)/T(180);
            Eigen::Quaterniond _quat_att;
            get_q_from_euler(_quat_att, _eular_att);
            _att_msg.header.stamp = _timestamp;
            _att_msg.pose.orientation.w = _quat_att.w();
            _att_msg.pose.orientation.x = _quat_att.x();
            _att_msg.pose.orientation.y = _quat_att.y();
            _att_msg.pose.orientation.z = _quat_att.z();

            _rigid_pos_pub.publish(_pos_msg);
            _rigid_vel_pub.publish(_vel_msg);
            _rigid_acc_pub.publish(_acc_msg);
            _rigid_att_pub.publish(_att_msg);
        }

    #ifdef LOG_FLAG
        void start_logger() {
            predict_logger.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/real_predict.csv");
            if (!predict_logger.is_open()) {
                ROS_WARN("cannot open the logger");
            } else {
                predict_logger << "timestamp" << ",";
                predict_logger << "x" << ",";
                predict_logger << "y" << ",";
                predict_logger << "z" << ",";
                predict_logger << "vx" << ",";
                predict_logger << "vy" << ",";
                predict_logger << "vz" << ",";
                predict_logger << "ax" << ",";
                predict_logger << "ay" << ",";
                predict_logger << "az" << ",";
                predict_logger << "phi" << ",";
                predict_logger << "theta" << ",";
                // predict_logger << "psi" << ",";
                predict_logger << "psi" << std::endl;
                // predict_logger << "bias_ax" << ",";
                // predict_logger << "bias_ay" << ",";
                // predict_logger << "bias_az" << ",";
                // predict_logger << "bias_wx" << ",";
                // predict_logger << "bias_wy" << ",";
                // predict_logger << "bias_wz" << std::endl;
            }

            update_logger.open("/home/lhc/work/estimator_ws/src/ukf_test/logger/real_update.csv");
            if (!update_logger.is_open()) {
                ROS_WARN("cannot open the logger");
            } else {
                update_logger << "timestamp" << ",";
                update_logger << "x" << ",";
                update_logger << "y" << ",";
                update_logger << "z" << ",";
                update_logger << "vx" << ",";
                update_logger << "vy" << ",";
                update_logger << "vz" << ",";
                update_logger << "phi" << ",";
                update_logger << "theta" << ",";
                update_logger << "psi" << std::endl;
            }
        }

        void record_predict(State& _p_d, ros::Time & _time_stamp) {
            if (predict_logger.is_open()) {
                predict_logger << _time_stamp << ",";
                predict_logger << _p_d.x() << ",";
                predict_logger << _p_d.y() << ",";
                predict_logger << _p_d.z() << ",";
                predict_logger << _p_d.vx() << ",";
                predict_logger << _p_d.vy() << ",";
                predict_logger << _p_d.vz() << ",";
                predict_logger << _p_d.ax() << ",";
                predict_logger << _p_d.ay() << ",";
                predict_logger << _p_d.az() << ",";
                predict_logger << _p_d.qx() << ",";
                predict_logger << _p_d.qy() << ",";
                // predict_logger << _p_d.qz() << ",";
                predict_logger << _p_d.qz() << std::endl;
                // predict_logger << _p_d.bax() << ",";
                // predict_logger << _p_d.bay() << ",";
                // predict_logger << _p_d.baz() << ",";
                // predict_logger << _p_d.bwx() << ",";
                // predict_logger << _p_d.bwy() << ",";
                // predict_logger << _p_d.bwz() << std::endl;
            }
        }

        void record_update(VioMeasurement& _m_d, ros::Time & _time_stamp) {
            if (update_logger.is_open()) {
                update_logger << _time_stamp << ",";
                update_logger << _m_d.vio_x() << ",";
                update_logger << _m_d.vio_y() << ",";
                update_logger << _m_d.vio_z() << ",";
                update_logger << _m_d.vio_vx() << ",";
                update_logger << _m_d.vio_vy() << ",";
                update_logger << _m_d.vio_vz() << ",";
                update_logger << _m_d.vio_qx() << ",";
                update_logger << _m_d.vio_qy() << ",";
                update_logger << _m_d.vio_qz() << std::endl;
            }
        }
    #endif


    private:
        ros::NodeHandle nh_;
        Kalman::UnscentedKalmanFilter<State> * _ukf_ptr;
        bool _has_init;
        bool _predict_has_init;
        ros::Subscriber Vio_measure_sub;
        ros::Publisher _rigid_pos_pub;
        ros::Publisher _rigid_vel_pub;
        ros::Publisher _rigid_acc_pub;
        ros::Publisher _rigid_att_pub;
        State _x_ukf;
        SystemModel _sys;
        VioModel _vm;
        bool _verbose;
        pthread_mutex_t _ukf_core_mutex;
    #ifdef LOG_FLAG
        std::ofstream predict_logger;
        std::ofstream update_logger;
    #endif

};

#endif