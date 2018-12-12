#ifndef UPDATA_PART_HPP_
#define UPDATA_PART_HPP_
#include "ros/ros.h"
// #include "ukf_test/SystemModel.hpp"
// #include "ukf_test/VioMeasurementModel.hpp"
#include "geometry_math_type.h"
#include "gps.hpp"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"

#include <kalman/UnscentedKalmanFilter.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <pthread.h>

#ifdef LOG_FLAG
#include <string>
#include <iostream>
#include <fstream>
#endif

template<typename T, class State, class Control, class SystemModel, class OdomMeasurement, class OdomMeasurementModel, class LidarMeasurement, 
class LidarMeasurementModel, class GpsMeasurement, class GpsMeasurementModel, class VioMeasurement, class VioModel>
class Filter_update_part {
    public:
 
        Filter_update_part():
        nh_("~update_part") {
            _has_init = false;
            _predict_has_init = false;
            _gps_has_init = false;
            _update_has_ready = false;
            // _lidar_update_has_ready = false;
            // _gps_update_has_ready = false;
            // _odom_update_has_ready = false;
            lidar_to_imu << 0, 0.000123, -0.1374;
            odom_to_imu << 0.2, -0.245123, 0.835524;
            gps_to_imu << -0.319632, -0.878521, -0.2035;
            o_to_imu << 0, -0.825123, 0.942;

            float alpha, beta, kappa;
            std::string odom_topic_name;
            std::string lidar_topic_name;
            std::string gps_topic_name;
            std::string _postopic, _veltopic, _acctopic, _atttopic, wgs_topic;
            pthread_mutex_init(&_ukf_core_mutex, NULL);

            nh_.param<std::string>("odom_topic", odom_topic_name, "/odom" );
            nh_.param<std::string>("lidar_topic", lidar_topic_name, "/lidar_odom" );
            nh_.param<std::string>("gps_topic", gps_topic_name, "/gps" );
            nh_.param<float>("alpha", alpha, 1.0f);
            nh_.param<float>("beta", beta, 2.0f);
            nh_.param<float>("kappa", kappa, 0.0f);
            nh_.param<bool>("verbose", _verbose, false);
            nh_.param<std::string>("postopic", _postopic, "/vio_data_rigid1/pos");
            nh_.param<std::string>("veltopic", _veltopic, "/vio_data_rigid1/vel");
            nh_.param<std::string>("acctopic", _acctopic, "/vio_data_rigid1/acc");
            nh_.param<std::string>("atttopic", _atttopic, "/vio_data_rigid1/att");
            nh_.param<std::string>("wgstopic", wgs_topic, "/wgs");
            
            T _alpha = T(alpha);
            T _beta = T(beta);
            T _kappa = T(kappa);
            std::cout << "********* param list **********" << std::endl;
            std::cout << "alpha: " << _alpha << std::endl;
            std::cout << "beta: " << _beta << std::endl;
            std::cout << "kappa: " << _kappa << std::endl;
            _ukf_ptr = new Kalman::UnscentedKalmanFilter<State>(_alpha, _beta, _kappa);
            
            Odom_measure_sub = nh_.subscribe(odom_topic_name, 1, &Filter_update_part::odom_update_cb, this);
            Gps_measure_sub = nh_.subscribe(gps_topic_name, 1, &Filter_update_part::gps_update_cb, this);
            Lidar_odom_sub = nh_.subscribe(lidar_topic_name, 1, &Filter_update_part::lidar_update_cb, this);
            
            _rigid_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>(_postopic, 2);
            _rigid_vel_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_veltopic, 2);
            _rigid_acc_pub = nh_.advertise<geometry_msgs::Vector3Stamped>(_acctopic, 2);
            _rigid_att_pub = nh_.advertise<geometry_msgs::PoseStamped>(_atttopic, 2);
            _WGS_84_pub = nh_.advertise<sensor_msgs::NavSatFix>(wgs_topic, 2);
            _gps_start_pub = nh_.advertise<nav_msgs::Path>("/gps_path", 10);
            
    #ifdef LOG_FLAG
            start_logger();
    #endif

        }

        ~Filter_update_part() {
            delete _ukf_ptr;
        }
        
        void lidar_update_cb(const nav_msgs::Odometry& msg) {

            pthread_mutex_lock(&_ukf_core_mutex);
            State _temp_x = _ukf_ptr->getState();
            pthread_mutex_unlock(&_ukf_core_mutex);

                Eigen::Quaterniond temp_lidar_q;
                temp_lidar_q.w() = msg.pose.pose.orientation.w;
                temp_lidar_q.x() = msg.pose.pose.orientation.x;
                temp_lidar_q.y() = msg.pose.pose.orientation.z;
                temp_lidar_q.z() = - msg.pose.pose.orientation.y;
                Eigen::Vector3d lidar_euler;
                get_euler_from_q(lidar_euler, temp_lidar_q);
                T temp_qx = lidar_euler(0)/T(M_PI)*T(180);
                T temp_qy = lidar_euler(1)/T(M_PI)*T(180);
                T temp_qz = lidar_euler(2)/T(M_PI)*T(180);

            T temp_yaw = _temp_x.qz();
            T temp_yaw1 = constrain_yaw(temp_yaw);
            T _delta = fabsf(temp_qz - temp_yaw1);
            // std::cout << "before: temp_qz:" << temp_qz << std::endl;
            // std::cout << "before: yaw    :" << temp_yaw << std::endl;
            // std::cout << "delta          :" << _delta << std::endl;
            if (_delta <= T(180)) {
                if (temp_qz >= temp_yaw1) {
                    temp_qz = temp_yaw + _delta;
                } else {
                    temp_qz = temp_yaw - _delta;
                }
            } else {
                _delta = T(360) - _delta;
                if (temp_qz < temp_yaw1) {
                    temp_qz = temp_yaw + _delta;
                } else {
                    temp_qz = temp_yaw - _delta;
                }
            }
            // std::cout << "after: temp_qz:" << temp_qz << std::endl;

            if (!_has_init ) {
                State x;
                x.setZero();
                x.x() = msg.pose.pose.position.x + lidar_to_imu[0];
                x.y() = msg.pose.pose.position.z + lidar_to_imu[1];
                x.z() = - msg.pose.pose.position.y + lidar_to_imu[2];
                // Eigen::Quaterniond lidar_q;
                // lidar_q.w() = msg.pose.pose.orientation.w;
                // lidar_q.x() = msg.pose.pose.orientation.x;
                // lidar_q.y() = msg.pose.pose.orientation.z;
                // lidar_q.z() = - msg.pose.pose.orientation.y;
                // Eigen::Vector3d lidar_euler;
                // get_euler_from_q(lidar_euler, lidar_q);
                x.qx() = temp_qx;
                x.qy() = temp_qy;
                x.qz() = temp_qz;
                init_process(x);
                _has_init = true;
            } else {
                LidarMeasurement lidar_state;
                lidar_state.lidar_x() = msg.pose.pose.position.x + lidar_to_imu[0];
                lidar_state.lidar_y() = msg.pose.pose.position.z + lidar_to_imu[1];
                lidar_state.lidar_z() = - msg.pose.pose.position.y + lidar_to_imu[2];
                Start_in_earth[0] = lidar_state.lidar_x();
                Start_in_earth[1] = lidar_state.lidar_y();
                Start_in_earth[2] = lidar_state.lidar_z();
                // Eigen::Quaterniond lidar_q;
                // lidar_q.w() = msg.pose.pose.orientation.w;
                // lidar_q.x() = msg.pose.pose.orientation.x;
                // lidar_q.y() = msg.pose.pose.orientation.z;
                // lidar_q.z() = - msg.pose.pose.orientation.y;
                // Eigen::Vector3d lidar_euler;
                // get_euler_from_q(lidar_euler, lidar_q);
                lidar_state.lidar_qx() = temp_qx;
                lidar_state.lidar_qy() = temp_qy;
                lidar_state.lidar_qz() = temp_qz;
                if (!_predict_has_init) {
                    State x;
                    x.setZero();
                    x.x() = msg.pose.pose.position.x + lidar_to_imu[0];
                    x.y() = msg.pose.pose.position.z + lidar_to_imu[1];
                    x.z() = - msg.pose.pose.position.y + lidar_to_imu[2];
                    Start_in_earth[0] = lidar_state.lidar_x();
                    Start_in_earth[1] = lidar_state.lidar_y();
                    Start_in_earth[2] = lidar_state.lidar_z();
                    Eigen::Quaterniond lidar_q;
                    lidar_q.w() = msg.pose.pose.orientation.w;
                    lidar_q.x() = msg.pose.pose.orientation.x;
                    lidar_q.y() = msg.pose.pose.orientation.z;
                    lidar_q.z() = - msg.pose.pose.orientation.y;
                    Eigen::Vector3d lidar_euler;
                    get_euler_from_q(lidar_euler, lidar_q);
                    x.qx() = lidar_euler(0)/T(M_PI)*T(180);
                    x.qy() = lidar_euler(1)/T(M_PI)*T(180);
                    x.qz() = lidar_euler(2)/T(M_PI)*T(180);
                    init_process(x);
                } else {
                    if (_update_has_ready) {
                        update_process_lidar(lidar_state, msg.header.stamp);
                        _update_has_ready = false;
                    }
                }
            }
        }

        void gps_update_cb(const sensor_msgs::NavSatFix& msg) {
            if (!_has_init || !_predict_has_init) {
                return;
            } else {
                if (!_gps_has_init) {
                    // Center[0] = msg.latitude;
                    // Center[1] = msg.longitude;
                    // Center[2] = msg.altitude;
                    Eigen::Vector3d Start_wgs;
                    Start_wgs[0] = msg.latitude;
                    Start_wgs[1] = msg.longitude;
                    Start_wgs[2] = 0; //msg.altitude;
                    Center_yaw << 0, 0, msg.position_covariance[0];

                    // Eigen::Vector3d Start_to_Origin;
                    // Start_to_Origin = - Start_in_earth;
                    // // Eigen::Vector3d Center_XYZ;
                    // // xyz_to_XYZ(Start_to_Origin, Start_wgs, Center_XYZ);
                    // // XYZ_to_WGS(Center_XYZ, Center);

                    // xyz_to_WGS(Start_to_Origin, Start_wgs, Center, Center_yaw);
                    // std::cout << "Start_wgs:" << Start_wgs.transpose() << std::endl;
                    // std::cout << "Origin_wgs:" << Center.transpose() << std::endl;

                    Center = Start_wgs;

                    _gps_has_init = true;
                } else {
                    Eigen::Vector3d gps_data;
                    gps_data[0] = msg.latitude;
                    gps_data[1] = msg.longitude;
                    gps_data[2] = msg.altitude;

                    // Eigen::Vector3d gps_XYZ;
                    // WGS_to_XYZ(gps_data, gps_XYZ);
                    Eigen::Vector3d gps_xyz;
                    // XYZ_to_xyz(gps_XYZ, Center, gps_xyz);
                    WGS_to_xyz(gps_data, Center, gps_xyz, Center_yaw);
                    std::cout << "gps_xyz:" << gps_xyz.transpose() << std::endl;

                    gps_path.header.stamp = msg.header.stamp;
                    gps_path.header.frame_id = "camera_init";
                    geometry_msgs::PoseStamped _gps_pos;
                    _gps_pos.pose.position.x = gps_xyz[0];
                    _gps_pos.pose.position.y = -gps_xyz[2];
                    _gps_pos.pose.position.z = gps_xyz[1];
                    gps_path.poses.push_back(_gps_pos);
                    _gps_start_pub.publish(gps_path);

                    std::cout << "center_yaw:" << Center_yaw << std::endl;
                    GpsMeasurement gps_state;
                    gps_state.gps_x() = gps_xyz[0] + gps_to_imu[0];
                    gps_state.gps_y() = gps_xyz[1] + gps_to_imu[1];
                    // gps_state.gps_z() = gps_xyz[2] + gps_to_imu[2];
                    // Eigen::Vector3d gps_test;
                    // gps_test[0] = gps_xyz[0] + gps_to_imu[0];
                    // gps_test[1] = gps_xyz[1] + gps_to_imu[1];
                    // gps_test[2] = gps_xyz[2] + gps_to_imu[2];
                    // std::cout << "gps_test:" << gps_test.transpose() << std::endl;
                    
                    gps_state.gps_yaw() = - msg.position_covariance[0] + Center_yaw[2];
                    if (_update_has_ready) {
                        //update_process_gps(gps_state, msg.header.stamp);
                        _update_has_ready = false;
                    }
                }
            }
       }

        void odom_update_cb(const nav_msgs::Odometry& msg) {
            if (!_has_init || !_predict_has_init) {
                return;
            } else {
                OdomMeasurement odom_state;
                odom_state.odom_vx() = 0;
                odom_state.odom_vy() = msg.twist.twist.linear.x;
                odom_state.odom_vz() = 0;
                if (_update_has_ready) {
                    update_process_odom(odom_state, msg.header.stamp);
                    _update_has_ready = false;
                }
            }
        }

        void reinit() {
            ROS_WARN("update reinit");
            _has_init = false;
            _predict_has_init = false;
            _gps_has_init = false;
        }

        void init_process(State& init_state) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _ukf_ptr->init(init_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
        }

        void update_process_lidar(LidarMeasurement& lidar_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->update(_lidarm, lidar_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            if (_verbose) {
                std::cout << "x(lidar): [" << lidar_state << "]" << std::endl;
                std::cout << "x(ukf): [" << _x_ukf << "]" << std::endl;
            }
    #ifdef LOG_FLAG
            // record_predict(_x_ukf, _time_stamp);
            record_update_lidar(lidar_state, _time_stamp);
    #endif
        }

        void update_process_gps(GpsMeasurement& gps_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->update(_gpsm, gps_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            if (_verbose) {
                std::cout << "x(gps): [" << gps_state << "]" << std::endl;
                std::cout << "x(ukf): [" << _x_ukf << "]" << std::endl;
            }
    #ifdef LOG_FLAG
            // record_predict(_x_ukf, _time_stamp);
            record_update_gps(gps_state, _time_stamp);
    #endif
        }

        void update_process_odom(OdomMeasurement& odom_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->update(_om, odom_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            if (_verbose) {
                std::cout << "x(odom): [" << odom_state << "]" << std::endl;
                std::cout << "x(ukf): [" << _x_ukf << "]" << std::endl;
            }
    #ifdef LOG_FLAG
            // record_predict(_x_ukf, _time_stamp);
            record_update_odom(odom_state, _time_stamp);
    #endif
        }

        void predict_process(Control& imu_state, ros::Time _time_stamp) {
            pthread_mutex_lock(&_ukf_core_mutex);
            _x_ukf = _ukf_ptr->predict(_sys, imu_state);
            pthread_mutex_unlock(&_ukf_core_mutex);
            ros_publish_state(_x_ukf, _time_stamp); 
            _update_has_ready = true;
            // _lidar_update_has_ready = true;
            // _gps_update_has_ready = true;
            // _odom_update_has_ready = true;
    #ifdef LOG_FLAG
            record_predict(_x_ukf, _time_stamp);
            record_WGS(_x_ukf, _time_stamp);
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
            sensor_msgs::NavSatFix WGS_84;

            // Eigen::Vector3d pos_now, pos_ned, euler_to_ned;
            // Eigen::Matrix3d dcm_to_ned;
            // pos_now << now_state.x(), now_state.y(), now_state.z();
            // euler_to_ned << 0, 0, Center_yaw;
            // get_dcm_from_euler(dcm_to_ned, euler_to_ned);
            // pos_ned = dcm_to_ned * pos_now;

            _pos_msg.header.stamp = _timestamp;
            _pos_msg.pose.position.x = now_state.x();
            _pos_msg.pose.position.y = now_state.y();
            _pos_msg.pose.position.z = now_state.z();
            // _pos_msg.pose.position.x = pos_ned[0];
            // _pos_msg.pose.position.y = pos_ned[1];
            // _pos_msg.pose.position.z = pos_ned[2];


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

            Eigen::Vector3d pos_xyz;
            pos_xyz[0] = now_state.x();
            pos_xyz[1] = now_state.y();
            pos_xyz[2] = now_state.z();
            T pos_yaw;
            pos_yaw = - now_state.qz() + Center_yaw[2]; 
            Eigen::Vector3d pos_XYZ, pos_wgs; 
            // xyz_to_XYZ(pos_xyz, Center, pos_XYZ);
            // XYZ_to_WGS(pos_XYZ, pos_wgs);
            xyz_to_WGS(pos_xyz, Center, pos_wgs, Center_yaw);
            WGS_84.header.stamp = _timestamp;
            WGS_84.latitude = pos_wgs[0];
            WGS_84.longitude = pos_wgs[1];
            // WGS_84.altitude = pos_wgs[2];
            WGS_84.position_covariance[0] = pos_yaw;

            _rigid_pos_pub.publish(_pos_msg);
            _rigid_vel_pub.publish(_vel_msg);
            _rigid_acc_pub.publish(_acc_msg);
            _rigid_att_pub.publish(_att_msg);
            _WGS_84_pub.publish(WGS_84);
        }

    #ifdef LOG_FLAG
        void start_logger() {
            predict_logger.open("/home/htr/ukf_ws/src/ukf_test/logger/real_predict.csv");
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

            update_logger.open("/home/htr/ukf_ws/src/ukf_test/logger/real_update.csv");
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

            wgs_logger.open("/home/htr/ukf_ws/src/ukf_test/logger/WGS.csv");
            if (!wgs_logger.is_open()) {
                ROS_WARN("cannot open the logger");
            } else {
                wgs_logger << "UTC Timestamp" << ","; 
                wgs_logger << "Latitude" << ",";
                wgs_logger << "Longitude" << ",";
                wgs_logger << "Heading" << std::endl;
            }


        }

        void record_WGS(State& _p_d, ros::Time& _time_stamp) {
            if (_gps_has_init) {
                Eigen::Vector3d final_wgs;
                Eigen::Vector3d _pos;
                _pos[0] = _p_d.x();
                _pos[1] = _p_d.y();
                _pos[2] = 0;
                xyz_to_WGS(_pos, Center, final_wgs, Center_yaw);
                T pos_yaw, pos_yaw_;
                pos_yaw = - _p_d.qz() + Center_yaw[2]; 
                pos_yaw_ = constrain_yaw(pos_yaw);

                if (wgs_logger.is_open()) {
                    wgs_logger << _time_stamp << ",";
                    wgs_logger << std::setprecision(15) << final_wgs[0] << ",";
                    wgs_logger << std::setprecision(15) << final_wgs[1] << ",";
                    wgs_logger << std::setprecision(15) << pos_yaw_ << std::endl;
                }
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

        void record_update_lidar(LidarMeasurement& _m_d, ros::Time & _time_stamp) {
            if (update_logger.is_open()) {
                float zero = 0.0;
                update_logger << _time_stamp << ",";
                update_logger << _m_d.lidar_x() << ",";
                update_logger << _m_d.lidar_y() << ",";
                update_logger << _m_d.lidar_z() << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << _m_d.lidar_qx() << ",";
                update_logger << _m_d.lidar_qy() << ",";
                update_logger << _m_d.lidar_qz() << std::endl;
            }
        }

       void record_update_gps(GpsMeasurement& _m_d, ros::Time & _time_stamp) {
            if (update_logger.is_open()) {
                float zero = 0.0;
                update_logger << _time_stamp << ",";
                update_logger << _m_d.gps_x() << ",";
                update_logger << _m_d.gps_y() << ",";
                update_logger << _m_d.gps_z() << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << _m_d.gps_yaw() << std::endl;
            }
        }

        void record_update_odom(OdomMeasurement& _m_d, ros::Time & _time_stamp) {
            if (update_logger.is_open()) {
                float zero = 0.0;
                update_logger << _time_stamp << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << _m_d.odom_vx() << ",";
                update_logger << _m_d.odom_vy() << ",";
                update_logger << _m_d.odom_vz() << ",";
                update_logger << zero << ",";
                update_logger << zero << ",";
                update_logger << zero << std::endl;
            }
        }
    #endif


    private:
        ros::NodeHandle nh_;
        Kalman::UnscentedKalmanFilter<State> * _ukf_ptr;
        bool _has_init;
        bool _predict_has_init;
        bool _gps_has_init;
        bool _update_has_ready;
        // bool _lidar_update_has_ready;
        // bool _gps_update_has_ready;
        // bool _odom_update_has_ready;
        ros::Subscriber Odom_measure_sub;
        ros::Subscriber Lidar_odom_sub;
        ros::Subscriber Gps_measure_sub;
        ros::Publisher _rigid_pos_pub;
        ros::Publisher _rigid_vel_pub;
        ros::Publisher _rigid_acc_pub;
        ros::Publisher _rigid_att_pub;
        ros::Publisher _WGS_84_pub;
        ros::Publisher _gps_start_pub;
        nav_msgs::Path gps_path;
        Eigen::Vector3d Start_in_earth;
        Eigen::Vector3d Center;
        Eigen::Vector3d Center_yaw;
        State _x_ukf;
        SystemModel _sys;
        VioModel _vm;
        OdomMeasurementModel _om;
        GpsMeasurementModel _gpsm;
        LidarMeasurementModel _lidarm;
        bool _verbose;
        Eigen::Vector3d lidar_to_imu;
        Eigen::Vector3d odom_to_imu;
        Eigen::Vector3d gps_to_imu;
        Eigen::Vector3d o_to_imu;
        pthread_mutex_t _ukf_core_mutex;
    #ifdef LOG_FLAG
        std::ofstream predict_logger;
        std::ofstream update_logger;
        std::ofstream wgs_logger;
    #endif

};

#endif