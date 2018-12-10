#ifndef VIOMEASUREMENTMODE_HPP_
#define VIOMEASUREMENTMODE_HPP_

#include <kalman/MeasurementModel.hpp>
#include "geometry_math_type.h"

namespace Test1 {

template<typename T>
class GpsMeasurement : public Kalman::Vector<T, 4> {
    public:
        KALMAN_VECTOR(GpsMeasurement, T, 4)

        static constexpr size_t GPS_X = 0;
        static constexpr size_t GPS_Y = 1;
        static constexpr size_t GPS_Z = 2;
        static constexpr size_t GPS_YAW = 3;

        T gps_x()     const { return (*this)[ GPS_X ]; }  
        T gps_y()     const { return (*this)[ GPS_Y ]; }  
        T gps_z()     const { return (*this)[ GPS_Z ]; }  
        T gps_yaw()     const { return (*this)[ GPS_YAW ]; }  

        T& gps_x()     { return (*this)[ GPS_X ]; }
        T& gps_y()     { return (*this)[ GPS_Y ]; }
        T& gps_z()     { return (*this)[ GPS_Z ]; }
        T& gps_yaw()     { return (*this)[ GPS_YAW ]; }

};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class GpsMeasurementModel : public Kalman::MeasurementModel<State<T>, GpsMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::GpsMeasurement<T> M;
        GpsMeasurementModel() {}

        M h(const S& x) const {
            M measurement;
            // Eigen::Matrix3d odom_R;
            // Eigen::Vector3d e;
            // e(0) = x.qx();
            // e(1) = x.qy();
            // e(2) = x.qz();
            // get_dcm_from_euler(odom_R, e); // imu to earth
            // // measurement.odom_vx() = odom_R(0, 0) * x.vx() + odom_R(1, 0) * x.vy() + odom_R(2, 0) * x.vz();
            // // measurement.odom_vy() = odom_R(0, 1) * x.vx() + odom_R(1, 1) * x.vy() + odom_R(2, 1) * x.vz();
            // // measurement.odom_vz() = odom_R(0, 2) * x.vx() + odom_R(1, 2) * x.vy() + odom_R(2, 2) * x.vz();
            measurement.gps_x() = x.x();
            measurement.gps_y() = x.y();
            measurement.gps_z() = x.z();
            measurement.gps_yaw() = x.qz();

            return measurement;
        }
};

template<typename T>
class OdomMeasurement : public Kalman::Vector<T, 3> {
    public:
        KALMAN_VECTOR(OdomMeasurement, T, 3)

        static constexpr size_t ODOM_VX = 0;
        static constexpr size_t ODOM_VY = 1;
        static constexpr size_t ODOM_VZ = 2;

        T odom_vx()     const { return (*this)[ ODOM_VX ]; }  
        T odom_vy()     const { return (*this)[ ODOM_VY ]; }  
        T odom_vz()     const { return (*this)[ ODOM_VZ ]; }  

        T& odom_vx()     { return (*this)[ ODOM_VX]; }
        T& odom_vy()     { return (*this)[ ODOM_VY]; }
        T& odom_vz()     { return (*this)[ ODOM_VZ]; }

};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OdomMeasurementModel : public Kalman::MeasurementModel<State<T>, OdomMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::OdomMeasurement<T> M;
        OdomMeasurementModel() {}

        M h(const S& x) const {
            M measurement;
            Eigen::Matrix3d odom_R;
            Eigen::Vector3d e;
            e(0) = x.qx();
            e(1) = x.qy();
            e(2) = x.qz();
            get_dcm_from_euler(odom_R, e); // imu to earth
            // measurement.odom_vx() = odom_R(0, 0) * x.vx() + odom_R(1, 0) * x.vy() + odom_R(2, 0) * x.vz();
            // measurement.odom_vy() = odom_R(0, 1) * x.vx() + odom_R(1, 1) * x.vy() + odom_R(2, 1) * x.vz();
            // measurement.odom_vz() = odom_R(0, 2) * x.vx() + odom_R(1, 2) * x.vy() + odom_R(2, 2) * x.vz();
            measurement.odom_vx() = 0;
            measurement.odom_vy() = odom_R(0, 1) * x.vx() + odom_R(1, 1) * x.vy() + odom_R(2, 1) * x.vz();
            measurement.odom_vz() = 0;

            return measurement;
        }
};

template<typename T>
class LidarMeasurement : public Kalman::Vector<T, 6> {
    public:
        KALMAN_VECTOR(LidarMeasurement, T, 6)

        static constexpr size_t LIDAR_X = 0;
        static constexpr size_t LIDAR_Y = 1;
        static constexpr size_t LIDAR_Z = 2;
        // static constexpr size_t VIO_QW = 3;
        static constexpr size_t LIDAR_QX = 3;
        static constexpr size_t LIDAR_QY = 4;
        static constexpr size_t LIDAR_QZ = 5;

        T lidar_x()     const { return (*this)[ LIDAR_X]; }  
        T lidar_y()     const { return (*this)[ LIDAR_Y]; }  
        T lidar_z()     const { return (*this)[ LIDAR_Z]; }  
        // T lidar_qw()     const { return (*this)[ LIDAR_QW]; }  
        T lidar_qx()     const { return (*this)[ LIDAR_QX]; }  
        T lidar_qy()     const { return (*this)[ LIDAR_QY]; }  
        T lidar_qz()     const { return (*this)[ LIDAR_QZ]; }  

        T& lidar_x()     { return (*this)[ LIDAR_X]; }
        T& lidar_y()     { return (*this)[ LIDAR_Y]; }
        T& lidar_z()     { return (*this)[ LIDAR_Z]; }
        // T& lidar_qw()     { return (*this)[ LIDAR_QW]; }
        T& lidar_qx()     { return (*this)[ LIDAR_X]; }
        T& lidar_qy()     { return (*this)[ LIDAR_Y]; }
        T& lidar_qz()     { return (*this)[ LIDAR_Z]; }

};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class LidarMeasurementModel : public Kalman::MeasurementModel<State<T>, LidarMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::LidarMeasurement<T> M;
        LidarMeasurementModel() {}

        M h(const S& x) const {
            M measurement;
            measurement.lidar_x() = x.x();
            measurement.lidar_y() = x.y();
            measurement.lidar_z() = x.z();
            // Eigen::Quaterniond lidar_q;
            //Eigen::Vector3d e;
            // e(0) = x.qx();
            // e(1) = x.qy();
            // e(2) = x.qz();
            // e(0) = x.qx()/T(180)*T(M_PI);
            // e(1) = x.qy()/T(180)*T(M_PI);
            // e(2) = x.qz()/T(180)*T(M_PI);
            // get_q_from_euler(vio_q, e);
            // get_q_from
            // measurement.vio_qw() = 1;
            // measurement.vio_qx() = vio_q.x();
            // measurement.vio_qy() = vio_q.y();
            // measurement.vio_qz() = vio_q.z();
            measurement.lidar_qx() = x.qx();
            measurement.lidar_qy() = x.qy();
            measurement.lidar_qz() = x.qz();

            return measurement;
        }
};

template<typename T>
class VioMeasurement : public Kalman::Vector<T, 9> {
    public:
        KALMAN_VECTOR(VioMeasurement, T, 9)

        static constexpr size_t VIO_X = 0;
        static constexpr size_t VIO_Y = 1;
        static constexpr size_t VIO_Z = 2;
        // static constexpr size_t VIO_QW = 3;
        static constexpr size_t VIO_QX = 3;
        static constexpr size_t VIO_QY = 4;
        static constexpr size_t VIO_QZ = 5;

        static constexpr size_t VIO_VX = 6;
        static constexpr size_t VIO_VY = 7;
        static constexpr size_t VIO_VZ = 8;

        // static constexpr size_t VIO_WX = 9;
        // static constexpr size_t VIO_WY = 10;
        // static constexpr size_t VIO_WZ = 11;

        T vio_x()       const { return (*this)[ VIO_X ]; } // change value    const to keep the address of *this
        T vio_y()       const { return (*this)[ VIO_Y ]; }
        T vio_z()       const { return (*this)[ VIO_Z ]; }
        // T vio_qw()       const { return (*this)[ VIO_QW ]; }
        T vio_qx()       const { return (*this)[ VIO_QX ]; }
        T vio_qy()       const { return (*this)[ VIO_QY ]; }
        T vio_qz()       const { return (*this)[ VIO_QZ ]; }

        T vio_vx()       const { return (*this)[ VIO_VX ]; }
        T vio_vy()       const { return (*this)[ VIO_VY ]; }
        T vio_vz()       const { return (*this)[ VIO_VZ ]; }
        // T vio_wx()       const { return (*this)[ VIO_WX ]; }
        // T vio_wy()       const { return (*this)[ VIO_WY ]; }
        // T vio_wz()       const { return (*this)[ VIO_WZ ]; }

        T& vio_x()       { return (*this)[ VIO_X ]; } //return address for what? 
        T& vio_y()       { return (*this)[ VIO_Y ]; }
        T& vio_z()       { return (*this)[ VIO_Z ]; }
        // T& vio_qw()       { return (*this)[ VIO_QW ]; }
        T& vio_qx()       { return (*this)[ VIO_QX ]; }
        T& vio_qy()       { return (*this)[ VIO_QY ]; }
        T& vio_qz()       { return (*this)[ VIO_QZ ]; }

        T& vio_vx()       { return (*this)[ VIO_VX ]; }
        T& vio_vy()       { return (*this)[ VIO_VY ]; }
        T& vio_vz()       { return (*this)[ VIO_VZ ]; }
        // T& vio_wx()       { return (*this)[ VIO_WX ]; }
        // T& vio_wy()       { return (*this)[ VIO_WY ]; }
        // T& vio_wz()       { return (*this)[ VIO_WZ ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class VioMeasurementModel : public Kalman::MeasurementModel<State<T>, VioMeasurement<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::VioMeasurement<T> M;
        VioMeasurementModel() {}

        M h(const S& x) const {
            M measurement;
            measurement.vio_x() = x.x();
            measurement.vio_y() = x.y();
            measurement.vio_z() = x.z();
            // measurement.vio_qw() = x.qw();
            Eigen::Quaterniond vio_q;
            //Eigen::Vector3d e;
            // e(0) = x.qx();
            // e(1) = x.qy();
            // e(2) = x.qz();
            // e(0) = x.qx()/T(180)*T(M_PI);
            // e(1) = x.qy()/T(180)*T(M_PI);
            // e(2) = x.qz()/T(180)*T(M_PI);
            // get_q_from_euler(vio_q, e);
            // get_q_from
            // measurement.vio_qw() = 1;
            // measurement.vio_qx() = vio_q.x();
            // measurement.vio_qy() = vio_q.y();
            // measurement.vio_qz() = vio_q.z();
            measurement.vio_qx() = x.qx();
            measurement.vio_qy() = x.qy();
            measurement.vio_qz() = x.qz();

            measurement.vio_vx() = x.vx();
            measurement.vio_vy() = x.vy();
            measurement.vio_vz() = x.vz();
            // measurement.vio_wx() = x.wx();
            // measurement.vio_wy() = x.wy();
            // measurement.vio_wz() = x.wz();
            return measurement;
        }
};

}

#endif