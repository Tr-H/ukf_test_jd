#ifndef VIOMEASUREMENTMODE_HPP_
#define VIOMEASUREMENTMODE_HPP_

#include <kalman/MeasurementModel.hpp>
#include "geometry_math_type.h"

namespace Test1 {

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

        T vio_x()       const { return (*this)[ VIO_X ]; }
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

        T& vio_x()       { return (*this)[ VIO_X ]; }
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