#ifndef GPS_HPP_
#define GPS_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

void WGS_to_XYZ(Eigen::Vector3d &wgs, Eigen::Vector3d &XYZ) {
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752.3141;//b为椭球的短半轴:a=6356.7523141km
    double H = wgs[2]; //+ a;
    // double e=sqrt(1-pow(b ,2)/pow(a ,2)); //e为椭球的第一偏心率
    // double e=sqrt(0.006693421622966); //克拉索夫斯基椭球
    // double e=sqrt(0.006694384999588); //1975年国际椭球
    double e = sqrt(0.0066943799013); //WGS-84椭球
    double m = M_PI/180;//经度维度需要转换成弧度.
    double B = wgs[0] * m;
    double L = wgs[1] * m;
    double W = sqrt(1-pow(e ,2)*pow(sin(B) ,2));
    double N = a/W; //N为椭球的卯酉圈曲率半径
    XYZ[0] = (N+H) * cos(B) * cos(L);
    XYZ[1] = (N+H) * cos(B) * sin(L);
    XYZ[2] = (N * (1-pow(e ,2)) + H) * sin(B);  
}

void XYZ_to_WGS(Eigen::Vector3d &XYZ, Eigen::Vector3d &wgs) {
    double v0 = XYZ[2] / sqrt(pow(XYZ[0], 2) + pow(XYZ[1] ,2));
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752;
    // double e=sqrt(1-pow(b ,2)/pow(a ,2)); //e为椭球的第一偏心率
    // double e=sqrt(0.006693421622966); //克拉索夫斯基椭球
    // double e=sqrt(0.006694384999588); //1975年国际椭球
    double e = sqrt(0.0066943799013); //WGS-84椭球
    // double W=sqrt(1-pow(e ,2)*pow(sin(B) ,2));
    double N = 0 ; //N为椭球的卯酉圈曲率半径
    double B1 = atan(v0), B2 = 0;
    double H = 0;
    while(fabs(B2-B1) > 1E-5) {
    N = a / sqrt(1-pow(e ,2) * pow(sin(B1) ,2));
    H = XYZ[2] / sin(B1) - N * (1-pow(e ,2));
    B2 = atan(XYZ[2] * (N+H) / sqrt((pow(XYZ[0], 2) + pow(XYZ[1], 2)) * (N * (1-pow(e ,2)) + H)));
    B1 = B2;
    }

    double m = M_PI/180;
    wgs[0] = B1 / m;
    wgs[1] = atan(XYZ[1] / XYZ[0]) / m;
    wgs[2] = H - a;  
}

// xyz is ENU
void XYZ_to_xyz(Eigen::Vector3d &XYZ, Eigen::Vector3d &Center, Eigen::Vector3d &xyz) {
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752.3141;//b为椭球的短半轴:a=6356.7523141km
    Eigen::Vector3d tmp_XYZ;
    Eigen::Vector3d Center_XYZ;
    WGS_to_XYZ(Center, Center_XYZ);
    tmp_XYZ = XYZ - Center_XYZ;
    double m=M_PI/180;
    xyz[0] = -sin(Center[0] * m) * cos(Center[1] * m) * tmp_XYZ[0] - sin(Center[0] * m) * sin(Center[1] * m) * tmp_XYZ[1]
             + cos(Center[0] * m) * tmp_XYZ[2];
    xyz[1] = -sin(Center[1] * m) * tmp_XYZ[0] + cos(Center[1] * m) * tmp_XYZ[1];
    xyz[2] = cos(Center[0] * m) * cos(Center[1] * m) * tmp_XYZ[0] + cos(Center[0] * m) * sin(Center[1] * m) * tmp_XYZ[1]
             + sin(Center[0] * m) * tmp_XYZ[2] - a; 
}

void xyz_to_XYZ(Eigen::Vector3d &xyz, Eigen::Vector3d &Center, Eigen::Vector3d &XYZ) {
    double a = 6378137;//a为椭球的长半轴:a=6378.137km
    double b = 6356752.3141;//b为椭球的短半轴:a=6356.7523141km
    double H0 = Center[2]; //+ a;
    // double e=sqrt(1-pow(b ,2)/pow(a ,2)); //e为椭球的第一偏心率
    // double e=sqrt(0.006693421622966); //克拉索夫斯基椭球
    // double e=sqrt(0.006694384999588); //1975年国际椭球
    double e = sqrt(0.0066943799013); //WGS-84椭球
    double m = M_PI/180;//经度维度需要转换成弧度.
    double B0 = Center[0] * m;
    double L0 = Center[1] * m;
    double W = sqrt(1-pow(e, 2) * pow(sin(B0), 2));
    double N0=a/W; //N为椭球的卯酉圈曲率半径
    
    XYZ[0] = (N0 + H0) * cos(B0) * cos(L0)
            -sin(B0) * cos(L0) * xyz[0] - sin(L0) * xyz[1] + cos(B0) * cos(L0) * xyz[2];
    XYZ[1] = (N0 + H0) * cos(B0) * sin(L0)
            -sin(B0) * sin(L0) * xyz[0] - cos(L0) * xyz[1] + cos(B0) * sin(L0) * xyz[2];
    XYZ[2] = (N0 * (1-pow(e, 2)) + H0) * sin(B0)
            -cos(B0) * xyz[0] + sin(B0) * xyz[2]; 
}

#endif