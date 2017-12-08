
#ifndef NAVIGATION_HXX_
#define NAVIGATION_HXX_

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include <stdexcept>

#include "global-defs.hxx"
#include "EKF_15state.hxx"

struct NavLog {
  double Time_s;             // [sec], timestamp of NAV filter
  double LLA[3];             // Latitude (rad), Longitude (rad), Altitude (m)
  double NEDVelocity_ms[3];  // NED Velocity, m/s
  double Euler_rad[3];       // Euler angles, rad
  double AccelBias_mss[3];   // x,y,z accelerometer bias, m/s/s
  double GyroBias_rads[3];   // x,y,z gyro bias, rad/s
  double Pp[3];              // [rad], covariance estimate for position
  double Pv[3];              // [rad], covariance estimate for velocity
  double Pa[3];              // [rad], covariance estimate for angles
  double Pab[3];             // [rad], covariance estimate for accelerometer bias
  double Pgb[3];             // [rad], covariance estimate for rate gyro bias
  double Quaternion[4];      // Quaternion estimate
};

struct NavOut {
  double Time_s;                            // [sec], timestamp of NAV filter
  Eigen::Matrix<double,3,1> LLA;            // Latitude (rad), Longitude (rad), Altitude (m)
  Eigen::Matrix<double,3,1>NEDVelocity_ms;  // NED Velocity, m/s
  Eigen::Matrix<double,3,1>Euler_rad;       // Euler angles, rad
  Eigen::Matrix<double,4,1>Quaternion;      // Quaternion estimate
  Eigen::Matrix<double,3,1>AccelBias_mss;   // x,y,z accelerometer bias, m/s/s
  Eigen::Matrix<double,3,1>GyroBias_rads;   // x,y,z gyro bias, rad/s
  Eigen::Matrix<double,3,1>Pp;              // [rad], covariance estimate for position
  Eigen::Matrix<double,3,1>Pv;              // [rad], covariance estimate for velocity
  Eigen::Matrix<double,3,1>Pa;              // [rad], covariance estimate for angles
  Eigen::Matrix<double,3,1>Pab;             // [rad], covariance estimate for accelerometer bias
  Eigen::Matrix<double,3,1>Pgb;             // [rad], covariance estimate for rate gyro bias
};

class Navigation {
  public:
    Navigation();
    void InitializeNavigation(const FmuData fmuData);
    void RunNavigation(const FmuData fmuData, NavOut *NavOutPtr);
    bool Initialized = false;
    NavLog Log();

  private:
    EKF15 *ekf_;
    NAVconfig config_;
    NAVdata nav_;
    GPSdata gps_;
    IMUdata imu_;

    double PrevTime_;

    const float uT2G_ = 0.01f;

    void GlobalDefsToImu(const FmuData fmuData, IMUdata *ImuDataPtr);
    void GlobalDefsToGps(const FmuData fmuData, GPSdata *GpsDataPtr);
    void NavToGlobalDefs(const NAVdata navData, NavOut *NavOutPtr);
};

#endif
