
#ifndef NAVIGATION_HXX_
#define NAVIGATION_HXX_

#include "global-defs.hxx"
#include "EKF_15state.hxx"

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

class Navigation {
  public:
    Navigation();
    void InitializeNavigation(const FmuData FmuDataRef);
    void RunNavigation(const FmuData FmuDataRef, NavigationData *NavigationDataPtr);
    bool Initialized = false;
  private:
    EKF15 *ekf_;
    NAVconfig config_;
    NAVdata nav_;
    GPSdata gps_;
    IMUdata imu_;

    double PrevTime_;

    const float uT2G_ = 0.01f;

    void GlobalDefsToImu(const FmuData FmuDataRef, IMUdata *ImuDataPtr);
    void GlobalDefsToGps(const FmuData FmuDataRef, GPSdata *GpsDataPtr);
    void NavToGlobalDefs(const NAVdata NavDataRef, NavigationData *NavigationDataPtr);
};

#endif
