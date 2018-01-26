
#include "navigation.hxx"

Navigation::Navigation() {
  ekf_ = new EKF15();
}

void Navigation::InitializeNavigation(const FmuData fmuData) {
  GlobalDefsToImu(fmuData,&imu_);
  GlobalDefsToGps(fmuData,&gps_);

  if (!Initialized) {
    if (fmuData.Gps[0].Fix) {
      nav_ = ekf_->init(imu_,gps_);
      Initialized = true;
    } else {
      Initialized = false;
    }
  }
}

void Navigation::RunNavigation(const FmuData fmuData, NavOut *NavOutPtr) {
  GlobalDefsToImu(fmuData,&imu_);
  GlobalDefsToGps(fmuData,&gps_);
  nav_ = ekf_->update(imu_,gps_);
  NavToGlobalDefs(nav_,NavOutPtr);

  navOut_ = *NavOutPtr;
}

void Navigation::GlobalDefsToImu(const FmuData fmuData, IMUdata *ImuDataPtr) {
  ImuDataPtr->time = fmuData.Time_us/1000000.0L;

  ImuDataPtr->p = fmuData.Mpu9250.Gyro_rads[0];
  ImuDataPtr->q = fmuData.Mpu9250.Gyro_rads[1];
  ImuDataPtr->r = fmuData.Mpu9250.Gyro_rads[2];

  ImuDataPtr->ax = fmuData.Mpu9250.Accel_mss[0];
  ImuDataPtr->ay = fmuData.Mpu9250.Accel_mss[1];
  ImuDataPtr->az = fmuData.Mpu9250.Accel_mss[2];

  ImuDataPtr->hx = fmuData.Mpu9250.Mag_uT[0] * uT2G_;
  ImuDataPtr->hy = fmuData.Mpu9250.Mag_uT[1] * uT2G_;
  ImuDataPtr->hz = fmuData.Mpu9250.Mag_uT[2] * uT2G_;

  ImuDataPtr->temp = fmuData.Mpu9250.Temp_C;
}

void Navigation::GlobalDefsToGps(const FmuData fmuData, GPSdata *GpsDataPtr) {
  GpsDataPtr->time = fmuData.Gps[0].Sec;  
  
  GpsDataPtr->lat = fmuData.Gps[0].LLA[0];
  GpsDataPtr->lon = fmuData.Gps[0].LLA[1];
  GpsDataPtr->alt = fmuData.Gps[0].LLA[2];

  GpsDataPtr->vn = fmuData.Gps[0].NEDVelocity_ms[0];
  GpsDataPtr->ve = fmuData.Gps[0].NEDVelocity_ms[1];
  GpsDataPtr->vd = fmuData.Gps[0].NEDVelocity_ms[2];

  GpsDataPtr->sats = fmuData.Gps[0].NumberSatellites;

  // Use changes in TOW to determine if new data has been aqcuired off the GPS.
  if (fmuData.Gps[0].TOW != PrevTime_) {
    GpsDataPtr->newData = true;
    PrevTime_ = fmuData.Gps[0].TOW;
  } else {
    GpsDataPtr->newData = false;
  }
}

void Navigation::NavToGlobalDefs(const NAVdata navData, NavOut *NavOutPtr) {
  NavOutPtr->Time_s = navData.time;
  
  NavOutPtr->LLA[0] = navData.lat;
  NavOutPtr->LLA[1] = navData.lon;
  NavOutPtr->LLA[2] = navData.alt;

  NavOutPtr->NEDVelocity_ms[0] = navData.vn;
  NavOutPtr->NEDVelocity_ms[1] = navData.ve;
  NavOutPtr->NEDVelocity_ms[2] = navData.vd;

  NavOutPtr->Euler_rad[0] = navData.phi;
  NavOutPtr->Euler_rad[1] = navData.the;
  NavOutPtr->Euler_rad[2] = navData.psi;

  NavOutPtr->Quaternion[0] = navData.qw;
  NavOutPtr->Quaternion[1] = navData.qx;
  NavOutPtr->Quaternion[2] = navData.qy;
  NavOutPtr->Quaternion[3] = navData.qz;

  NavOutPtr->AccelBias_mss[0] = navData.abx;
  NavOutPtr->AccelBias_mss[1] = navData.aby;
  NavOutPtr->AccelBias_mss[2] = navData.abz;

  NavOutPtr->GyroBias_rads[0] = navData.gbx;
  NavOutPtr->GyroBias_rads[1] = navData.gby;
  NavOutPtr->GyroBias_rads[2] = navData.gbz;

  NavOutPtr->Pp[0] = navData.Pp0;
  NavOutPtr->Pp[1] = navData.Pp1;
  NavOutPtr->Pp[2] = navData.Pp2;

  NavOutPtr->Pv[0] = navData.Pv0;
  NavOutPtr->Pv[1] = navData.Pv1;
  NavOutPtr->Pv[2] = navData.Pv2;

  NavOutPtr->Pa[0] = navData.Pa0;
  NavOutPtr->Pa[1] = navData.Pa1;
  NavOutPtr->Pa[2] = navData.Pa2;

  NavOutPtr->Pab[0] = navData.Pabx;
  NavOutPtr->Pab[1] = navData.Paby;
  NavOutPtr->Pab[2] = navData.Pabz;

  NavOutPtr->Pgb[0] = navData.Pgbx;
  NavOutPtr->Pgb[1] = navData.Pgby;
  NavOutPtr->Pgb[2] = navData.Pgbz;
}

NavLog Navigation::Log(const NavOut& navOut)
{
  NavLog navLog;

  navLog.Time_s = navOut.Time_s;

  for (int i = 0; i < 3; i++) {
    navLog.LLA[i] = navOut.LLA[i];
    navLog.NEDVelocity_ms[i] = (float) navOut.NEDVelocity_ms[i];
    navLog.Euler_rad[i] = (float) navOut.Euler_rad[i];
    navLog.AccelBias_mss[i] = (float) navOut.AccelBias_mss[i];
    navLog.GyroBias_rads[i] = (float) navOut.GyroBias_rads[i];
    navLog.Pp[i] = (float) navOut.Pp[i];
    navLog.Pv[i] = (float) navOut.Pv[i];
    navLog.Pa[i] = (float) navOut.Pa[i];
    navLog.Pab[i] = (float) navOut.Pab[i];
    navLog.Pgb[i] = (float) navOut.Pgb[i];
  }
  for (int i = 0; i < 4; i++) {
    navLog.Quaternion[i] = (float) navOut.Quaternion[i];
  }

  return navLog;
}
