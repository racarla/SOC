
#include "navigation.hxx"

Navigation::Navigation() {
  ekf_ = new EKF15();
}

void Navigation::InitializeNavigation(const FmuData FmuDataRef) {
  GlobalDefsToImu(FmuDataRef,&imu_);
  GlobalDefsToGps(FmuDataRef,&gps_);

  if (!Initialized) {
    if (FmuDataRef.Gps[0].Fix) {
      nav_ = ekf_->init(imu_,gps_);
      Initialized = true;
    } else {
      Initialized = false;
    }
  }
}

void Navigation::RunNavigation(const FmuData FmuDataRef, NavigationData *NavigationDataPtr) {
  GlobalDefsToImu(FmuDataRef,&imu_);
  GlobalDefsToGps(FmuDataRef,&gps_);
  nav_ = ekf_->update(imu_,gps_);
  NavToGlobalDefs(nav_,NavigationDataPtr);
}

void Navigation::GlobalDefsToImu(const FmuData FmuDataRef, IMUdata *ImuDataPtr) {
  ImuDataPtr->time = FmuDataRef.Time_us/1000000.0L;

  ImuDataPtr->p = FmuDataRef.Mpu9250.Gyro_rads(0,0);
  ImuDataPtr->q = FmuDataRef.Mpu9250.Gyro_rads(1,0);
  ImuDataPtr->r = FmuDataRef.Mpu9250.Gyro_rads(2,0);

  ImuDataPtr->ax = FmuDataRef.Mpu9250.Accel_mss(0,0);
  ImuDataPtr->ay = FmuDataRef.Mpu9250.Accel_mss(1,0);
  ImuDataPtr->az = FmuDataRef.Mpu9250.Accel_mss(2,0);

  ImuDataPtr->hx = FmuDataRef.Mpu9250.Mag_uT(0,0) * uT2G_;
  ImuDataPtr->hy = FmuDataRef.Mpu9250.Mag_uT(1,0) * uT2G_;
  ImuDataPtr->hz = FmuDataRef.Mpu9250.Mag_uT(2,0) * uT2G_;

  ImuDataPtr->temp = FmuDataRef.Mpu9250.Temp_C;
}

void Navigation::GlobalDefsToGps(const FmuData FmuDataRef, GPSdata *GpsDataPtr) {
  GpsDataPtr->time = FmuDataRef.Gps[0].Sec;
  
  GpsDataPtr->lat = FmuDataRef.Gps[0].LLA(0,0);
  GpsDataPtr->lon = FmuDataRef.Gps[0].LLA(1,0);
  GpsDataPtr->alt = FmuDataRef.Gps[0].LLA(2,0);

  GpsDataPtr->vn = FmuDataRef.Gps[0].NEDVelocity_ms(0,0);
  GpsDataPtr->ve = FmuDataRef.Gps[0].NEDVelocity_ms(1,0);
  GpsDataPtr->vd = FmuDataRef.Gps[0].NEDVelocity_ms(2,0);

  GpsDataPtr->sats = FmuDataRef.Gps[0].NumberSatellites;
  
  if (GpsDataPtr->time != PrevTime_) {
    GpsDataPtr->newData = true;
    PrevTime_ = GpsDataPtr->time;
  } else {
    GpsDataPtr->newData = false;
  }
}

void Navigation::NavToGlobalDefs(const NAVdata NavDataRef, NavigationData *NavigationDataPtr) {
  NavigationDataPtr->Time_s = NavDataRef.time;
  
  NavigationDataPtr->LLA(0,0) = NavDataRef.lat;
  NavigationDataPtr->LLA(1,0) = NavDataRef.lon;
  NavigationDataPtr->LLA(2,0) = NavDataRef.alt;

  NavigationDataPtr->NEDVelocity_ms(0,0) = NavDataRef.vn;
  NavigationDataPtr->NEDVelocity_ms(1,0) = NavDataRef.ve;
  NavigationDataPtr->NEDVelocity_ms(2,0) = NavDataRef.vd;

  NavigationDataPtr->Euler_rad(0,0) = NavDataRef.phi;
  NavigationDataPtr->Euler_rad(1,0) = NavDataRef.the;
  NavigationDataPtr->Euler_rad(2,0) = NavDataRef.psi;

  NavigationDataPtr->Quaternion(0,0) = NavDataRef.qw;
  NavigationDataPtr->Quaternion(1,0) = NavDataRef.qx;
  NavigationDataPtr->Quaternion(2,0) = NavDataRef.qy;
  NavigationDataPtr->Quaternion(3,0) = NavDataRef.qz;

  NavigationDataPtr->AccelBias_mss(0,0) = NavDataRef.abx;
  NavigationDataPtr->AccelBias_mss(1,0) = NavDataRef.aby;
  NavigationDataPtr->AccelBias_mss(2,0) = NavDataRef.abz;

  NavigationDataPtr->GyroBias_rads(0,0) = NavDataRef.gbx;
  NavigationDataPtr->GyroBias_rads(1,0) = NavDataRef.gby;
  NavigationDataPtr->GyroBias_rads(2,0) = NavDataRef.gbz;

  NavigationDataPtr->Pp(0,0) = NavDataRef.Pp0;
  NavigationDataPtr->Pp(1,0) = NavDataRef.Pp1;
  NavigationDataPtr->Pp(2,0) = NavDataRef.Pp2;

  NavigationDataPtr->Pv(0,0) = NavDataRef.Pv0;
  NavigationDataPtr->Pv(1,0) = NavDataRef.Pv1;
  NavigationDataPtr->Pv(2,0) = NavDataRef.Pv2;

  NavigationDataPtr->Pa(0,0) = NavDataRef.Pa0;
  NavigationDataPtr->Pa(1,0) = NavDataRef.Pa1;
  NavigationDataPtr->Pa(2,0) = NavDataRef.Pa2;

  NavigationDataPtr->Pab(0,0) = NavDataRef.Pabx;
  NavigationDataPtr->Pab(1,0) = NavDataRef.Paby;
  NavigationDataPtr->Pab(2,0) = NavDataRef.Pabz;

  NavigationDataPtr->Pgb(0,0) = NavDataRef.Pgbx;
  NavigationDataPtr->Pgb(1,0) = NavDataRef.Pgby;
  NavigationDataPtr->Pgb(2,0) = NavDataRef.Pgbz;
}
