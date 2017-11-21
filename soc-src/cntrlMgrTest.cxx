/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#include <iostream>
#include "cntrlMgr.hxx"


int main(void)  /* Program tester */
{
  float timeStart_s = 0.0;
  float timeInit_s = 1.0;
  float timeEngage_s = 2.0;
  float timeHold_s = 4.0;
  float TimeReset_s = 5.0;
  float TimeEnd_s = 6.0;
  float TimeStep_s = 1.0/10.0;
  float TimeCurr_s;


  CntrlMgr cntrlMgr;       // Create the Controller Manager
  cntrlMgr.Init(); // Define the Baseline and Research Controller

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.5 * TimeStep_s;
  float dMeas = 0.0;

  int numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"   << "\t" << "intErr" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;


      VecCmd refVec(4);
      refVec[0] = 1;//Data.SbusRx[0].Inceptors[0];
      refVec[1] = 1;//Data.SbusRx[0].Inceptors[1];
      refVec[2] = 1;//Data.SbusRx[0].Inceptors[2];
      refVec[3] = 1;//Data.SbusRx[0].Inceptors[4];

      VecCmd measVecAngles(4);
      measVecAngles[0] = meas;//NavData.Euler_rad[0];
      measVecAngles[1] = 0;//NavData.Euler_rad[1];
      measVecAngles[2] = 0;//NavData.Euler_rad[2];
      measVecAngles[3] = 15; // FIXIT - Need airspeed

      VecCmd measVecRates(4);
      measVecRates[0] = 0;//Data.Mpu9250.Gyro_rads[0];
      measVecRates[1] = 0;//ata.Mpu9250.Gyro_rads[1];
      measVecRates[2] = 0;//Data.Mpu9250.Gyro_rads[2];
      measVecRates[3] = 10;


    if (TimeCurr_s <= timeInit_s) {
      cntrlMgr.Mode(kCntrlStandby);

    } else if (TimeCurr_s <= timeEngage_s) {
      cntrlMgr.Mode(kCntrlInit);

    } else if (TimeCurr_s <= timeHold_s) {
      cntrlMgr.Mode(kCntrlEngage);
      
    } else if (TimeCurr_s <= TimeReset_s) {
      cntrlMgr.Mode(kCntrlHold);
      
    } else if (TimeCurr_s <= TimeEnd_s) {
      cntrlMgr.Mode(kCntrlReset);
    }


std::cout << refVec.transpose() << "\t" << measVecAngles.transpose() << "\t" << measVecRates.transpose() << std::endl;
      VecCmd cmdBase = cntrlMgr.CmdBase(refVec, TimeCurr_s);
      VecCmd cmdRes = cntrlMgr.CmdRes(refVec, measVecAngles, measVecRates, TimeCurr_s);
     VecCmd cmdCntrl = cntrlMgr.Cmd();
std::cout << cmdBase.transpose() << std::endl;





    meas += measStep;

  }
}
