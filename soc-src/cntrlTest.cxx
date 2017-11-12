/*
Simple excitation generation tester

See: LICENSE.md for Copyright and License Agreement

History:
Chris Regan
2017-11-12 - Chris Regan - Defined cntrlPIDamp class and methods

*/

#include <iostream>
#include "cntrlFunc.hxx"


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


  float kP = 0.1;
  float kI = 1.0;
  float kD = 0.0;
  float kDamp = -0.0;
  float cmdRng[2] = {-0.5, 0.5};


  cntrlPID testPID1;
  testPID1.setParam(kP, kI, kD, cmdRng[0], cmdRng[1]);

  cntrlPID testPID2;
  testPID2.setParam(kP, kI, kD, cmdRng[0], cmdRng[1]);

  cntrlPIDamp testPIDamp;
  testPIDamp.setParam(kP, kI, kDamp, cmdRng[0], cmdRng[1]);

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.5 * TimeStep_s;
  float dMeas = 0.0;

  int numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"   << "\t" << "intErr" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    if (TimeCurr_s <= timeInit_s) {
      testPID1.runMode = 0;

    } else if (TimeCurr_s <= timeEngage_s) {
      testPID1.runMode = 2;

    } else if (TimeCurr_s <= timeHold_s) {
      testPID1.runMode = 3;
      
    } else if (TimeCurr_s <= TimeReset_s) {
      testPID1.runMode = 1;
      testPIDamp.iErr = 0; // Should have no effect on PID
      testPID2.iErr = 0; // Should have no effect on PID
      
    } else if (TimeCurr_s <= TimeEnd_s) {
      testPID1.runMode = -1;
    }

    meas += measStep;

    float cmdPID = testPID1.compute(ref, meas, TimeStep_s);
    int runMode = testPID1.runMode;
    float intErr = testPID1.iErr;

    float cmdPIDamp = testPIDamp.compute(ref, meas, dMeas, TimeStep_s);

    std::cout << TimeCurr_s << "\t" << runMode << "\t" << ref <<"\t" << meas << "\t" << (ref-meas) << "\t" << cmdPID   << "\t" << intErr << std::endl;
  }
}
