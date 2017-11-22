/*
Simple control allocation tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include <cstdio>
#include <ctime>

#include <iostream>
#include "cntrlAllocMgr.hxx"
//#include "cntrlAllocFunc.hxx"


static const float kD2R = M_PI / 180.0;

int main(void)  /* Program tester */
{
  // Define Control Allocation
  CntrlAllocMgr cntrlAllocMgr; // Create the Control Allocator

  uint8_t numObj = 3;
  uint8_t numEff = 6;

  // Setup Timer
  std::clock_t tStart_tick, tStop_tick;
  double tElaps_s;

  MatCntrlEff cntrlEff(numObj, numEff); // effectiveness matrix
  VecObj vObj(numObj), vErr(numObj); // objectives
  VecEff uCmd(numEff), uMin(numEff), uMax(numEff), uPref(numEff); // cmd, min, max, prefered effector commands
  MatObj wtObj(numObj, numObj); // Objective weighting matrix
  MatEff wtEff(numEff, numEff); // Effector weighting matrix

  // Define Problem

  // Surface Order - Elev, Rud, AilL, FlapL, FlapR, AilR
  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
  // cntrlEff << 0.0000, -5.0084, 78.235,  33.013, -33.013, -78.235,
  //            -133.69, -0.0047, 3.0002,  2.6238,  2.6238,  3.0002,
  //             0.0000, -82.041, 5.7521, -1.7941,  1.7941,  5.7521; // rad/s per rad

  cntrlEff << 0.0000,  0.0000, 78.235,  33.013, -33.013, -78.235,
             -133.69,  0.0000, 0.0000,  0.0000,  0.0000,  0.0000,
              0.0000, -82.041, 0.0000,  0.0000,  0.0000,  0.0000; // rad/s per rad

  cntrlEff *= kD2R;

  uMin << -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R ;
  uMax <<  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R ;
  uPref << 0.0,        0.0,        0.0,        0.0,        0.0,        0.0;

  vObj << -20*kD2R,  5*kD2R,  2*kD2R; // Command rad/s


  std::cout << "Matrix cntrlEff:\n" << cntrlEff << std::endl;
  std::cout << "Vector vObj:\n" << vObj << std::endl;

  uCmd.Zero(numEff);

  wtObj.setIdentity();
  wtEff.setIdentity();


  cntrlAllocMgr.Init(cntrlEff, wtObj, wtEff, uMin, uMax, uPref);     // Initialize Control Allocator


  tStart_tick = std::clock(); // Start the timer

  // Call control allocation
  int numIter = 1000;
  for(int i = 0 ; i < numIter ; i++){

    uCmd = cntrlAllocMgr.Compute(vObj);

    //uCmd = CntrlAllocPseudo(cntrlEff, vObj, uPref);
    //uCmd = CntrlAllocPseudoWt(cntrlEff, vObj, wtObj, wtEff, uPref);

    //float gammaEff = .001;
    //uint8_t numIter = 200;
    //uCmd = CntrlAllocFxp(cntrlEff, vObj, uMin, uMax, wtObj, wtEff, uPref, gammaEff, numIter);
  }
  tStop_tick = std::clock(); // stop the timer
  tElaps_s = ( tStop_tick - tStart_tick ) / (double) CLOCKS_PER_SEC; // estimate elapsed time

  std::cout<<"Average tElaps_s: "<< tElaps_s / numIter <<'\n';

  std::cout << "Unsaturated Solution:\n" << uCmd/kD2R << std::endl;

  vErr = vObj - (cntrlEff * uCmd);
  std::cout << "Unsaturation Solution Error:\n" << vErr << std::endl;

  // Apply Saturation
  Saturate(uMin, uMax, uCmd);

  std::cout << "Saturated Solution:\n" << uCmd/kD2R << std::endl;

  vErr = vObj - (cntrlEff * uCmd);
  std::cout << "Saturation Solution Error:\n" << vErr << std::endl;
}



