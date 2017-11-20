/*
Simple control allocation tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include <cstdio>
#include <ctime>

#include <iostream>
#include "cntrlAllocFunc.hxx"


int main(void)  /* Program tester */
{
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
  cntrlEff << 7712, -25060,   5603, -7712, -25060, -5603, 
      -1025,      0,   6862,  5582,      0, 861.9, 
      -5582,      0, -861.9,  1284,      0, -17460;

  vObj << -7.0E+5,  8.0E+5,  9.0E+5;

  uMin << -29.0, -29.0, -15.0, -20.0, -20.0, -30.0 ;
  uMax <<  15.0,  15.0,  15.0,  20.0,  20.0,  30.0 ;
  uPref <<  0.0,   0.0,   0.0,   0.0,   0.0,   0.0 ;


  std::cout << "Matrix cntrlEff:\n" << cntrlEff << std::endl;
  std::cout << "Vector vObj:\n" << vObj << std::endl;

  uCmd.Zero(numEff);

  wtObj.setIdentity();
  wtEff.setIdentity();

  tStart_tick = std::clock(); // Start the timer

  // Call control allocation
  int numIter = 1000;
  for(int i = 0 ; i < numIter ; i++){
    //CntrlAllocPseudo(cntrlEff, vObj, uPref, uCmd);
    //CntrlAllocPseudoWt(cntrlEff, vObj, wtObj, wtEff, uPref, uCmd);

    float gammaEff = .001;
    uint8_t numIter = 200;
    CntrlAllocFxp(cntrlEff, vObj, uMin, uMax, wtObj, wtEff, uPref, uCmd, gammaEff, numIter);
  }
  tStop_tick = std::clock(); // stop the timer
  tElaps_s = ( tStop_tick - tStart_tick ) / (double) CLOCKS_PER_SEC; // estimate elapsed time

  std::cout<<"Average tElaps_s: "<< tElaps_s / numIter <<'\n';

  std::cout << "Unsaturated Solution:\n" << uCmd << std::endl;

  vErr = vObj - (cntrlEff * uCmd);
  std::cout << "Unsaturation Solution Error:\n" << vErr << std::endl;

  // Apply Saturation
  Saturate(uMin, uMax, uCmd);

  std::cout << "Saturated Solution:\n" << uCmd << std::endl;

  vErr = vObj - (cntrlEff * uCmd);
  std::cout << "Saturation Solution Error:\n" << vErr << std::endl;
}



