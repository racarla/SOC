/*
// Simple control allocation tester
*/

#include <cstdio>
#include <ctime>

#include <iostream>
#include "cntrlAllocFunc.hxx"


int main(void)  /* Program tester */
{
  // Setup Timer
  std::clock_t tStart_tick, tStop_tick;
  double tElaps_s;

  MatCntrlEff   cntrlEff; // effectiveness matrix
  VecObj vObj, vErr; // objectives
  VecEff uCmd, uMin, uMax, uPref; // cmd, min, max, prefered effector commands
  MatObj wObj; // Objective weighting matrix
  MatEff wEff; // Effector weighting matrix
  
  int numObj = 3;
  int numEff = 6;

  cntrlEff.conservativeResize(numObj, numEff);
  vObj.conservativeResize(numObj);
  vErr.conservativeResize(numObj);
  uCmd.conservativeResize(numEff);
  uMin.conservativeResize(numEff);
  uMax.conservativeResize(numEff);
  uPref.conservativeResize(numEff);
  wObj.conservativeResize(numObj, numObj);
  wEff.conservativeResize(numEff, numEff);

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

  wObj.setIdentity();
  wEff.setIdentity();

  tStart_tick = std::clock(); // Start the timer

  // Call control allocation
  int numIter = 1000;
  for(int i = 0 ; i < numIter ; i++){
    //ContAllocPseudo(cntrlEff, vObj, uPref, uCmd);
    //ContAllocPseudoWeighted(cntrlEff, vObj, wObj, wEff, uPref, uCmd);

    float gammaEff = .001;
    int numIter = 200;
    ContAllocFxp(cntrlEff, vObj, uMin, uMax, wObj, wEff, uPref, uCmd, gammaEff, numIter);
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



