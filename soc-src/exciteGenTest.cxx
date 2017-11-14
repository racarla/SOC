/*
Simple excitation generation tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include <iostream>
#include "exciteGenFunc.hxx"


int main(void)  /* Program tester */
{


ExciteGenDisc TestExc;
VecChan timeStart_s, timeDur_s, amp_nd;

TestExc.SetParam(Doublet, timeStart_s, timeDur_s, amp_nd);


float time_s;
VecChan excite_nd;
TestExc.Compute(time_s, excite_nd)



  
  float timeStart_s = 1.0;
  float timeStep_s = 1.0/10.0;
  float timeCurr_s;
  float excite_nd;
  int exciteFlag;

  // Doublets
  float timeDur_s = 1.0;
  float timeEnd_s = 2.0*timeStart_s + 2.0*timeDur_s;
  float amp_nd = 1.0;

  int numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Doublet" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteDoublet(timeCurr_s, timeStart_s, timeDur_s, amp_nd, excite_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag<< "\t" << excite_nd << std::endl;
  }

  // Doublet121
  timeEnd_s = 2.0*timeStart_s + 4.0*timeDur_s;

  numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Doublet121" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteDoublet121(timeCurr_s, timeStart_s, timeDur_s, amp_nd, excite_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag<< "\t" << excite_nd << std::endl;
  }


  // Doublet3211
  timeEnd_s = 2.0*timeStart_s + 7.0*timeDur_s;

  numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Doublet3211" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteDoublet3211(timeCurr_s, timeStart_s, timeDur_s, amp_nd, excite_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag<< "\t" << excite_nd << std::endl;
  }

  // Chirp
  float freqStart_rps = 10.0;
  float freqEnd_rps = 1.0;
  float ampStart_nd = 10.0;
  float ampEnd_nd = 1.0;
  timeDur_s = 20.0;
  timeEnd_s = 2.0*timeStart_s + timeDur_s;

  numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Linear Chirp" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteChirp(timeCurr_s, timeStart_s, timeDur_s,
      freqStart_rps, freqEnd_rps, ampStart_nd, ampEnd_nd,
      excite_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag<< "\t" << excite_nd << std::endl;
  }


  // OMS
  int numElem = 3;
  VectorElem freqVec_rps, phaseVec_rad, ampVec_nd;

  freqVec_rps.conservativeResize(numElem);
  phaseVec_rad.conservativeResize(numElem);
  ampVec_nd.conservativeResize(numElem);

  freqVec_rps << 1.0, 2.0, 3.0;
  phaseVec_rad << 0.0, 0.0, 0.0;
  ampVec_nd << 1.0, 1.0, 1.0;

  timeDur_s = 20.0;
  timeEnd_s = 2.0*timeStart_s + timeDur_s;

  numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "OMS" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteOms(timeCurr_s, timeStart_s, timeDur_s,
      freqVec_rps, phaseVec_rad, ampVec_nd,
      excite_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag<< "\t" << excite_nd << std::endl;
  }

// Multichannel OMS
  int numChan = 3;
  numElem = 4;
  MatrixElem freqMat_rps, phaseMat_rad, ampMat_nd;
  VectorExcite exciteVec_nd;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_nd.conservativeResize(numChan, numElem);
  exciteVec_nd.conservativeResize(numChan);

  freqMat_rps << 1.0, 2.0, 3.0, 4.0, 
                 2.0, 3.0, 4.0, 5.0, 
                 3.0, 4.0, 5.0, 6.0;
  phaseMat_rad << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;
  ampMat_nd << 1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0;

  timeDur_s = 20.0;
  timeEnd_s = 2.0*timeStart_s + timeDur_s;

  numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "MutiChannel OMS" << std::endl;
  
  exciteFlag = 0;

  // Loop over each channel
  std::cout << "timeCurr_s" << "\t" << "exciteFlag" << "\t" << "excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    exciteFlag = ExciteMultiOms(timeCurr_s, timeStart_s, timeDur_s,
      freqMat_rps, phaseMat_rad, ampMat_nd,
      exciteVec_nd);
  
    std::cout << timeCurr_s << "\t" << exciteFlag << "\t" << exciteVec_nd.transpose() << std::endl;
  }

}



