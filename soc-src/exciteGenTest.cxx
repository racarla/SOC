/*
Simple excitation generation tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#include <iostream>
#include "exciteGenFunc.hxx"

void disc(void);
void chirp(void);
void oms(void);

int main(void) {
  disc();
  chirp();
  oms();
}

void disc(void)  /* Program tester */
{

  uint8_t numChan = 1;
  float timeStep_s = 1.0/10.0;

  VecChan timeStart_s(numChan); 
  VecChan timeOnePulse_s(numChan);
  VecChan amp_nd(numChan);
  VecChan excite_nd(numChan);

  timeStart_s << 1.0;
  timeOnePulse_s << 1.0;
  amp_nd << 1.0;

  ExciteDisc TestExcDoublet;
  TestExcDoublet.Init(kDoublet3211, timeStart_s, timeOnePulse_s, amp_nd);
  
  // Doublets
  float timeEnd_s = 2.0*timeStart_s.maxCoeff() + 7.0*timeOnePulse_s.maxCoeff();

  int numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Doublet" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "excite_nd" << std::endl;
  float timeCurr_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    excite_nd = TestExcDoublet.Compute(timeCurr_s);
  
    std::cout << timeCurr_s << "\t" << excite_nd.transpose() << std::endl;
  }
}

void chirp(void)
{

  // Chirp
  uint8_t numChan = 2;
  float timeStep_s = 1.0/10.0;

  VecChan timeStart_s(numChan);
  VecChan timeDur_s(numChan);
  VecChan freqStart_rps(numChan);
  VecChan freqEnd_rps(numChan);
  VecChan ampStart_nd(numChan);
  VecChan ampEnd_nd(numChan);
  VecChan excite_nd(numChan);

  timeStart_s << 1.0, 1.0;
  timeDur_s << 20.0, 20.0;
  freqStart_rps << 1.0, 0.1;
  freqEnd_rps << 0.1, 1.0;
  ampStart_nd << 1.0, 1.0;
  ampEnd_nd << 1.0, 1.0;


  ExciteChirp TestExcChirp;
  TestExcChirp.Init(kLinear, timeStart_s, timeDur_s, freqStart_rps, freqEnd_rps, ampStart_nd, ampEnd_nd);


  // Time Vectors
  float timeEnd_s = 2.0*timeStart_s.maxCoeff() + timeDur_s.maxCoeff();

  int numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "Linear Chirp" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "excite_nd" << std::endl;
  float timeCurr_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    excite_nd = TestExcChirp.Compute(timeCurr_s);
  
    std::cout << timeCurr_s << "\t" << excite_nd.transpose() << std::endl;
  }
}

void oms(void)
{

// Multichannel OMS

  uint8_t numChan = 3;
  uint8_t numElem = 4;
  float timeStep_s = 1.0/10.0;

  VecChan timeStart_s(numChan);
  VecChan timeDur_s(numChan);
  MatChanElem freq_rps(numChan, numElem);
  MatChanElem phase_rad(numChan, numElem);
  MatChanElem amp_nd(numChan, numElem);
  VecChan excite_nd(numChan);

  timeStart_s << 1.0, 1.0, 1.0;
  timeDur_s << 20.0, 20.0, 20.0;


  freq_rps << 1.0, 2.0, 3.0, 4.0, 
                 2.0, 3.0, 4.0, 5.0, 
                 3.0, 4.0, 5.0, 6.0;
  phase_rad << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;
  amp_nd << 1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0;


  ExciteMultisine TestExcOms;
  TestExcOms.Init(kOMS, timeStart_s, timeDur_s, freq_rps, phase_rad, amp_nd);

  // Time Vectors
  float timeEnd_s = 2.0*timeStart_s.maxCoeff() + timeDur_s.maxCoeff();

  int numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  std::cout << "MutiChannel OMS" << std::endl;
  std::cout << "timeCurr_s" << "\t" << "excite_nd" << std::endl;
  float timeCurr_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    timeCurr_s = (float) iIter * timeStep_s;

    excite_nd = TestExcOms.Compute(timeCurr_s);
  
    std::cout << timeCurr_s << "\t" << excite_nd.transpose() << std::endl;
  }
}


