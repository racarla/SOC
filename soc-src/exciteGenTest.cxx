/*
// Simple excitation generation tester
*/

#include <iostream>
#include "exciteGenFunc.hxx"


int main(void)  /* Program tester */
{
  float TimeStart_s = 1.0;
  float TimeStep_s = 1.0/10.0;
  float TimeCurr_s;
  float Excite_nd;
  int ExciteFlag;

  // Doublets
  float TimeDur_s = 1.0;
  float TimeEnd_s = 2.0*TimeStart_s + 2.0*TimeDur_s;
  float Amp_nd = 1.0;

  int numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "Doublet" << std::endl;
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteDoublet(TimeCurr_s, TimeStart_s, TimeDur_s, Amp_nd, Excite_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag<< "\t" << Excite_nd << std::endl;
  }

  // Doublet121
  TimeEnd_s = 2.0*TimeStart_s + 4.0*TimeDur_s;

  numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "Doublet121" << std::endl;
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteDoublet121(TimeCurr_s, TimeStart_s, TimeDur_s, Amp_nd, Excite_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag<< "\t" << Excite_nd << std::endl;
  }


  // Doublet3211
  TimeEnd_s = 2.0*TimeStart_s + 7.0*TimeDur_s;

  numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "Doublet3211" << std::endl;
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteDoublet3211(TimeCurr_s, TimeStart_s, TimeDur_s, Amp_nd, Excite_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag<< "\t" << Excite_nd << std::endl;
  }

  // Chirp
  float FreqStart_rps = 10.0;
  float FreqEnd_rps = 1.0;
  float AmpStart_nd = 10.0;
  float AmpEnd_nd = 1.0;
  TimeDur_s = 20.0;
  TimeEnd_s = 2.0*TimeStart_s + TimeDur_s;

  numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "Linear Chirp" << std::endl;
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteChirp(TimeCurr_s, TimeStart_s, TimeDur_s,
      FreqStart_rps, FreqEnd_rps, AmpStart_nd, AmpEnd_nd,
      Excite_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag<< "\t" << Excite_nd << std::endl;
  }


  // OMS
  int numElem = 3;
  VectorElem FreqVec_rps, PhaseVec_rad, AmpVec_nd;

  FreqVec_rps.conservativeResize(numElem);
  PhaseVec_rad.conservativeResize(numElem);
  AmpVec_nd.conservativeResize(numElem);

  FreqVec_rps << 1.0, 2.0, 3.0;
  PhaseVec_rad << 0.0, 0.0, 0.0;
  AmpVec_nd << 1.0, 1.0, 1.0;

  TimeDur_s = 20.0;
  TimeEnd_s = 2.0*TimeStart_s + TimeDur_s;

  numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "OMS" << std::endl;
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteOMS(TimeCurr_s, TimeStart_s, TimeDur_s,
      FreqVec_rps, PhaseVec_rad, AmpVec_nd,
      Excite_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag<< "\t" << Excite_nd << std::endl;
  }

// Multichannel OMS
  int numChan = 3;
  numElem = 4;
  MatrixElem FreqMat_rps, PhaseMat_rad, AmpMat_nd;
  VectorExcite ExciteVec_nd;

  FreqMat_rps.conservativeResize(numChan, numElem);
  PhaseMat_rad.conservativeResize(numChan, numElem);
  AmpMat_nd.conservativeResize(numChan, numElem);
  ExciteVec_nd.conservativeResize(numChan);

  FreqMat_rps << 1.0, 2.0, 3.0, 4.0, 
                 2.0, 3.0, 4.0, 5.0, 
                 3.0, 4.0, 5.0, 6.0;
  PhaseMat_rad << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0;
  AmpMat_nd << 1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0,
               1.0, 1.0, 1.0, 1.0;;

  TimeDur_s = 20.0;
  TimeEnd_s = 2.0*TimeStart_s + TimeDur_s;

  numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "MutiChannel OMS" << std::endl;
  
  int exciteFlag = 0;

  // Loop over each channel
  std::cout << "TimeCurr_s" << "\t" << "ExciteFlag" << "\t" << "Excite_nd" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    ExciteFlag = ExciteMultiOMS(TimeCurr_s, TimeStart_s, TimeDur_s,
      FreqMat_rps, PhaseMat_rad, AmpMat_nd,
      ExciteVec_nd);
  
    std::cout << TimeCurr_s << "\t" << ExciteFlag << "\t" << ExciteVec_nd.transpose() << std::endl;
  }

}



