/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#include <iostream>
#include "exciteGenFunc.hxx"

/* Discrete Excitations */
ExciteDisc::ExciteDisc(void)
{

}

void ExciteDisc::SetParamDisc(kExciteDisc discType, VecChan timeVecStart_s, VecChan timeVecOnePulse_s, VecChan ampVec_nd)
{
  discType_ = discType;
  numChan_ = timeVecStart_s.size();

  timeVecStart_s_ = timeVecStart_s;
  timeVecOnePulse_s_ = timeVecOnePulse_s;
  ampVec_nd_ = ampVec_nd;

  timeVecDur_s_.setZero(numChan_);

  for (int iChan = 0; iChan < numChan_; iChan++) {
    switch (discType_){
      case Pulse: timeVecDur_s_(iChan) = 1 * timeVecOnePulse_s_(iChan);
      case Doublet: timeVecDur_s_(iChan) = 2 * timeVecOnePulse_s_(iChan);
      case Doublet121: timeVecDur_s_(iChan) = 4 * timeVecOnePulse_s_(iChan);
      case Doublet3211: timeVecDur_s_(iChan) = 7 * timeVecOnePulse_s_(iChan);
    }
  }
std::cout << "TEST" << std::endl;

}

int ExciteDisc::ComputeDisc(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;
  VecChanInt exciteVecFlag;

  exciteVec_nd.setZero(numChan_);
  exciteVecFlag.setZero(numChan_);

  for (int iChan = 0; iChan < numChan_; iChan++) {
    timeDur_s = timeVecDur_s_(iChan);
    time_s = timeCurr_s - timeVecStart_s_(iChan);

    if ((time_s >= 0) && (time_s < timeDur_s)) {

      switch (discType_){
        case Pulse:
          exciteVec_nd(iChan) = SigPulse(time_s, timeVecOnePulse_s_(iChan), ampVec_nd_(iChan));
          break;

        case Doublet:
          exciteVec_nd(iChan) = SigDoublet(time_s, timeVecOnePulse_s_(iChan), ampVec_nd_(iChan));
          break;

        case Doublet121:
          exciteVec_nd(iChan) = SigDoublet121(time_s, timeVecOnePulse_s_(iChan), ampVec_nd_(iChan));
          break;

        case Doublet3211:
          exciteVec_nd(iChan) = SigDoublet3211(time_s, timeVecOnePulse_s_(iChan), ampVec_nd_(iChan));
          break;
      }

      exciteVecFlag(iChan) = 1;
    }
  }

  return exciteVecFlag.any();
}

// Pulse with 1-pulse timeDur_s long
float ExciteDisc::SigPulse(float &time_s, float &timeDur_s, float &amp_nd)
{
  float excite_nd;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 1-1 with 1-pulse timeDur_s long, total of 2*timeDur_s excitation
float ExciteDisc::SigDoublet(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float excite_nd;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur2_s) {
    excite_nd = -amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 1-2-1 - 1-pulse timeDur_s long, total of 4*timeDur_s excitation
float ExciteDisc::SigDoublet121(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float excite_nd;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s) {
    excite_nd = -amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s + timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 3-2-1-1 - 1-pulse timeDur_s long, total of 7*timeDur_s excitation
float ExciteDisc::SigDoublet3211(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float timeDur3_s = 3.0 * timeDur_s;
  float excite_nd;

  if (time_s < timeDur3_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s) {
    excite_nd = -amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s + timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s + timeDur_s + timeDur_s) {
    excite_nd = -amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}



/* Chirp Excitations */
ExciteChirp::ExciteChirp(void)
{
  
}

void ExciteChirp::SetParamChirp(kExciteChirp chirpType, VecChan timeVecStart_s, VecChan timeVecDur_s, VecChan freqVecStart_rps, VecChan freqVecEnd_rps, VecChan ampVecStart_nd, VecChan ampVecEnd_nd)
{
  chirpType_ = chirpType;
  numChan_ = timeVecStart_s.size();

  timeVecStart_s_ = timeVecStart_s;
  timeVecDur_s_ = timeVecDur_s;
  freqVecStart_rps_ = freqVecStart_rps;
  freqVecEnd_rps_ = freqVecEnd_rps;
  ampVecStart_nd_ = ampVecStart_nd;
  ampVecEnd_nd_ = ampVecEnd_nd;
}

int ExciteChirp::ComputeChirp(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;
  VecChanInt exciteVecFlag;

  exciteVec_nd.setZero(numChan_);
  exciteVecFlag.setZero(numChan_);

  for (int iChan = 0; iChan < numChan_; iChan++) {
    timeDur_s = timeVecDur_s_(iChan);
    time_s = timeCurr_s - timeVecStart_s_(iChan);

    if ((time_s >= 0) && (time_s < timeDur_s)) {

    switch (chirpType_){
      case Linear:
        exciteVec_nd(iChan) = SigChirpLinear(time_s, timeDur_s, freqVecStart_rps_(iChan), freqVecEnd_rps_(iChan), ampVecStart_nd_(iChan), ampVecEnd_nd_(iChan));

        break;
      }

      exciteVecFlag(iChan) = 1;
    }
  }

  return exciteVecFlag.any();
}

// Chirp (frequency Sweep), linear varying amplitude and frequency
float ExciteChirp::SigChirpLinear(float &time_s, float &timeDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_nd)
{
  // linear varying instantanious frequency
  float freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * timeDur_s) * time_s;

  // linear varying amplitude
  float amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * time_s / timeDur_s;

  // chirp Equation
  float excite_nd = amp_nd * sin(freq_rps * time_s);

  return excite_nd;
}


/* MultiSine Excitations */
ExciteMultisine::ExciteMultisine(void)
{
  
}

void ExciteMultisine::SetParamMultisine(kExciteMultisine multiSineType, VecChan timeVecStart_s, VecChan timeVecDur_s, MatChanElem freqMat_rps, MatChanElem phaseMat_rad, MatChanElem ampMat_nd)
{
  multiSineType_ = multiSineType;
  numChan_ = timeVecStart_s.size();

  timeVecStart_s_ = timeVecStart_s;
  timeVecDur_s_ = timeVecDur_s;
  freqMat_rps_ = freqMat_rps;
  phaseMat_rad_ = phaseMat_rad;
  ampMat_nd_ = ampMat_nd;
}

int ExciteMultisine::ComputeMultisine(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;
  VecChanInt exciteVecFlag;

  exciteVec_nd.setZero(numChan_);
  exciteVecFlag.setZero(numChan_);

  for (int iChan = 0; iChan < numChan_; iChan++) {
    timeDur_s = timeVecDur_s_(iChan);
    time_s = timeCurr_s - timeVecStart_s_(iChan);

    if ((time_s >= 0) && (time_s < timeDur_s)) {

      switch (multiSineType_){
        case OMS:
          VecElem freqList_rps = freqMat_rps_.row(iChan);
          VecElem phaseList_rad = phaseMat_rad_.row(iChan);
          VecElem ampList_nd = ampMat_nd_.row(iChan);

          exciteVec_nd(iChan) = SigMultisineOms(time_s, freqList_rps, phaseList_rad, ampList_nd);
          break;
      }

      exciteVecFlag(iChan) = 1;
    }
  }

  return exciteVecFlag.any();
}

// Optimal MultiSine
float ExciteMultisine::SigMultisineOms(float &time_s, VecElem &freqList_rps, VecElem &phaseList_rad, VecElem &ampList_nd)
{
  // Number of elements in the List
  int numElem = freqList_rps.size();

  // Scale the excitation to preserve unity
  float scale = sqrt(1.0 / numElem);

  // Compute the Excitation - scale * sum(amp .* cos(freq * time + phase))
  VecElem exciteList_nd = ampList_nd.array() * (freqList_rps * time_s + phaseList_rad).array().cos();
  float excite_nd = scale * exciteList_nd.sum();
 
  return excite_nd;
}
