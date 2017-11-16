/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#include "exciteGenFunc.hxx"

/* Discrete Excitations */
ExciteDisc::ExciteDisc(void)
{

}

void ExciteDisc::SetParamDisc(ExciteDiscType discType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan ampVec_nd_)
{
  discType = discType_;
  numChan = timeVecStart_s_.size();

  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  ampVec_nd = ampVec_nd_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
}

int ExciteDisc::ComputeDisc(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    timeDur_s = timeVecDur_s[iChan];
    time_s = timeCurr_s - timeVecStart_s[iChan];

    exciteVec_nd[iChan] = 0.0;

    float timeEnd_s = 0.0;
    switch (discType){
      case Pulse: timeEnd_s = 1*timeDur_s;
      case Doublet: timeEnd_s = 2*timeDur_s;
      case Doublet121: timeEnd_s = 4*timeDur_s;
      case Doublet3211: timeEnd_s = 7*timeDur_s;
    }

    if ((time_s >= 0) && (time_s < timeEnd_s)) {

      switch (discType){
        case Pulse:
          exciteVec_nd[iChan] = SigPulse(time_s, timeDur_s, ampVec_nd[iChan]);
          break;

        case Doublet:
          exciteVec_nd[iChan] = SigDoublet(time_s, timeDur_s, ampVec_nd[iChan]);
          break;

        case Doublet121:
          exciteVec_nd[iChan] = SigDoublet121(time_s, timeDur_s, ampVec_nd[iChan]);
          break;

        case Doublet3211:
          exciteVec_nd[iChan] = SigDoublet3211(time_s, timeDur_s, ampVec_nd[iChan]);
          break;
      }

      //exciteVecFlag[iChan] = 1;
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

void ExciteChirp::SetParamChirp(ExciteChirpType chirpType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan freqVecStart_rps_, VecChan freqVecEnd_rps_, VecChan ampVecStart_nd_, VecChan ampVecEnd_nd_)
{
  chirpType = chirpType_;
  numChan = timeVecStart_s_.size();

  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  freqVecStart_rps = freqVecStart_rps_;
  freqVecEnd_rps = freqVecEnd_rps_;
  ampVecStart_nd = ampVecStart_nd_;
  ampVecEnd_nd = ampVecEnd_nd_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
}

int ExciteChirp::ComputeChirp(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    timeDur_s = timeVecDur_s[iChan];
    time_s = timeCurr_s - timeVecStart_s[iChan];
    
    exciteVec_nd[iChan] = 0.0;

    if ((time_s >= 0) && (time_s < timeDur_s)) {

    switch (chirpType){
      case Linear:
        exciteVec_nd[iChan] = SigChirpLinear(time_s, timeDur_s, freqVecStart_rps[iChan], freqVecEnd_rps[iChan], ampVecStart_nd[iChan], ampVecEnd_nd[iChan]);

        break;
      }

      //exciteVecFlag[iChan] = 1;
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

void ExciteMultisine::SetParamMultisine(ExciteMultisineType multiSineType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, MatChanElem freqMat_rps_, MatChanElem phaseMat_rad_, MatChanElem ampMat_nd_)
{
  multiSineType = multiSineType_;
  numChan = timeVecStart_s_.size();

  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  freqMat_rps = freqMat_rps_;
  phaseMat_rad = phaseMat_rad_;
  ampMat_nd = ampMat_nd_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
}

int ExciteMultisine::ComputeMultisine(float timeCurr_s, VecChan &exciteVec_nd)
{
  float time_s, timeDur_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    timeDur_s = timeVecDur_s[iChan];
    time_s = timeCurr_s - timeVecStart_s[iChan];

    exciteVec_nd[iChan] = 0.0;

    if ((time_s >= 0) && (time_s < timeDur_s)) {

      switch (multiSineType){
        case OMS:
          VecElem freqList_rps = freqMat_rps.row(iChan);
          VecElem phaseList_rad = phaseMat_rad.row(iChan);
          VecElem ampList_nd = ampMat_nd.row(iChan);

          exciteVec_nd(iChan) = SigMultisineOms(time_s, freqList_rps, phaseList_rad, ampList_nd);
          break;
      }

      //exciteVecFlag[iChan] = 1;
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
