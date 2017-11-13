/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include "exciteGenFunc.hxx"

// Step
int ExciteStep(const float &timeCurr_s, const float &timeStart_s, const float &amp_nd, float &excite_nd)
{
  excite_nd = 0.0;  // [nd], excitation command

  if (timeCurr_s >= timeStart_s) {
    excite_nd = amp_nd;
    return 1;
  }
  return 0;
}

// Pulse with 1-pulse timeDur_s long
int ExcitePulse(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const float &amp_nd, float &excite_nd)
{  
  excite_nd = 0.0;  // [nd], excitation command

  if ((timeCurr_s >= timeStart_s) && (timeCurr_s < timeStart_s + timeDur_s)) {
    excite_nd = amp_nd;
    return 1;
  }
  return 0;
}

// Doublet 1-1 with 1-pulse timeDur_s long, total of 2*timeDur_s excitation
int ExciteDoublet(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const float &amp_nd, float &excite_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  
  excite_nd = 0.0;  // [nd], excitation command

  if ((timeCurr_s >= timeStart_s) && (timeCurr_s < timeStart_s + timeDur2_s)) {
    if (timeCurr_s < timeStart_s + timeDur_s) {
      excite_nd = amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur2_s) {
      excite_nd = -amp_nd;
      return 1;
    }
    return 1;
  }
  return 0;
}

// Doublet 1-2-1 - 1-pulse timeDur_s long, total of 4*timeDur_s excitation
int ExciteDoublet121(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const float &amp_nd, float &excite_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  
  excite_nd = 0.0;  // [nd], excitation command

  if ((timeCurr_s >= timeStart_s) && (timeCurr_s < timeStart_s + 4.0*timeDur_s)) {
    if (timeCurr_s < timeStart_s + timeDur_s) {
      excite_nd = amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur_s + timeDur2_s) {
      excite_nd = -amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur_s + timeDur2_s + timeDur_s) {
      excite_nd = amp_nd;
      return 1;
    }
    return 1;
  }
  return 0;
}

// Doublet 3-2-1-1 - 1-pulse timeDur_s long, total of 7*timeDur_s excitation
int ExciteDoublet3211(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const float &amp_nd, float &excite_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float timeDur3_s = 3.0 * timeDur_s;
  
  excite_nd = 0.0;  // [nd], excitation command

  if ((timeCurr_s >= timeStart_s) && (timeCurr_s < timeStart_s + 7.0*timeDur_s)) {
    if (timeCurr_s < timeStart_s + timeDur3_s) {
      excite_nd = amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur3_s + timeDur2_s) {
      excite_nd = -amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur3_s + timeDur2_s + timeDur_s) {
      excite_nd = amp_nd;
      return 1;
    } else if (timeCurr_s < timeStart_s + timeDur3_s + timeDur2_s + timeDur_s + timeDur_s) {
      excite_nd = -amp_nd;
      return 1;
    }
    return 1;
  }
  return 0;
}

// Chirp (frequency Sweep), linear varying amplitude and frequency
int ExciteChirp(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const float &freqStart_rps, const float &freqEnd_rps, const float &ampStart_nd, const float &ampEnd_nd, float &excite_nd)
{
  float freq_rps;    // [rps], frequency, linearly changing with time
  float amp_nd = 0.0;  // [nd], amplitude, linearly changing with time

  excite_nd = 0.0;  // [nd], excitation command
  
  if ((timeCurr_s >= timeStart_s) && (timeCurr_s <= timeStart_s + timeDur_s)) {
    // linear varying instantanious frequency
    freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * timeDur_s) * (timeCurr_s - timeStart_s);

    // linear varying amplitude
    amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * (timeCurr_s - timeStart_s) / timeDur_s;

    // chirp Equation
    excite_nd = amp_nd * sin(freq_rps * (timeCurr_s - timeStart_s));

    return 1;
  }
  return 0;
}

// Mulitchannel Chirp (frequency Sweep), linear varying amplitude and frequency
int ExciteMultiChirp(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const VectorExcite &freqVecStart_rps, const VectorExcite &freqVecEnd_rps, const VectorExcite &ampVecStart_nd, const VectorExcite &ampVecEnd_nd, VectorExcite &exciteVec_nd)
{
  int numChan = freqVecStart_rps.size(); // Number of channels
  
  exciteVec_nd.fill(0.0);  // [nd], excitation vector command

  int exciteFlag = 0;

  // Loop over each channel
  for (int iChan = 0 ; iChan < numChan ; iChan++) {

    exciteFlag = ExciteChirp(timeCurr_s, timeStart_s, timeDur_s,
      freqVecStart_rps(iChan), freqVecEnd_rps(iChan), ampVecStart_nd(iChan), ampVecEnd_nd(iChan),
      exciteVec_nd(iChan));
  }
  return exciteFlag;

}

// Optimal MultiSine
int ExciteOms(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const VectorElem &freqVec_rps, const VectorElem &phaseVec_rad, const VectorElem &ampVec_nd, float &excite_nd)
{
  int numElem = freqVec_rps.size();  // Length of input vectors
  excite_nd = 0.0;  // [nd], excitation command

  float scale = sqrt(1.0 / numElem);

  if ((timeCurr_s >= timeStart_s) && (timeCurr_s <= timeStart_s + timeDur_s)) {
    // Loop over each element
    for (int iElem = 0 ; iElem < numElem ; iElem++) {
      excite_nd += scale * ampVec_nd[iElem] * cos(freqVec_rps[iElem] * (timeCurr_s - timeStart_s) + phaseVec_rad[iElem]);
    }
    return 1;
  }
  return 0;
}

// Multichannel Optimal MultiSine
int ExciteMultiOms(const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s, const MatrixElem &freqMat_rps, const MatrixElem &phaseMat_rad, const MatrixElem &ampMat_nd, VectorExcite &exciteVec_nd)
{
  int numChan = freqMat_rps.rows(); // Number of channels
  
  exciteVec_nd.fill(0.0);  // [nd], excitation vector command

  int exciteFlag = 0;

  // Loop over each channel
  for (int iChan = 0; iChan < numChan; iChan++) {

    exciteFlag = ExciteOms(timeCurr_s, timeStart_s, timeDur_s,
      freqMat_rps.row(iChan), phaseMat_rad.row(iChan), ampMat_nd.row(iChan),
      exciteVec_nd(iChan));
  }
  return exciteFlag;
}