/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-14 - Chris Regan - Formed Classes for Discrete, Chirp, and OMS. Vectorized
*/

#ifndef EXCITEGEN_H
#define EXCITEGEN_H

#include <math.h>
#include <eigen3/Eigen/Dense>

#define MaxChan 16
#define MaxElem 30

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, MaxElem, 1> VecElem;
typedef Eigen::Matrix<float, -1, -1, 0, MaxChan, MaxElem> MatChanElem;
typedef Eigen::Matrix<float, -1, 1, 0, MaxChan, 1> VecChan;
typedef Eigen::Matrix<int, -1, 1, 0, MaxChan, 1> VecChanInt;


/* Discrete Excitations */
enum ExciteDiscType {Pulse = 0, Doublet = 1, Doublet121 = 2, Doublet3211 = 3};

class ExciteDisc {
private:
  VecChan timeVecStart_s, timeVecDur_s, ampVec_nd;
  int numChan;

  float GenPulse(float &time_s);
  float GenDoublet(float &time_s);
  float GenDoublet121(float &time_s);
  float GenDoublet3211(float &time_s);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  ExciteDiscType discType;

  // Constructor
  ExciteDisc (ExciteDiscType discType_, int numChan_) ;

  void SetParam (VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan ampVec_nd_);
  int Compute (VecChan timeCurr_s, VecChan &excite_nd);
};

ExciteDisc::ExciteDisc(ExciteDiscType discType_, int numChan_) {
  discType = discType_;
  numChan = numChan_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
};

void ExciteDisc::SetParam (VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan ampVec_nd_) {
  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  ampVec_nd = ampVec_nd_;
};

int ExciteDisc::Compute (float timeCurr_s, VecChan &exciteVec_nd)
{
  VecChan time_s;
  timeVec_s = timeDur_s.array() - timeCurr_s.array();

  float time_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    time_s = timeVec_s[iChan];
    exciteFlag[iChan] = 0;
    exciteVec_nd[iChan] = 0.0;

    timeDur_s = timeVecDur_s[iChan];
    amp_nd = ampVec_nd[iChan];

    if ((time_s >= 0) && (time_s < timeDur_s)) {

      switch (discType){
        case Pulse:
          exciteVec_nd[iChan] = GenPulse(time_s);
          break;

        case Doublet:
          exciteVec_nd[iChan] = GenDoublet(time_s);
          break;

        case Doublet121:
          exciteVec_nd[iChan] = GenDoublet121(time_s);
          break;

        case Doublet3211:
          exciteVec_nd[iChan] = GenDoublet3211(time_s);
          break;
      }

      exciteFlag[iChan] = 1;
    }
  }

  return exciteFlag.any();
};



/* Chirp Excitations */
enum ExciteChirpType {ConstAmp = 0, LinearAmp = 1};

class ExciteChirp {
private:
  VecChan timeVecStart_s, timeVecDur_s;
  VecChan freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd;
  int numChan;

  float GenChirpLinearAmp(float &time_s);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  ExciteChirpType chirpType;

  ExciteChirp(ExciteChirpType chirpType_, int numChan_);
  void SetParam(VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan freqVecStart_rps_, VecChan freqVecEnd_rps_, VecChan ampVecStart_nd_, VecChan ampVecEnd_nd_) ;
  int Compute(float timeCurr_s, VecChan &excite_nd);
};

ExciteChirp::ExciteChirp(ExciteChirpType chirpType_, int numChan_){
  chirpType = chirpType_;
  numChan = numChan_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
};

void ExciteChirp::SetParam(VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan freqVecStart_rps_, VecChan freqVecEnd_rps_, VecChan ampVecStart_nd_, VecChan ampVecEnd_nd_) {
  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  freqVecStart_rps = freqVecStart_rps_;
  freqVecEnd_rps = freqVecEnd_rps_;
  ampVecStart_nd = ampVecStart_nd_;
  ampVecEnd_nd = ampVecEnd_nd_;
};

int ExciteChirp::Compute (float timeCurr_s, VecChan &exciteVec_nd)
{
  VecChan time_s;
  timeVec_s = timeDur_s.array() - timeCurr_s.array();

  float time_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    time_s = timeVec_s[iChan];
    exciteFlag[iChan] = 0;
    exciteVec_nd[iChan] = 0.0;

    timeDur_s = timeVecDur_s[iChan];
    amp_nd = ampVec_nd[iChan];

    if ((time_s >= 0) && (time_s < timeDur_s)) {

    switch (chirpType){
      case LinearAmp:
        exciteVec_nd[iChan] = GenChirpLinearAmp(time_s[iChan], timeDur_s[iChan], freqStart_rps[iChan], freqEnd_rps[iChan], ampStart_nd[iChan], ampEnd_nd[iChan]);

        break;
      }

      exciteFlag[iChan] = 1;
    }
  }

  return exciteFlag.any();
};


/* MultiSine Excitations */
enum ExciteMultiSineType {OMS = 1};

class ExciteMultiSine {
private:
  VecChan timeVecStart_s, timeVecDur_s;
  MatChanElem freqMat_rps, phaseMat_rad, ampMat_nd;
  int numChan;

  float GenMultiSineOms(float &time_s);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  ExciteMultiSineType multiSineType;

  // Constructor
  ExciteMultiSine (ExciteMultiSineType multiSineType_, int numChan_);

  void SetParam (VecChan timeVecStart_s_, VecChan timeVecDur_s_, MatChanElem freqMat_rps_, MatChanElem phaseMat_rad_, MatChanElem ampMat_nd_);

  int Compute (float timeCurr_s, VecChan &exciteVec_nd);
};


// Constructor
ExciteMultisine::ExciteMultiSine (ExciteMultiSineType multiSineType_, int numChan_) {
  multiSineType = multiSineType_;
  numChan = numChan_;

  exciteVec_nd.Zero(numChan);
  exciteVecFlag.Zero(numChan);
};

void ExciteMultisine::SetParam (VecChan timeVecStart_s_, VecChan timeVecDur_s_, MatChanElem freqMat_rps_, MatChanElem phaseMat_rad_, MatChanElem ampMat_nd_) {
  timeVecStart_s = timeVecStart_s_;
  timeVecDur_s = timeVecDur_s_;
  freqMat_rps = freqMat_rps_;
  phaseMat_rad = phaseMat_rad_;
  ampMat_nd = ampMat_nd_;
};

int ExciteMultisine::Compute (float timeCurr_s, VecChan &exciteVec_nd)
{
  VecChan time_s;
  timeVec_s = timeDur_s.array() - timeCurr_s.array();

  float time_s;

  for (int iChan = 0; iChan < numChan; iChan++) {
    time_s = timeVec_s[iChan];
    exciteFlag[iChan] = 0;
    exciteVec_nd[iChan] = 0.0;

    timeDur_s = timeVecDur_s[iChan];
    amp_nd = ampVec_nd[iChan];

    if ((time_s >= 0) && (time_s < timeDur_s)) {

      switch (multiSineType){
        case OMS:
          exciteVec_nd[iChan] = GenMuliSineOms(time_s[iChan], freq_rps[iChan], phase_rad[iChan], amp_nd[iChan]);
          break;
      }

      exciteFlag[iChan] = 1;
    }
  }

  return exciteFlag.any();
};


//
// Single Channel Excitation Signal Generators
//
// Pulse with 1-pulse timeDur_s long
float ExciteDisc::GenPulse(float &time_s)
{
  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
};

// Doublet 1-1 with 1-pulse timeDur_s long, total of 2*timeDur_s excitation
float ExciteDisc::GenDoublet(float &time_s)
{
  float timeDur2_s = 2.0 * timeDur_s;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur2_s) {
    excite_nd = -amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
};

// Doublet 1-2-1 - 1-pulse timeDur_s long, total of 4*timeDur_s excitation
float ExciteDisc::GenDoublet121(float &time_s)
{
  float timeDur2_s = 2.0 * timeDur_s;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s) {
    excite_nd = -amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s + timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
};

// Doublet 3-2-1-1 - 1-pulse timeDur_s long, total of 7*timeDur_s excitation
float ExciteDisc::GenDoublet3211(float &time_s)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float timeDur3_s = 3.0 * timeDur_s;

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
};

// Chirp (frequency Sweep), linear varying amplitude and frequency
float ExciteChirp::GenChirpLinearAmp(float &time_s)
{
  float freq_rps;    // [rps], frequency, linearly changing with time
  float amp_nd = 0.0;  // [nd], amplitude, linearly changing with time
  
  // linear varying instantanious frequency
  freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * timeDur_s) * time_s;

  // linear varying amplitude
  amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * time_s / timeDur_s;

  // chirp Equation
  excite_nd = amp_nd * sin(freq_rps * time_s);

  return excite_nd;
};

// Optimal MultiSine
float ExciteMultisine::GenMultiSineOms(float &time_s)
{
  // Compute the Excitation - scale * amp .* sum(cos(freq * time + phase))
  excite_nd = ampVec_nd.array() * (freqVec_rps.array() * time_s.array() + phaseVec_rad.array()).cos().array();
 
  // Scale the excitation to preserve unity
  float scale = sqrt(1.0 / numElem);
  excite_nd *= scale; // Apply scaling

  return excite_nd;
};

#endif // EXCITEGEN_H