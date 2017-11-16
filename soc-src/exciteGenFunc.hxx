/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#ifndef EXCITEGEN_H
#define EXCITEGEN_H

#include <eigen3/Eigen/Dense>

#define MaxChan 16
#define MaxElem 30

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, MaxElem, 1> VecElem;
typedef Eigen::Matrix<float, -1, -1, 0, MaxChan, MaxElem> MatChanElem;
typedef Eigen::Matrix<float, -1, 1, 0, MaxChan, 1> VecChan;
typedef Eigen::Matrix<int, -1, 1, 0, MaxChan, 1> VecChanInt;


enum ExciteDiscType {Pulse = 0, Doublet = 1, Doublet121 = 2, Doublet3211 = 3};
enum ExciteChirpType {Linear = 1};
enum ExciteMultisineType {OMS = 1};

/* Discrete Excitations */
class ExciteDisc {
private:
  VecChan timeVecStart_s, timeVecDur_s, ampVec_nd;
  int numChan;

  ExciteDiscType discType;

  float SigPulse(float &time_s, float &timeDur_s, float &amp_nd);
  float SigDoublet(float &time_s, float &timeDur_s, float &amp_nd);
  float SigDoublet121(float &time_s, float &timeDur_s, float &amp_nd);
  float SigDoublet3211(float &time_s, float &timeDur_s, float &amp_nd);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  // Constructor
  ExciteDisc();
  void SetParamDisc(ExciteDiscType discType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan ampVec_nd_);
  int ComputeDisc(float timeCurr_s, VecChan &exciteVec_nd);
};


/* Chirp Excitations */
class ExciteChirp {
private:
  VecChan timeVecStart_s, timeVecDur_s;
  VecChan freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd;
  int numChan;

  ExciteChirpType chirpType;

  float SigChirpLinear(float &time_s, float &timeDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_rad);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  ExciteChirp();
  void SetParamChirp(ExciteChirpType chirpType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, VecChan freqVecStart_rps_, VecChan freqVecEnd_rps_, VecChan ampVecStart_nd_, VecChan ampVecEnd_nd_);
  int ComputeChirp(float timeCurr_s, VecChan &excite_nd);
};


/* MultiSine Excitations */
class ExciteMultisine {
private:
  VecChan timeVecStart_s, timeVecDur_s;
  MatChanElem freqMat_rps, phaseMat_rad, ampMat_nd;
  int numChan;

  float SigMultisineOms(float &time_s, VecElem &freqList_rps, VecElem &phaseList_rad, VecElem &ampList_nd);

public:
  float timeCurr_s;
  VecChan exciteVec_nd;
  VecChanInt exciteVecFlag;

  ExciteMultisineType multiSineType;

  ExciteMultisine();
  void SetParamMultisine(ExciteMultisineType multiSineType_, VecChan timeVecStart_s_, VecChan timeVecDur_s_, MatChanElem freqMat_rps_, MatChanElem phaseMat_rad_, MatChanElem ampMat_nd_);
  int ComputeMultisine(float timeCurr_s, VecChan &exciteVec_nd);
};


#endif // EXCITEGEN_H