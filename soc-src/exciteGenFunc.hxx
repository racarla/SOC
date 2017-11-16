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


enum kExciteDisc {Pulse = 0, Doublet = 1, Doublet121 = 2, Doublet3211 = 3};
enum kExciteChirp {Linear = 1};
enum kExciteMultisine {OMS = 1};

/* Discrete Excitations */
class ExciteDisc {
 public:
  float timeCurr_s_;

  // Constructor
  ExciteDisc();
  void SetParamDisc(kExciteDisc discType, VecChan timeVecStart_s, VecChan timeVecOnePulse_s, VecChan ampVec_nd);
  int ComputeDisc(float timeCurr_s, VecChan &exciteVec_nd);

 private:
  VecChan timeVecStart_s_, timeVecOnePulse_s_, timeVecDur_s_, ampVec_nd_;
  int numChan_;

  kExciteDisc discType_;

  float SigPulse(float &time_s, float &timeOnePulse_s, float &amp_nd);
  float SigDoublet(float &time_s, float &timeOnePulse_s, float &amp_nd);
  float SigDoublet121(float &time_s, float &timeOnePulse_s, float &amp_nd);
  float SigDoublet3211(float &time_s, float &timeOnePulse_s, float &amp_nd);
};


/* Chirp Excitations */
class ExciteChirp {
 public:
  float timeCurr_s_;

  ExciteChirp();
  void SetParamChirp(kExciteChirp chirpType, VecChan timeVecStart_s, VecChan timeVecDur_s, VecChan freqVecStart_rps, VecChan freqVecEnd_rps, VecChan ampVecStart_nd, VecChan ampVecEnd_nd);
  int ComputeChirp(float timeCurr_s, VecChan &excite_nd);

 private:
  VecChan timeVecStart_s_, timeVecDur_s_;
  VecChan freqVecStart_rps_, freqVecEnd_rps_, ampVecStart_nd_, ampVecEnd_nd_;
  int numChan_;

  kExciteChirp chirpType_;

  float SigChirpLinear(float &time_s, float &timeDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_rad);
};


/* MultiSine Excitations */
class ExciteMultisine {
 public:
  float timeCurr_s_;

  kExciteMultisine multiSineType_;

  ExciteMultisine();
  void SetParamMultisine(kExciteMultisine multiSineType, VecChan timeVecStart_s, VecChan timeVecDur_s, MatChanElem freqMat_rps, MatChanElem phaseMat_rad, MatChanElem ampMat_nd);
  int ComputeMultisine(float timeCurr_s, VecChan &exciteVec_nd);

 private:
  VecChan timeVecStart_s_, timeVecDur_s_;
  MatChanElem freqMat_rps_, phaseMat_rad_, ampMat_nd_;
  int numChan_;

  float SigMultisineOms(float &time_s, VecElem &freqList_rps, VecElem &phaseList_rad, VecElem &ampList_nd);

};


#endif // EXCITEGEN_H