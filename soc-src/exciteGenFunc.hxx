/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#ifndef EXCITEGEN_H
#define EXCITEGEN_H

#include <stdint.h>
#include <Eigen/Core>

#ifndef kMaxExciteChan
#define kMaxExciteChan 4
#endif

#ifndef kMaxExciteElem
#define kMaxExciteElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxExciteElem, 1> VecElem;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxExciteChan, kMaxExciteElem> MatChanElem;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxExciteChan, 1> VecChan;

typedef Eigen::Matrix<float, kMaxExciteChan, 1> VecChanLog;


enum ExciteDiscType {kPulse = 0, kDoublet = 1, kDoublet121 = 2, kDoublet3211 = 3};
enum ExciteChirpType {kLinear = 1};
enum ExciteMultisineType {kOMS = 1};


/* Discrete Excitations */
class ExciteDisc {
 public:
  float timeExcite_s_;

  ExciteDisc() {}; // Constructor
  ~ExciteDisc() {}; // Destructor
  void Init(ExciteDiscType discType, VecChan timeVecStart_s, VecChan timeVecOnePulse_s, VecChan ampVec_nd);
  VecChan Compute(float timeExcite_s);

 private:
  VecChan timeVecStart_s_, timeVecOnePulse_s_, timeVecDur_s_, ampVec_nd_;
  uint8_t numChan_;

  ExciteDiscType discType_;
};


/* Chirp Excitations */
class ExciteChirp {
 public:
  float timeExcite_s_;

  ExciteChirp() {}; // Constructor
  ~ExciteChirp() {}; // Destructor
  void Init(ExciteChirpType chirpType, VecChan timeVecStart_s, VecChan timeVecDur_s, VecChan freqVecStart_rps, VecChan freqVecEnd_rps, VecChan ampVecStart_nd, VecChan ampVecEnd_nd);
  VecChan Compute(float timeExcite_s);

 private:
  VecChan timeVecStart_s_, timeVecDur_s_;
  VecChan freqVecStart_rps_, freqVecEnd_rps_, ampVecStart_nd_, ampVecEnd_nd_;
  uint8_t numChan_;

  ExciteChirpType chirpType_;
};


/* MultiSine Excitations */
class ExciteMultisine {
 public:
  float timeExcite_s_;

  ExciteMultisineType multiSineType_;

  ExciteMultisine() {}; // Constructor
  ~ExciteMultisine() {}; // Destructor
  void Init(ExciteMultisineType multiSineType, VecChan timeVecStart_s, VecChan timeVecDur_s, MatChanElem freqMat_rps, MatChanElem phaseMat_rad, MatChanElem ampMat_nd);
  VecChan Compute(float timeExcite_s);

 private:
  VecChan timeVecStart_s_, timeVecDur_s_;
  MatChanElem freqMat_rps_, phaseMat_rad_, ampMat_nd_;
  uint8_t numChan_;

};


/* Single Channel Exication Signal Generators */
float SigPulse(float &time_s, float &timeOnePulse_s, float &amp_nd);
float SigDoublet(float &time_s, float &timeOnePulse_s, float &amp_nd);
float SigDoublet121(float &time_s, float &timeOnePulse_s, float &amp_nd);
float SigDoublet3211(float &time_s, float &timeOnePulse_s, float &amp_nd);
float SigChirpLinear(float &time_s, float &timeDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_rad);
float SigMultisineOms(float &time_s, VecElem &freqList_rps, VecElem &phaseList_rad, VecElem &ampList_nd);


#endif // EXCITEGEN_H