/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#ifndef EXCITEGEN_H
#define EXCITEGEN_H

#include <math.h>
#include <eigen3/Eigen/Dense>

#define MaxChan 16
#define MaxElem 30

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, MaxElem, 1> VectorElem;
typedef Eigen::Matrix<float, -1, -1, 0, MaxChan, MaxElem> MatrixElem;
typedef Eigen::Matrix<float, -1, 1, 0, MaxChan, 1> VectorExcite;

int ExciteStep(
  const float &timeCurr_s, const float &timeStart_s,
  const float &amp_nd,
  float &excite_nd
  );

int ExcitePulse(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const float &amp_nd,
  float &excite_nd
  );

int ExciteDoublet(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const float &amp_nd,
  float &excite_nd
  );

int ExciteDoublet121(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const float &amp_nd,
  float &excite_nd
  );

int ExciteDoublet3211(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const float &amp_nd,
  float &excite_nd
  );

int ExciteChirp(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const float &freqStart_rps, const float &freqEnd_rps, const float &ampStart_nd, const float &ampEnd_nd,
  float &excite_nd
  );

int ExciteMultiChirp(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const VectorExcite &freqVecStart_rps, const VectorExcite &freqVecEnd_rps, const VectorExcite &ampVecStart_nd, const VectorExcite &ampVecEnd_nd,
  VectorExcite &exciteVec_nd
  );

int ExciteOms(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const VectorElem &freqVec_rps, const VectorElem &phaseVec_rad, const VectorElem &ampVec_nd,
  float &excite_nd
  );

int ExciteMultiOms(
  const float &timeCurr_s, const float &timeStart_s, const float &timeDur_s,
  const MatrixElem &freqMat_rps, const MatrixElem &phaseMat_rad, const MatrixElem &ampMat_nd,
  VectorExcite &exciteVec_nd
  );

#endif // EXCITEGEN_H