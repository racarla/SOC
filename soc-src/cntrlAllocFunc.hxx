/*
Classes and Functions for Control Allocation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#ifndef CNTRLALLOC_H
#define CNTRLALLOC_H

#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>
//#include <Eigen/QR>

#ifndef kMaxAllocObj
#define kMaxAllocObj 3
#endif

#ifndef kMaxAllocEff
#define kMaxAllocEff 6
#endif

// Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options = 0, int MaxRowsAtCompileTime = RowsAtCompileTime, int MaxColsAtCompileTime = ColsAtCompileTime>
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocObj, kMaxAllocEff> MatCntrlEff;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocObj, 1> VecAllocObj;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocEff, 1> VecAllocEff;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocObj, kMaxAllocObj> MatObj;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocEff, kMaxAllocEff> MatEff;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocEff, kMaxAllocObj> MatCntrlEffT;

typedef Eigen::Matrix<uint8_t, -1, 1, 0, kMaxAllocEff, 1> VecAllocEffInt;

typedef Eigen::Matrix<float, kMaxAllocEff, 1> VecAllocEffLog;

#define kMaxSolvObj kMaxAllocObj // FIXIT - This is method dependent, may want to just allow dynamic sizing
#define kMaxSolvEff kMaxAllocEff // FIXIT - This is method dependent, may want to just allow dynamic sizing

typedef Eigen::Matrix<float, -1, -1, 0, kMaxSolvObj, kMaxSolvEff> MatSolv;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxSolvObj, 1> VecSolvObj;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxSolvEff, 1> VecSolvEff;

// Enumerated list of Control Allocation Methods
enum CntrlAllocMethod { kPseudo = 10, kPseudoWt = 15, kPseudoRedis = 20, kPseudoRedisScale = 25, kFxp = 50, kMolp = 60, kMoqp = 70, kSqp = 75};

// 
VecAllocEff CntrlAlloc(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref,
  CntrlAllocMethod method);

// Allocation Methods
VecAllocEff CntrlAllocPseudo(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uPref);

VecAllocEff CntrlAllocPseudoWt(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref);

VecAllocEff CntrlAllocFxp(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref,
  float gammaEff = 0.001, uint8_t numIter = 200);

VecAllocEff CntrlAllocPseudoRedis(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref);

VecAllocEff CntrlAllocPseudoRedisScale(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref);

VecAllocEff CntrlAllocMolp(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref,
  float gammaEff = 0.001);

VecAllocEff CntrlAllocMoqp(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref,
  float gammaEff = 0.001);

VecAllocEff CntrlAllocSqp(
  const MatCntrlEff& cntrlEff, const VecAllocObj& vObj, const VecAllocEff& uMin, const VecAllocEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecAllocEff& uPref);


// Pseudo-Inverse Solvers
void SolvPinv(
  const MatSolv& A, const VecSolvObj& b,
  VecSolvEff& x);

void SolvPinvLU(
  const MatSolv& A, const VecSolvObj& b,
  VecSolvEff& x);

void SolvPinvCholesky(
  const MatSolv& A, const VecSolvObj& b,
  VecSolvEff& x);

void SolvPinvSVD(
  const MatSolv& A, const VecSolvObj& b,
  VecSolvEff& x);

void SolvPinvQR(
  const MatSolv& A, const VecSolvObj& b,
  VecSolvEff& x);


// Quadratic Programming Solvers
/*void QuadProgPhase1(
  const MatSolv& A, const VecSolvObj& b, const VecSolvEff& xMin, const VecSolvEff& xMax,
  VecAllocEffInt& iEffSat, VecAllocEff& x,
  float tol = 1.0E-8);

void QuadProgPhase2(
  const MatSolv& A1, const VecSolvObj& b1, const MatSolv& A2, const VecSolvObj& b2, const VecSolvEff& xMin, const VecSolvEff& xMax,
  VecAllocEffInt& iEffSat, VecAllocEff& x,
  float tol = 1.0E-8);
*/

// General Functions
void Saturate(
  const VecAllocEff& uMin, const VecAllocEff& uMax,
  VecAllocEff& uCmd);

uint8_t SaturateIndex(
  const VecAllocEff& uMin, const VecAllocEff& uMax,
  VecAllocEff& uCmd, VecAllocEffInt& iEffSat);

void FindFree(
  const VecAllocEffInt& iEffSat,
  VecAllocEffInt& iEffFree);

void ShrinkMatColumns(
  const MatCntrlEff& M, const VecAllocEffInt& elemKeep,
  MatCntrlEff& MShrink);

void ShrinkVecElem(
  const VecAllocEff& V, const VecAllocEffInt& elemKeep,
  VecAllocEff& VShrink);

void ExpandMatColumns(
  const MatCntrlEff& M, const VecAllocEffInt& elemKeep,
  MatCntrlEff& MExp);

void ExpandVecElem(
  const VecAllocEff& V, const VecAllocEffInt& elemKeep,
  VecAllocEff& VExp);


#endif // CNTRLALLOC_H