/*
Classes and Functions for Control Allocation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#ifndef CNTRLALLOC_H
#define CNTRLALLOC_H

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>
//#include <Eigen/QR>

#ifndef kMaxAllocObj
#define kMaxAllocObj 5
#endif

#ifndef kMaxAllocEff
#define kMaxAllocEff 16
#endif

#include <stdint.h>

// Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options = 0, int MaxRowsAtCompileTime = RowsAtCompileTime, int MaxColsAtCompileTime = ColsAtCompileTime>
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocObj, kMaxAllocEff> MatCntrlEff;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocObj, 1> VecObj;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocEff, 1> VecEff;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocObj, kMaxAllocObj> MatObj;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocEff, kMaxAllocEff> MatEff;
typedef Eigen::Matrix<float, -1, -1, 0, kMaxAllocEff, kMaxAllocObj> MatCntrlEffT;

typedef Eigen::Matrix<uint8_t, -1, 1, 0, kMaxAllocEff, 1> VecEffInt;

#define kMaxSolvObj kMaxAllocObj // FIXIT - This is method dependent, may want to just allow dynamic sizing
#define kMaxSolvEff kMaxAllocEff // FIXIT - This is method dependent, may want to just allow dynamic sizing

typedef Eigen::Matrix<float, -1, -1, 0, kMaxSolvObj, kMaxSolvEff> MatSolv;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxSolvObj, 1> VecSolvObj;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxSolvEff, 1> VecSolvEff;

// Enumerated list of Control Allocation Methods
enum CntrlAllocMethod { kPseudo = 10, kPseudoWt = 15, kPseudoRedis = 20, kPseudoRedisScale = 25, kFxp = 50, kMolp = 60, kMoqp = 70, kSqp = 75};

// 
VecEff CntrlAlloc(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref,
  CntrlAllocMethod method);

// Allocation Methods
VecEff CntrlAllocPseudo(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uPref);

VecEff CntrlAllocPseudoWt(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref);

VecEff CntrlAllocFxp(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref,
  float gammaEff = 0.001, uint8_t numIter = 200);

VecEff CntrlAllocPseudoRedis(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref);

VecEff CntrlAllocPseudoRedisScale(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref);

VecEff CntrlAllocMolp(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref,
  float gammaEff = 0.001);

VecEff CntrlAllocMoqp(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref,
  float gammaEff = 0.001);

VecEff CntrlAllocSqp(
  const MatCntrlEff& cntrlEff, const VecObj& vObj, const VecEff& uMin, const VecEff& uMax, const MatObj& wtObj, const MatEff& wtEff, const VecEff& uPref);


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
  VecEffInt& iEffSat, VecEff& x,
  float tol = 1.0E-8);

void QuadProgPhase2(
  const MatSolv& A1, const VecSolvObj& b1, const MatSolv& A2, const VecSolvObj& b2, const VecSolvEff& xMin, const VecSolvEff& xMax,
  VecEffInt& iEffSat, VecEff& x,
  float tol = 1.0E-8);
*/

// General Functions
void Saturate(
  const VecEff& uMin, const VecEff& uMax,
  VecEff& uCmd);

uint8_t SaturateIndex(
  const VecEff& uMin, const VecEff& uMax,
  VecEff& uCmd, VecEffInt& iEffSat);

void FindFree(
  const VecEffInt& iEffSat,
  VecEffInt& iEffFree);

void ShrinkMatColumns(
  const MatCntrlEff& M, const VecEffInt& elemKeep,
  MatCntrlEff& MShrink);

void ShrinkVecElem(
  const VecEff& V, const VecEffInt& elemKeep,
  VecEff& VShrink);

void ExpandMatColumns(
  const MatCntrlEff& M, const VecEffInt& elemKeep,
  MatCntrlEff& MExp);

void ExpandVecElem(
  const VecEff& V, const VecEffInt& elemKeep,
  VecEff& VExp);


#endif // CNTRLALLOC_H