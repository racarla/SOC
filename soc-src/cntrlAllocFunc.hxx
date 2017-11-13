/*
Classes and Functions for Control Allocation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#ifndef CNTRLALLOC_H
#define CNTRLALLOC_H

#include <eigen3/Eigen/Dense>

#define MaxObj 5
#define MaxEff 16

// Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options = 0, int MaxRowsAtCompileTime = RowsAtCompileTime, int MaxColsAtCompileTime = ColsAtCompileTime>
typedef Eigen::Matrix<float, -1, -1, 0, MaxObj, MaxEff> MatCntrlEff;
typedef Eigen::Matrix<float, -1, 1, 0, MaxObj, 1> VecObj;
typedef Eigen::Matrix<float, -1, 1, 0, MaxEff, 1> VecEff;
typedef Eigen::Matrix<float, -1, -1, 0, MaxObj, MaxObj> MatObj;
typedef Eigen::Matrix<float, -1, -1, 0, MaxEff, MaxEff> MatEff;
typedef Eigen::Matrix<float, -1, -1, 0, MaxEff, MaxObj> MatCntrlEffT;

typedef Eigen::Matrix<int, -1, 1, 0, MaxEff, 1> VecEffInt;

#define MaxSolvObj MaxObj
#define MaxSolvEff MaxEff

typedef Eigen::Matrix<float, -1, -1, 0, MaxSolvObj, MaxSolvEff> MatSolv;
typedef Eigen::Matrix<float, -1, 1, 0, MaxSolvObj, 1> VecSolvObj;
typedef Eigen::Matrix<float, -1, 1, 0, MaxSolvEff, 1> VecSolvEff;

// Enumerated list of Control Allocation Methods
enum CntrlAllocMethod { Pseudo = 10, PseudoWt = 15, PseudoRedis = 20, PseudoRedisScale = 25, Fxp = 50, Molp = 60, Moqp = 70, Sqp = 75};

// 
void CntrlAlloc(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd,
  CntrlAllocMethod method);

// Allocation Methods
void CntrlAllocPseudo(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uPref,
  VecEff &uCmd);

void CntrlAllocPseudoWt(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref, 
  VecEff &uCmd);

void CntrlAllocFxp(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd, 
  float gammaEff = 0.001, int numIter = 200);

void CntrlAllocPseudoRedis(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd);

void CntrlAllocPseudoRedisScale(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd);

void CntrlAllocMolp(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd,
  float gammaEff = 0.001);

void CntrlAllocMoqp(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd,
  float gammaEff = 0.001);

void CntrlAllocSqp(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wtObj, const MatEff &wtEff, const VecEff &uPref,
  VecEff &uCmd);


// Pseudo-Inverse Solvers
void SolvPinv(
  const MatSolv &A, const VecSolvObj &b,
  VecSolvEff &x);

void SolvPinvLU(
  const MatSolv &A, const VecSolvObj &b,
  VecSolvEff &x);

void SolvPinvCholesky(
  const MatSolv &A, const VecSolvObj &b,
  VecSolvEff &x);

void SolvPinvSVD(
  const MatSolv &A, const VecSolvObj &b,
  VecSolvEff &x);

void SolvPinvQR(
  const MatSolv &A, const VecSolvObj &b,
  VecSolvEff &x);


// Quadratic Programming Solvers
void QuadProgPhase1(
  const MatSolv &A, const VecSolvObj &b, const VecSolvEff &xMin, const VecSolvEff &xMax,
  VecEffInt &iEffSat, VecEff &x,
  float tol = 1.0E-8);

void QuadProgPhase2(
  const MatSolv &A1, const VecSolvObj &b1, const MatSolv &A2, const VecSolvObj &b2, const VecSolvEff &xMin, const VecSolvEff &xMax,
  VecEffInt &iEffSat, VecEff &x,
  float tol = 1.0E-8);


// General Functions
void Saturate(
  const VecEff &uMin, const VecEff &uMax,
  VecEff &uCmd);

int SaturateIndex(
  const VecEff &uMin, const VecEff &uMax,
  VecEff &uCmd, VecEffInt &iEffSat);

int FindFree(
  const VecEffInt &iEffSat,
  VecEffInt &iEffFree);

void ShrinkMatColumns(
  const MatCntrlEff &M, const VecEffInt &elemKeep,
  MatCntrlEff &MShrink);

void ShrinkVecElem(
  const VecEff &V, const VecEffInt &elemKeep,
  VecEff &VShrink);

void ExpandMatColumns(
  const MatCntrlEff &M, const VecEffInt &elemKeep,
  MatCntrlEff &MExp);

void ExpandVecElem(
  const VecEff &V, const VecEffInt &elemKeep,
  VecEff &VExp);


#endif // CNTRLALLOC_H