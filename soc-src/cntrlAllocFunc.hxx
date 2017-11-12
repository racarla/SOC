/*
*/

#ifndef CNTRLALLOC_INTERFACE_H
#define CNTRLALLOC_INTERFACE_H

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


void ControlAlloc(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd,
  int method);

// Allocation Methods
void ContAllocPseudo(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uPref,
  VecEff &uCmd);

void ContAllocPseudoWeighted(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref, 
  VecEff &uCmd);

void ContAllocFxp(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd, 
  float gammaEff = 0.001, int numIter = 200);

void ContAllocPseudoRedis(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd);

void ContAllocPseudoRedisScale(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd);

void ContAllocMOLP(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd,
  float gammaEff = 0.001);

void ContAllocMOQP(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
  VecEff &uCmd,
  float gammaEff = 0.001);

void ContAllocSQP(
  const MatCntrlEff &cntrlEff, const VecObj &vObj, const VecEff &uMin, const VecEff &uMax, const MatObj &wObj, const MatEff &wEff, const VecEff &uPref,
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


#endif // CNTRLALLOC_INTERFACE_H