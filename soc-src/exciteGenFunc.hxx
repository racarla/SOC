/* 
 */

#ifndef EXCITEGEN_INTERFACE_H
#define EXCITEGEN_INTERFACE_H

#include <math.h>
#include <eigen3/Eigen/Dense>

#define MaxChan 16
#define MaxElem 30

// Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options = 0, int MaxRowsAtCompileTime = RowsAtCompileTime, int MaxColsAtCompileTime = ColsAtCompileTime>
typedef Eigen::Matrix<float, -1, 1, 0, MaxElem, 1> VectorElem;
typedef Eigen::Matrix<float, -1, -1, 0, MaxChan, MaxElem> MatrixElem;
typedef Eigen::Matrix<float, -1, 1, 0, MaxChan, 1> VectorExcite;


int ExciteDoublet(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const float &Amp_nd,
	float &Excite_nd
	);

int ExciteDoublet121(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const float &Amp_nd,
	float &Excite_nd
	);

int ExciteDoublet3211(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const float &Amp_nd,
	float &Excite_nd
	);

int ExciteChirp(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const float &FreqStart_rps, const float &FreqEnd_rps, const float &AmpStart_nd, const float &AmpEnd_nd,
	float &Excite_nd
	);

int ExciteOMS(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const VectorElem &FreqVec_rps, const VectorElem &PhaseVec_rad, const VectorElem &AmpVec_nd,
	float &Excite_nd
	);

int ExciteMultiOMS(
	const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s,
	const MatrixElem &FreqMat_rps, const MatrixElem &PhaseMat_rad, const MatrixElem &AmpMat_nd,
	VectorExcite &ExciteVec_nd
	);

#endif // EXCITEGEN_INTERFACE_H