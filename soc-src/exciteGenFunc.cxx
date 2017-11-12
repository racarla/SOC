/*
 */


#include "exciteGenFunc.hxx"

int ExciteDoublet(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const float &Amp_nd, float &Excite_nd) {
	// Doublet 1-1 with 1-pulse TimeDur_s long, total of 2*TimeDur_s excitation

	float TimeDur2_s = 2.0 * TimeDur_s;
	
	Excite_nd = 0.0;	// [nd], excitation command

	if ((TimeCurr_s >= TimeStart_s) && (TimeCurr_s < TimeStart_s + TimeDur2_s)) {
		if (TimeCurr_s < TimeStart_s + TimeDur_s) {
			Excite_nd = Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur2_s) {
			Excite_nd = -Amp_nd;
			return 1;
		}
		
		return 1;
	}

	return 0;
}

int ExciteDoublet121(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const float &Amp_nd, float &Excite_nd) {
	// Doublet 1-2-1 - 1-pulse TimeDur_s long, total of 4*TimeDur_s excitation

	float TimeDur2_s = 2.0 * TimeDur_s;
	
	Excite_nd = 0.0;	// [nd], excitation command

	if ((TimeCurr_s >= TimeStart_s) && (TimeCurr_s < TimeStart_s + 4.0*TimeDur_s)) {
		if (TimeCurr_s < TimeStart_s + TimeDur_s) {
			Excite_nd = Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur_s + TimeDur2_s) {
			Excite_nd = -Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur_s + TimeDur2_s + TimeDur_s) {
			Excite_nd = Amp_nd;
			return 1;
		}
		
		return 1;
	}

	return 0;
}

int ExciteDoublet3211(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const float &Amp_nd, float &Excite_nd) {
	// Doublet 3-2-1-1 - 1-pulse TimeDur_s long, total of 7*TimeDur_s excitation

	float TimeDur2_s = 2.0 * TimeDur_s;
	float TimeDur3_s = 3.0 * TimeDur_s;
	
	Excite_nd = 0.0;	// [nd], excitation command

	if ((TimeCurr_s >= TimeStart_s) && (TimeCurr_s < TimeStart_s + 7.0*TimeDur_s)) {
		if (TimeCurr_s < TimeStart_s + TimeDur3_s) {
			Excite_nd = Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur3_s + TimeDur2_s) {
			Excite_nd = -Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur3_s + TimeDur2_s + TimeDur_s) {
			Excite_nd = Amp_nd;
			return 1;
		} else if (TimeCurr_s < TimeStart_s + TimeDur3_s + TimeDur2_s + TimeDur_s + TimeDur_s) {
			Excite_nd = -Amp_nd;
			return 1;
		}
		
		return 1;
	}

	return 0;
}

int ExciteChirp(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const float &FreqStart_rps, const float &FreqEnd_rps, const float &AmpStart_nd, const float &AmpEnd_nd, float &Excite_nd) {
	// Chirp (Frequency Sweep), linear varying amplitude and frequency
	
	float Freq_rps;		// [rps], frequency, linearly changing with time
	float Amp_nd = 0.0;	// [nd], amplitude, linearly changing with time

	Excite_nd = 0.0;	// [nd], excitation command
	
	if ((TimeCurr_s >= TimeStart_s) && (TimeCurr_s <= TimeStart_s + TimeDur_s)) {
		// linear varying instantanious frequency
		Freq_rps = FreqStart_rps + (FreqEnd_rps - FreqStart_rps) / (2.0 * TimeDur_s) * (TimeCurr_s - TimeStart_s);

		// linear varying amplitude
		Amp_nd = AmpStart_nd + (AmpEnd_nd - AmpStart_nd) * (TimeCurr_s - TimeStart_s) / TimeDur_s;

		// chirp Equation
		Excite_nd = Amp_nd * sin(Freq_rps * (TimeCurr_s - TimeStart_s));

		return 1;
	}

	return 0;
}

int ExciteOMS(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const VectorElem &FreqVec_rps, const VectorElem &PhaseVec_rad, const VectorElem &AmpVec_nd, float &Excite_nd) {
	// Optimal MultiSine
	int numElem = FreqVec_rps.size();	// Length of input vectors
	Excite_nd = 0.0;	// [nd], excitation command

	float scale = sqrt(1.0 / numElem);

	if ((TimeCurr_s >= TimeStart_s) && (TimeCurr_s <= TimeStart_s + TimeDur_s)) {
		// Loop over each element
		for (int iElem = 0; iElem < numElem; iElem++) {
			Excite_nd += scale * AmpVec_nd[iElem] * cos(FreqVec_rps[iElem] * (TimeCurr_s - TimeStart_s) + PhaseVec_rad[iElem]);
		}

		return 1;
	}

	return 0;
}


int ExciteMultiOMS(const float &TimeCurr_s, const float &TimeStart_s, const float &TimeDur_s, const MatrixElem &FreqMat_rps, const MatrixElem &PhaseMat_rad, const MatrixElem &AmpMat_nd, VectorExcite &ExciteVec_nd) {
	// Multichannel Optimal MultiSine
	int numChan = FreqMat_rps.rows(); // Number of channels
	
	ExciteVec_nd.fill(0.0);	// [nd], excitation vector command

	int exciteFlag = 0;

	// Loop over each channel
	for (int iChan = 0; iChan < numChan; iChan++) {

		exciteFlag = ExciteOMS(TimeCurr_s, TimeStart_s, TimeDur_s,
			FreqMat_rps.row(iChan), PhaseMat_rad.row(iChan), AmpMat_nd.row(iChan),
			ExciteVec_nd(iChan));
	}

	return exciteFlag;
}