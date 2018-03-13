/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

*/

#ifndef WAVEFUNC_H
#define WAVEFUNC_H

#include <stdint.h>
#include <Eigen/Core>

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

// Single Channel Exication Signal Generators
void WavePulse(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd);
void WaveDoublet(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd);
void WaveDoublet121(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd);
void WaveDoublet3211(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd);
void WaveChirpLinear(const float &t_s, const float &tDur_s, const float &freqStart_rps, const float &freqEnd_rps, const float &ampStart_nd, const float &ampEnd_nd, float *wave_nd);
void WaveMultisineOms(const float &t_s, const VecElem &freq_rps, const VecElem &phase_rad, const VecElem &amp_nd, float *wave_nd);

#endif // WAVEFUNC_H
