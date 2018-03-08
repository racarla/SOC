/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

*/

#ifndef WAVEGENFUNC_H
#define WAVEGENFUNC_H

#include <stdint.h>
#include <Eigen/Core>

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

// Single Channel Exication Signal Generators
float WavePulse(float &t_s, float &tOnePulse_s, float &amp_nd);
float WaveDoublet(float &t_s, float &tOnePulse_s, float &amp_nd);
float WaveDoublet121(float &t_s, float &tOnePulse_s, float &amp_nd);
float WaveDoublet3211(float &t_s, float &tOnePulse_s, float &amp_nd);
float WaveChirpLinear(float &t_s, float &tDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_rad);
float WaveMultisineOms(float &t_s, VecElem &freq_rps, VecElem &phase_rad, VecElem &amp_nd);

#endif // WAVEGENFUNC_H
