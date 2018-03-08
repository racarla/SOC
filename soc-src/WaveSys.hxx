/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#ifndef WAVESYS_H
#define WAVESYS_H

#include <stdint.h>
#include <Eigen/Core>

#include "WaveGenFunc.hxx"
#include "Utilities.hxx"

#ifndef kMaxWaveElem
#define kVerboseConfig 0
#endif

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

enum WaveType {kPulse = 1, kDoublet = 2, kDoublet121 = 3, kDoublet3211 = 4, kChirpLin = 11, kOMS = 21};
enum WaveClass {kDisc = 0, kChirp = 1, kMultisine = 2};

// Base Class for all Waveforms
class WaveSys {
 public:
  WaveSys() {};
  virtual ~WaveSys() {};
  virtual void Config(const ObjJson &objJson) {};
  virtual float Run(float tCurr_s) {};

 private:
  uint8_t iWave_;
  std::string sWaveType_;
  WaveClass eWaveClass_;
  WaveType eWaveType_;
};

// Discrete Waveforms
class WaveDisc : public WaveSys  {
 public:
  WaveDisc() {}; // Constructor
  ~WaveDisc() {}; // Destructor
  void Config(const ObjJson &objJson);
  float Run(float tCurr_s);

 private:
  float tOnePulse_s_, tDur_s_, amp_nd_;
};

// Chirp Waveforms
class WaveChirp : public WaveSys   {
 public:
  WaveChirp() {}; // Constructor
  ~WaveChirp() {}; // Destructor
  void Config(const ObjJson &objJson);
  float Run(float tCurr_s);

 private:
  float tDur_s_;
  float freqStart_rps_, freqEnd_rps_, ampStart_nd_, ampEnd_nd_;
};

// MultiSine Waveforms
class WaveMultisine : public WaveSys   {
 public:
  WaveMultisine() {}; // Constructor
  ~WaveMultisine() {}; // Destructor
  void Config(const ObjJson &objJson);
  float Run(float tCurr_s);

 private:
  float tDur_s_;
  VecElem freq_rps_, phase_rad_, amp_nd_;
};

#endif // WAVESYS_H
