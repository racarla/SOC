/*
Classes and Functions for SCAS

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef SCASSYS_H
#define SCASSYS_H

#include <stdint.h>
#include <iostream>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "ScasFunc.hxx"
#include "Utilities.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

// Enumerations

// Base Class for all Scas systems
class ScasBase {
 public:
  virtual void Config(const ObjJson &objJson) {};
  virtual void Run(const float &tCurr_s, float *cmdVec) {};

  virtual ~ScasBase() {};
};

// Scas systems
class Scas1 : public ScasBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &tCurr_s, float *cmdVec);

  ~Scas1() {};

 private:
};

// Scas systems
class Scas2 : public ScasBase   {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &tCurr_s, float *cmdVec);

  ~Scas2() {};

 private:
};

// Setup Vector of Pointers to instances of ScasSys
typedef std::shared_ptr<ScasBase> ScasSysPtr;
typedef std::map <std::string, ScasSysPtr> ScasSysMap;

// Factory Class for Scas systems
class ScasFactory {
 public:
  static void Config(const ObjJson &objJson, ScasSysMap *scasSysMap);
  static void ConfigInst(const ObjJson &objJson, ScasSysPtr *scasSysPtr);
};

#endif // SCASSYS_H
