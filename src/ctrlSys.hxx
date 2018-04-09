/*
Classes and Functions for Control Systems

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef CTRLSYS_H
#define CTRLSYS_H

#include <stdint.h>
#include <map>
#include <vector>
#include <memory>

#include "Eigen/Core"

#include "ctrlFunc.hxx"
#include "configFunc.hxx"
#include "definition-tree.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

typedef rapidjson::Value ObjJson;

// Base Class for all Ctrl systems
class CtrlBase {
 public:
  virtual void Config(const ObjJson &objJson) {};
  virtual void Run(DefinitionTree *signalTreePtr) {};

  virtual ~CtrlBase() {};
};

// Ctrl Empty system
class CtrlNone : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson) {};
  void Run(DefinitionTree *signalTreePtr) {};

  ~CtrlNone() {};
};

// Ctrl Constant system
class CtrlConst : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(DefinitionTree *signalTreePtr);

  ~CtrlConst() {};
 private:
  float val_;
};

// Ctrl Sum system, Just add two signals
class CtrlSum : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(DefinitionTree *signalTreePtr);

  ~CtrlSum() {};
};

// Ctrl Gain system
class CtrlGain : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(DefinitionTree *signalTreePtr);

  ~CtrlGain() {};
 private:
  float val_;
};

// Ctrl PID2 system
class CtrlPid2 : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson);
  void Run(DefinitionTree *signalTreePtr);

  ~CtrlPid2() {};
 private:
  CtrlFuncPid2 ctrlFuncPid2_;
};

// Ctrl State Space system
class CtrlSS : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson) {};
  void Run(DefinitionTree *signalTreePtr) {};

  ~CtrlSS() {};
 private:
  // CtrlFuncSS ctrlFuncSS;
};

// Factory Class for Ctrl systems
class CtrlSys {
 public:
  typedef std::shared_ptr<CtrlBase> SysInstPtr; // Pointer to an individual controller
  typedef std::vector <SysInstPtr> SysDefVec; // Vector of pointers to controllers
  typedef std::map <std::string, SysDefVec> SysDefMap; // Map of Vectors to Pointer to contoller instances

  static void ConfigDef(const ObjJson &objJson, SysDefMap *sysDefMap, DefinitionTree *signalTree); // Configuration of the whole Controller Definition Set
  static void ConfigDefVec(const ObjJson &objJson, SysDefVec *SysDefVec, DefinitionTree *signalTree); // Configure vector of controllers, ie. ScasBaseline
  static void ConfigDefInst(const ObjJson &objJson, SysInstPtr *SysInstPtr, DefinitionTree *signalTree); // Configure a single instance of a controller, ie. a single PID2 instance
  static void ConfigSignal(const ObjJson &objJson, const std::string &defSignal, float *targetPtr, DefinitionTree *signalTree);

  typedef std::vector<std::string> SysGroupVec; // Vector of strings descriptors
  typedef std::map <std::string, SysGroupVec> SysGroupMap; // Map of Vectors
  static void ConfigGroup(const ObjJson &objJson, SysGroupMap *sysGroupMap); // Configuration of the Controller Groups, ie. "Baseline" (consisting of "GuidBaseline" and "ScasBaseline")


  static void RunVec(const SysDefVec &SysDefVec, DefinitionTree &signalTree); // Loop through and run each of a vector of controllers, ie. ScasBaseline
  static void Run(const float &tCurr_s); // Run a Grouping of Controllers, ie. "Baseline" (consisting of "GuidBaseline" and "ScasBaseline")

 private:
  enum EnumType {kNone = 0, kConst = 1, kSum = 2, kGain = 3, kPid2 = 11, kSS = 41}; // Enumerations of the WaveSystem types
};


#endif // CTRLSYS_H
