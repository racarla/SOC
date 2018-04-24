/*
Classes and Functions for Control Systems

See: LICENSE.md for Copyright and License Agreement
*/

#pragma once

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
typedef std::vector<float> VecSignal;

// Base Class for all Ctrl systems
class CtrlBase {
 public:
  virtual void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr) {};
  virtual void Run() {};

  virtual ~CtrlBase() {};
};

// Ctrl Empty system
class CtrlNone : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr) {};
  void Run() {};

  ~CtrlNone() {};
};

// Ctrl Constant system
class CtrlConst : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr);
  void Run();

  ~CtrlConst() {};
 private:
  float val_;

  VecSignal vecOutSignal_; // Pointers to OutSignal
};

// Ctrl Sum system, Just add two signals
class CtrlSum : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr);
  void Run();

  ~CtrlSum() {};
 private:
  VecSignal vecRefSignal_; // Pointers to RefSignal
  VecSignal vecOutSignal_; // Pointers to OutSignal
};

// Ctrl Gain system
class CtrlGain : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr);
  void Run();

  ~CtrlGain() {};
 private:
  float val_;

  VecSignal vecRefSignal_; // Pointers to RefSignal
  VecSignal vecOutSignal_; // Pointers to OutSignal
};

// Ctrl PID2 system
class CtrlPid2 : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr);
  void Run(CtrlMode &ctrlMode);

  ~CtrlPid2() {};
 private:
  CtrlFuncPid2 ctrlFuncPid2_;

  VecSignal vecRefSignal_; // Pointers to RefSignal
  VecSignal vecMeasSignal_; // Pointers to MeasSignal
  VecSignal vecOutSignal_; // Pointers to OutSignal
};

// Ctrl State Space system
class CtrlSS : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson, DefinitionTree *signalTreePtr) {};
  void Run(CtrlMode &ctrlMode) {};

  ~CtrlSS() {};
 private:
  // CtrlFuncSS ctrlFuncSS_;

  VecSignal vecRefSignal_; // Pointers to RefSignal
  VecSignal vecMeasSignal_; // Pointers to MeasSignal
  VecSignal vecOutSignal_; // Pointers to OutSignal
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
  static void ConfigSignal(const ObjJson &objJson, const std::string &defSignal, VecSignal *vecSignal, DefinitionTree *signalTree);

  typedef std::vector<std::string> SysGroupVec; // Vector of strings descriptors
  typedef std::map <std::string, SysGroupVec> SysGroupMap; // Map of Vectors
  static void ConfigGroup(const ObjJson &objJson, SysGroupMap *sysGroupMap); // Configuration of the Controller Groups, ie. "Baseline" (consisting of "GuidBaseline" and "ScasBaseline")

  static void RunGroup(const SysGroupMap &sysGroupMap, DefinitionTree &signalTree, CtrlMode &ctrlMode);
  static void RunVec(const SysDefVec &SysDefVec, DefinitionTree &signalTree, CtrlMode &ctrlMode); // Loop through and run each of a vector of controllers, ie. ScasAttitude
  static void Run(DefinitionTree &signalTree, CtrlMode &ctrlMode);

 private:
  enum EnumType {kNone = 0, kConst = 1, kSum = 2, kGain = 3, kPid2 = 11, kSS = 41}; // Enumerations of the WaveSystem types
};
