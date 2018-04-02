/*
Classes and Functions for managing JSON and Eigen and STL types for Configuration

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef CONFIGFUNC_H
#define CONFIGFUNC_H


#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

typedef rapidjson::Value ObjJson;


// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

// JSON to Eigen
void Json2Eigen_VecFloat(const ObjJson &objJson, VecElem *vecElem); // Vector of floats

// STL Types
typedef std::vector <float> VecFloat;
typedef std::map <std::string, float> MapFloat;
typedef std::map <std::string, MapFloat > MapMapFloat;
typedef std::vector <std::string> VecString;
typedef std::map <std::string, VecString> MapVecString;
typedef std::vector <std::pair<std::string,std::string>> VecStringPair;

// JSON to STL
void Json2Stl_VecFloat(const ObjJson &objJson, VecFloat *vecFloat); // Vector of floats
void Json2Stl_MapFloat(const ObjJson &objJson, MapFloat *mapFloat); // Map of floats
void Json2Stl_VecString(const ObjJson &objJson, VecString *vecString); // Vector of Strings
void Json2Stl_MapVecString(const ObjJson &objJson, MapVecString *mapVecString); // Map of Vector of Strings
void Json2Stl_VecStringPair(const ObjJson &objJson, VecStringPair *vecStringPair); // Vector of String Pairs

#endif // CONFIGFUNC_H
