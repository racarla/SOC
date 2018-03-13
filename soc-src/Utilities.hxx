/*
Classes and Functions for managing JSON and Eigen and STL types

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef UTILITIES_H
#define UTILITIES_H


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
VecElem Json2Eigen_VecFloat(const ObjJson &objJson); // Vector of floats

// STL Types
typedef std::vector <float> VecFloat;
typedef std::map <std::string, float> MapFloat;
typedef std::map <std::string, MapFloat > MapMapFloat;
typedef std::vector <std::string> VecString;
typedef std::map <std::string, VecString> MapVecString;
typedef std::vector <std::pair<std::string,std::string>> VecStringPair;

// JSON to STL
VecFloat Json2Stl_VecFloat(const ObjJson &objJson); // Vector of floats
MapFloat Json2Stl_MapFloat(const ObjJson &objJson); // Map of floats
VecString Json2Stl_VecString(const ObjJson &objJson); // Vector of Strings
MapVecString Json2Stl_MapVecString(const ObjJson &objJson); // Map of Vector of Strings
VecStringPair Json2Stl_VecStringPair(const ObjJson &objJson); // Vector of String Pairs

#endif // WAVESYS_H
