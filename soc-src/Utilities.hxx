/*
Classes and Functions for managing JSON and Eigen and STL types

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef UTILITIES_H
#define UTILITIES_H


#include <stdint.h>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "../soc-includes/rapidjson/document.h"
#include "../soc-includes/rapidjson/stringbuffer.h"
#include "../soc-includes/rapidjson/writer.h"

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

typedef std::map <std::string, float> MapFloat;
typedef rapidjson::Value ObjJson;

// JSON to Eigen
VecElem Json2Eigen_VecFloat(const ObjJson &objJson); // Vector of floats

// JSON to STL
MapFloat Json2Stl_MapFloat(const ObjJson &objJson); // Map of floats

#endif // WAVESYS_H
