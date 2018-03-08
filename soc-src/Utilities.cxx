



#include <stdint.h>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "WaveGenFunc.hxx"

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

typedef std::map <std::string, float> MapFloat;
typedef rapidjson::Value ObjJson;

// Convert a Json Array of Floats into a Eigen Vector
VecElem Json2Eigen_VecFloat(const ObjJson &objJson) {
  VecElem vec;

  assert(objJson.IsArray());

  for (rapidjson::SizeType i = 0; i < objJson.Size(); i++) {
    freq_rps_[i] = objJson[i].GetFloat());
  }
  retun vec;
}


// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
MapFloat Json2Stl_MapFloat(const ObjJson &objJson) {
  MapFloat mapFloat;

  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    mapFloat.insert(std::make_pair(iMap->name.GetString(), objJson[iMap->name.GetString()].GetFloat()));
  }

  return mapFloat;
}
