/*
Classes and Functions for managing JSON and Eigen and STL types

See: LICENSE.md for Copyright and License Agreement
*/

#include "Utilities.hxx"
#include <iostream>


// Convert a Json Array of Floats into a Eigen Vector
VecElem Json2Eigen_VecFloat(const ObjJson &objJson) {
  VecElem vec;

  assert(objJson.IsArray());
  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();

  vec.conservativeResize(kMaxWaveElem);
  vec.setConstant(0.0);
  if ((int) numElemJson <= kMaxWaveElem) {
    for (rapidjson::SizeType i = 0; i < numElemJson; i++) {
      vec[i] = objJson[i].GetFloat();
    }
  } else {
    std::cout << "The size of the JSON array exceeds the max size of the Eigen Vector: " << (int) numElemJson << "\t" << kMaxWaveElem << std::endl;
  }

  // Return Value
  return vec;
}


// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
MapFloat Json2Stl_MapFloat(const ObjJson &objJson) {
  MapFloat mapFloat;

  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    mapFloat.insert(std::make_pair(iMap->name.GetString(), objJson[iMap->name.GetString()].GetFloat()));
  }

  return mapFloat;
}
