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
VecFloat Json2Stl_VecFloat(const ObjJson &objJson) {
  VecFloat vecFloat;

  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();
    for (rapidjson::SizeType i = 0; i < numElemJson; i++) {

    vecFloat.push_back(objJson[i].GetFloat());
  }

  return vecFloat;
}

// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
MapFloat Json2Stl_MapFloat(const ObjJson &objJson) {
  MapFloat mapFloat;

  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {

    mapFloat.insert(std::make_pair(iObj->name.GetString(), objJson[iObj->name.GetString()].GetFloat()));
  }

  return mapFloat;
}

// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
MapMapFloat Json2MapMapFloat(const ObjJson &objJson) {
  MapMapFloat mapMapFloat;

  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    const ObjJson &objInner = objJson[iMap->name.GetString()];

    MapFloat mapFloat = Json2Stl_MapFloat(objInner);

    mapMapFloat.insert(std::make_pair(iMap->name.GetString(), mapFloat));
    mapFloat.clear(); // Clear the inner map
  }

  return mapMapFloat;
}

MapVecString Json2Stl_MapVecString(const ObjJson &objJson) {
  MapVecString mapVecString;

  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    VecString vecString = Json2Stl_VecString(objJson[iMap->name.GetString()]);

    mapVecString.insert(std::make_pair(iMap->name.GetString(), vecString));
    vecString.clear();
  }

  return mapVecString;

}

// Convert a Json Object (list of objects of strings) to a vector of strings
VecString Json2Stl_VecString(const ObjJson &objJson) {
  VecString vecString;

  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {
    vecString.push_back(iObj->name.GetString());
  }

  return vecString;
}

// Convert a Json Object (list of objects of string pairs) to a vector of string pairs
VecStringPair Json2Stl_VecStringPair(const ObjJson &objJson) {
  VecStringPair vecStringPair;

  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {
    vecStringPair.push_back(std::make_pair(iObj->name.GetString(), objJson[iObj->name.GetString()].GetString()));
  }

  return vecStringPair;
}
