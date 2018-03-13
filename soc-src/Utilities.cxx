/*
Classes and Functions for managing JSON and Eigen and STL types

See: LICENSE.md for Copyright and License Agreement
*/

#include "Utilities.hxx"
#include <iostream>


// Convert a Json Array of Floats into a Eigen Vector
void Json2Eigen_VecFloat(const ObjJson &objJson, VecElem *vec) {

  assert(objJson.IsArray());
  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();

  uint8_t numElemEigen = (uint8_t) (*vec).size();

  vec->setConstant(0.0);
  assert(numElemJson <= numElemEigen);
  for (rapidjson::SizeType i = 0; i < numElemJson; i++) {
    (*vec)[i] = objJson[i].GetFloat();
  }
}


// Convert a Json Object (list of objects of string array) to a vector
void Json2Stl_VecFloat(const ObjJson &objJson, VecFloat *vecFloat) {
  rapidjson::SizeType numElemJson = (uint8_t) objJson.Size();
  for (rapidjson::SizeType i = 0; i < numElemJson; i++) {
    vecFloat->push_back(objJson[i].GetFloat());
  }
}

// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
void Json2Stl_MapFloat(const ObjJson &objJson, MapFloat *mapFloat) {
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {
    mapFloat->insert(std::make_pair(iObj->name.GetString(), objJson[iObj->name.GetString()].GetFloat()));
  }
}

// Convert a Json Object (list of objects of string array) to a map of vectors (string keys of strings)
void Json2MapMapFloat(const ObjJson &objJson, MapMapFloat *mapMapFloat) {
  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    const ObjJson &objInner = objJson[iMap->name.GetString()];

    MapFloat mapFloat;
    Json2Stl_MapFloat(objInner, &mapFloat);

    mapMapFloat->insert(std::make_pair(iMap->name.GetString(), mapFloat));
    mapFloat.clear(); // Clear the inner map
  }
}

void Json2Stl_MapVecString(const ObjJson &objJson, MapVecString *mapVecString) {
  for (ObjJson::ConstMemberIterator iMap = objJson.MemberBegin(); iMap != objJson.MemberEnd(); ++iMap) {

    VecString vecString;
    Json2Stl_VecString(objJson[iMap->name.GetString()], &vecString);

    mapVecString->insert(std::make_pair(iMap->name.GetString(), vecString));
    vecString.clear();
  }
}

// Convert a Json Object (list of objects of strings) to a vector of strings
void Json2Stl_VecString(const ObjJson &objJson, VecString *vecString) {
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {
    vecString->push_back(iObj->name.GetString());
  }
}

// Convert a Json Object (list of objects of string pairs) to a vector of string pairs
void Json2Stl_VecStringPair(const ObjJson &objJson, VecStringPair *vecStringPair) {
  for (ObjJson::ConstMemberIterator iObj = objJson.MemberBegin(); iObj != objJson.MemberEnd(); ++iObj) {
    vecStringPair->push_back(std::make_pair(iObj->name.GetString(), objJson[iObj->name.GetString()].GetString()));
  }
}
