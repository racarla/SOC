
#ifndef HDF5CLASS_HXX_
#define HDF5CLASS_HXX_

#include <H5Cpp.h>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"

class hdf5class {
  public:
    hdf5class(std::string FileName);
    void CreateGroup(std::string GroupName);
    void WriteData(std::string GroupName,std::string Name,uint8_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,uint16_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,uint32_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,float *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,double *data,std::string Attr,size_t rows,size_t columns);
  private:
    H5::H5File *file_;
};

#endif
