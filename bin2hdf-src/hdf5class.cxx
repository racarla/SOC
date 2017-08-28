
#include "hdf5class.hxx"

hdf5class::hdf5class(std::string FileName) {
  file_ = new H5::H5File(FileName.c_str(), H5F_ACC_EXCL);
}

void hdf5class::CreateGroup(std::string GroupName) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());
}

void hdf5class::WriteData(std::string GroupName,std::string Name,uint8_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT8,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT8);
}

void hdf5class::WriteData(std::string GroupName,std::string Name,uint16_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT16,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT16);
}

void hdf5class::WriteData(std::string GroupName,std::string Name,uint32_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT32,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT32);
}

void hdf5class::WriteData(std::string GroupName,std::string Name,float *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_FLOAT,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_FLOAT);
}

void hdf5class::WriteData(std::string GroupName,std::string Name,double *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_DOUBLE,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_DOUBLE);
}
