#!/usr/bin/python3

'''
bin2hdf.py
Brian R Taylor
brian.taylor@bolderflight.com
mod's by Curtis L. Olson <olson126@umn.edu>

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''

# import libraries
import h5py
import numpy as np
import struct
import argparse
import os
import sys

# class for parsing Bfs messages
class BfsMessage:
    _ParserState = 0
    _Length = 0
    _ReturnDataType = 0
    _Buffer = []
    _Payload = []
    _ReturnPayload = []
    _LengthBuffer = []
    _Checksum = [0,0]
    DataTypes = ("Uint64Key","Uint32Key","Uint16Key","Uint8Key","Int64Key","Int32Key","Int16Key","Int8Key","FloatKey","DoubleKey","Uint64Desc","Uint32Desc","Uint16Desc","Uint8Desc","Int64Desc","Int32Desc","Int16Desc","Int8Desc","FloatDesc","DoubleDesc","Data")
    def Parse(Self,ByteRead):
        Header = bytearray([0x42,0x46])
        HeaderLength = 5;
        if Self._ParserState < 2:
            if ByteRead == Header[Self._ParserState]:
                Self._Buffer.append(ByteRead)
                Self._ParserState += 1
                return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 2:
            Self._Buffer.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 3:
            Self._Buffer.append(ByteRead)
            Self._LengthBuffer.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == 4:
            Self._Buffer.append(ByteRead)
            Self._LengthBuffer.append(ByteRead)
            Self._Length = struct.unpack_from('@H',bytearray(Self._LengthBuffer),0)[0]
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState < (Self._Length + HeaderLength):
            Self._Buffer.append(ByteRead)
            Self._Payload.append(ByteRead)
            Self._ParserState += 1
            return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == (Self._Length + HeaderLength):
            Self._Checksum = Self.CalcChecksum(Self._Buffer)
            if ByteRead == Self._Checksum[0]:
                Self._ParserState += 1
                return False, Self._ReturnDataType, Self._ReturnPayload
            else:
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return False, Self._ReturnDataType, Self._ReturnPayload
        elif Self._ParserState == (Self._Length + HeaderLength + 1):
            if ByteRead == Self._Checksum[1]:
                Self._ReturnPayload = Self._Payload
                Self._ReturnDataType = Self._Buffer[2]
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return True, Self._ReturnDataType, Self._ReturnPayload
            else:
                Self._ParserState = 0
                Self._Length = 0
                Self._Buffer = []
                Self._Payload = []
                Self._LengthBuffer = []
                Self._Checksum = [0,0]
                return False, Self._ReturnDataType, Self._ReturnPayload
        else:
            Self._ParserState = 0
            Self._Length = 0
            Self._Buffer = []
            Self._Payload = []
            Self._LengthBuffer = []
            Self._Checksum = [0,0]
            return False, Self._ReturnDataType, Self._ReturnPayload

    def CalcChecksum(Self,ByteArray):
        Checksum = [0,0]
        for i in range(0,len(ByteArray)):
            Checksum[0] = (Checksum[0] + ByteArray[i]) % 256
            Checksum[1] = (Checksum[1] + Checksum[0]) % 256
        return Checksum

# Parse input argument, file name to open
parser = argparse.ArgumentParser()
parser.add_argument("file", help="convert flight data binary to HDF5 format, enter binary file name as argument")
parser.add_argument("--output", help="specify output file name", action="store")
args = parser.parse_args()
print("Bolder Flight Systems")
print("Flight Data Binary to HDF5 Converter")
print("Version 1.0.0")
print("")
print("Converting:", args.file)
# Open binary file
try:
    BinaryFile = open(args.file, 'rb')
except IOError:
    print("Could not read file:", args.file)
    sys.exit()
# Read binary file and close
FileContents = BinaryFile.read()
BinaryFile.close()

# Create HDF5 file
if args.output:
    DataLogName = args.output
else:
    FileNameCounter = 0
    DataLogBaseName = "data"
    DataLogType = ".h5";
    DataLogName = DataLogBaseName + "%03d" % FileNameCounter + DataLogType
    while os.path.isfile(DataLogName):
        FileNameCounter += 1
        DataLogName = DataLogBaseName + "%03d" % FileNameCounter + DataLogType
print("Data log file:", DataLogName)
try:
    DataLogFile = h5py.File(DataLogName,'w-',libver='earliest')
except:
    print("Could not create output file:", args.output)
    sys.exit()
    
# instance of BfsMessage class to parse file
DataLogMessage = BfsMessage()
Uint64Keys = []
Uint64Desc = []
Uint64Data = []
Uint32Keys = []
Uint32Desc = []
Uint32Data = []
Uint16Keys = []
Uint16Desc = []
Uint16Data = []
Uint8Keys = []
Uint8Desc = []
Uint8Data = []
Int64Keys = []
Int64Desc = []
Int64Data = []
Int32Keys = []
Int32Desc = []
Int32Data = []
Int16Keys = []
Int16Desc = []
Int16Data = []
Int8Keys = []
Int8Desc = []
Int8Data = []
FloatKeys = []
FloatDesc = []
FloatData = []
DoubleKeys = []
DoubleDesc = []
DoubleData = []
# parse byte array
counter = 0
FileContentsBinary = bytearray(FileContents)
print('parsing binary file...')
for k in range(0,len(FileContentsBinary)):
    if counter > 100:
        break
    ReadByte = FileContentsBinary[k]
    result = DataLogMessage.Parse(ReadByte)
    if result != None:
        ValidMessage, DataType, Payload = result
    else:
        ValidMessage = False
    if ValidMessage:
        if DataLogMessage.DataTypes[DataType] == 'Uint64Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Uint64Keys.append(KeyName)
            Uint64Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Uint32Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Uint32Keys.append(KeyName)
            Uint32Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Uint16Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Uint16Keys.append(KeyName)
            Uint16Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Uint8Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Uint8Keys.append(KeyName)
            Uint8Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Int64Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Int64Keys.append(KeyName)
            Int64Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Int32Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Int32Keys.append(KeyName)
            Int32Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Int16Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Int16Keys.append(KeyName)
            Int16Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'Int8Key':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            Int8Keys.append(KeyName)
            Int8Data.append([])
        if DataLogMessage.DataTypes[DataType] == 'FloatKey':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            FloatKeys.append(KeyName)
            FloatData.append([])
        if DataLogMessage.DataTypes[DataType] == 'DoubleKey':
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            DoubleKeys.append(KeyName)
            DoubleData.append([])
        if DataLogMessage.DataTypes[DataType] == 'Uint64Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Uint64Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Uint32Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Uint32Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Uint16Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Uint16Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Uint8Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Uint8Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Int64Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Int64Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Int32Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Int32Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Int16Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Int16Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Int8Desc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            Int8Desc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'FloatDesc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            FloatDesc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'DoubleDesc':
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            DoubleDesc.append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Data':
            offset = 0
            for i in range (len(Uint64Keys)):
                Uint64Data[i].append(struct.unpack_from('@Q',bytearray(Payload),offset)[0])
                offset += 8
            for i in range (len(Uint32Keys)):
                Uint32Data[i].append(struct.unpack_from('@I',bytearray(Payload),offset)[0])
                offset += 4
            for i in range (len(Uint16Keys)):
                Uint16Data[i].append(struct.unpack_from('@H',bytearray(Payload),offset)[0])
                offset += 2
            for i in range (len(Uint8Keys)):
                Uint8Data[i].append(struct.unpack_from('@B',bytearray(Payload),offset)[0])
                offset += 1
            for i in range (len(Int64Keys)):
                Int64Data[i].append(struct.unpack_from('@q',bytearray(Payload),offset)[0])
                offset += 8
            for i in range (len(Int32Keys)):
                Int32Data[i].append(struct.unpack_from('@i',bytearray(Payload),offset)[0])
                offset += 4
            for i in range (len(Int16Keys)):
                Int16Data[i].append*(struct.unpack_from('@h',bytearray(Payload),offset)[0])
                offset += 2
            for i in range (len(Int8Keys)):
                Int8Data[i].append(struct.unpack_from('@b',bytearray(Payload),offset)[0])
                offset += 1
            for i in range (len(FloatKeys)):
                FloatData[i].append(struct.unpack_from('@f',bytearray(Payload),offset)[0])
                offset += 4
            for i in range (len(DoubleKeys)):
                DoubleData[i].append(struct.unpack_from('@d',bytearray(Payload),offset)[0])
                offset += 8
            # DataLogFile.flush()
            counter += 1
            if (counter % 500) == 0:
                print("Scanning:", "%.0f seconds" % (counter/50))
                
print("Constructing hdf5 file...")
for i in range (len(Uint64Keys)):
    d = DataLogFile.create_dataset(Uint64Keys[i], (counter, 1),
                                   data=np.array(Uint64Data[i]), dtype='uint64')
    d.attrs["Description"] = Uint64Desc[i]
for i in range (len(Uint32Keys)):
    d = DataLogFile.create_dataset(Uint32Keys[i], (counter, 1),
                                   data=np.array(Uint32Data[i]), dtype='uint32')
    d.attrs["Description"] = Uint32Desc[i]
for i in range (len(Uint16Keys)):
    d = DataLogFile.create_dataset(Uint16Keys[i], (counter, 1),
                                   data=np.array(Uint16Data[i]), dtype='uint16')
    d.attrs["Description"] = Uint16Desc[i]
for i in range (len(Uint8Keys)):
    d = DataLogFile.create_dataset(Uint8Keys[i], (counter, 1),
                                   data=np.array(Uint8Data[i]), dtype='uint8')
    d.attrs["Description"] = Uint8Desc[i]
for i in range (len(Int64Keys)):
    d = DataLogFile.create_dataset(Int64Keys[i], (counter, 1),
                                   data=np.array(Int64Data[i]), dtype='int64')
    d.attrs["Description"] = Int64Desc[i]
for i in range (len(Int32Keys)):
    d = DataLogFile.create_dataset(Int32Keys[i], (counter, 1),
                                   data=np.array(Int32Data[i]), dtype='int32')
    d.attrs["Description"] = Int32Desc[i]
for i in range (len(Int16Keys)):
    d = DataLogFile.create_dataset(Int16Keys[i], (counter, 1),
                                   data=np.array(Int16Data[i]), dtype='int16')
    d.attrs["Description"] = Int16Desc[i]
for i in range (len(Int8Keys)):
    d = DataLogFile.create_dataset(Int8Keys[i], (counter, 1),
                                   data=np.array(Int8Data[i]), dtype='int8')
    d.attrs["Description"] = Int8Desc[i]
for i in range (len(FloatKeys)):
    d = DataLogFile.create_dataset(FloatKeys[i], (counter, 1),
                                   data=np.array(FloatData[i]), dtype='float')
    d.attrs["Description"] = FloatDesc[i]
for i in range (len(DoubleKeys)):
    d = DataLogFile.create_dataset(DoubleKeys[i], (counter, 1),
                                   data=np.array(DoubleData[i]), dtype='double')
    d.attrs["Description"] = DoubleDesc[i]
DataLogFile.close()
print("done!")
print("Created data log file:", DataLogName)
