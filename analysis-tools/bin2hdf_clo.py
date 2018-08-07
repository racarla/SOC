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
import re
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
print("Version 1.0.1")
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
print("Data log file name:", DataLogName)
try:
    DataLogFile = h5py.File(DataLogName,'w-',libver='earliest')
except:
    print("Could not create output file:", args.output)
    sys.exit()
    
# instance of BfsMessage class to parse file
DataLogMessage = BfsMessage()
storage = {}
types = [ 'Uint64', 'Uint32', 'Uint16', 'Uint8',
          'Int64', 'Int32', 'Int16', 'Int8',
          'Float', 'Double' ]
np_types = [ 'uint64', 'uint32', 'uint16', 'uint8',
          'int64', 'int32', 'int16', 'int8',
          'float', 'double' ]
packstr = [ 'Q', 'I', 'H', 'B',
            'q', 'i', 'h', 'b',
            'f', 'd' ]
sizes = [ 8, 4, 2, 1, 8, 4, 2, 1, 4, 8 ]
for t in types:
    storage[t] = { 'keys': [], 'desc': [], 'data': [] }

# parse byte array
counter = 0
pkey = re.compile('(.+)Key')
pdesc = re.compile('(.+)Desc')

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
        # match 'Key' token
        mkey = pkey.match(DataLogMessage.DataTypes[DataType])
        if mkey != None:
            KeyName = ""
            for i in range(0,len(Payload)):
                KeyName += chr(Payload[i])
            mtype = mkey[1]
            storage[mtype]['keys'].append(KeyName)
            storage[mtype]['data'].append([])
        # match 'Desc' token
        mdesc = pdesc.match(DataLogMessage.DataTypes[DataType])
        if mdesc != None:
            Desc = ""
            for i in range(0,len(Payload)):
                Desc += chr(Payload[i])
            dtype = mdesc[1]
            storage[dtype]['desc'].append(Desc)
        if DataLogMessage.DataTypes[DataType] == 'Data':
            offset = 0
            for j, t in enumerate(types):
                form = '@' + packstr[j]
                for i in range (len(storage[t]['keys'])):
                    storage[t]['data'][i].append(struct.unpack_from(form, bytearray(Payload),offset)[0])
                offset += sizes[j]
            # DataLogFile.flush()
            counter += 1
            if (counter % 500) == 0:
                print("Scanning:", "%.0f seconds" % (counter/50))
                
print("Writing hdf5 file...")
for j, t in enumerate(types):
    for i in range (len(storage[t]['keys'])):
        d = DataLogFile.create_dataset(storage[t]['keys'][i], (counter, 1),
                                       data=np.array(storage[t]['data'][i]),
                                       dtype=np_types[j])
        d.attrs["Description"] = storage[t]['desc'][i]
DataLogFile.close()
print("Finished writing hdf5 file:", DataLogName)
