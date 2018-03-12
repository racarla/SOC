#
# MAKEFILE
#
# Brian R Taylor
# brian.taylor@bolderflight.com
# 2017-04-18
#
# Copyright (c) 2017 Bolder Flight Systems
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
# and associated documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, distribute, 
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or 
# substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# compiler
CC=g++ -std=c++0x

# configuration
LFLAGS=-lz -lm -lhdf5 -lhdf5_cpp

# code to be compiled
OBJ =\
hdf5class.cxx \
main.cxx

# rules
all: output display

output: $(OBJ)
	@ echo "Building..."	
	$(CC) -I../bin2hdf-includes/ -I/usr/local/include -I/usr/include/hdf5/serial/ -L/usr/lib/x86_64-linux-gnu/hdf5/serial/lib -L/usr/lib/x86_64-linux-gnu/hdf5/serial/lib/libhdf5_cpp.a $^ -o $@ $(LFLAGS)
		
clean:
	-rm output

display: 
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, Bolder by Design!"
	@ echo "Copyright (c) 2017 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo "" 
