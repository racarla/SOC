/*
utils.hxx
Brian R Taylor
brian.taylor@bolderflight.com

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
*/

#ifndef UTILS_HXX_
#define UTILS_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <Eigen/Dense>
#include <sys/time.h>

uint64_t micros();

class elapsedMicros
{
private:
  unsigned long us;
public:
  elapsedMicros(void) { us = micros(); }
  elapsedMicros(unsigned long val) { us = micros() - val; }
  elapsedMicros(const elapsedMicros &orig) { us = orig.us; }
  operator unsigned long () const { return micros() - us; }
  elapsedMicros & operator = (const elapsedMicros &rhs) { us = rhs.us; return *this; }
  elapsedMicros & operator = (unsigned long val) { us = micros() - val; return *this; }
  elapsedMicros & operator -= (unsigned long val)      { us += val ; return *this; }
  elapsedMicros & operator += (unsigned long val)      { us -= val ; return *this; }
  elapsedMicros operator - (int val) const           { elapsedMicros r(*this); r.us += val; return r; }
  elapsedMicros operator - (unsigned int val) const  { elapsedMicros r(*this); r.us += val; return r; }
  elapsedMicros operator - (long val) const          { elapsedMicros r(*this); r.us += val; return r; }
  elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
  elapsedMicros operator + (int val) const           { elapsedMicros r(*this); r.us -= val; return r; }
  elapsedMicros operator + (unsigned int val) const  { elapsedMicros r(*this); r.us -= val; return r; }
  elapsedMicros operator + (long val) const          { elapsedMicros r(*this); r.us -= val; return r; }
  elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
};

#endif
