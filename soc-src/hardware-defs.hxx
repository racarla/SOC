

#ifndef HARDWARE_DEFS_HXX_
#define HARDWARE_DEFS_HXX_

#include <unistd.h>
#include <termios.h>
#include <stdint.h>

const char FmuPort[] = "/dev/ttyO4";
const speed_t FmuBaud = B921600;
const size_t kUartBufferMaxSize = 4096;

#endif
