#include <config.h>

#include "serialport.c"
#include "timing.c"

#if defined(_WIN32)
#  include "windows.c"
#elif defined(__linux)
#  include "linux.c"
#  include "linux_termios.h"
#elif defined(__APPLE__)
#  include "macosx.c"
#elif defined(__OpenBSD__)
#  include "freebsd.c"
#endif

