#ifndef LIBSERIALPORT_CONFIG_H
#  define LIBSERIALPORT_CONFIG_H

#  if defined(__linux)
#    ifndef LIBSERIALPORT_ATBUILD
#      define LIBSERIALPORT_ATBUILD
#    endif
#    define SP_PRIV __attribute__((visibility("hidden")))
#    define SP_API
#  elif defined(_WIN32)
#    ifndef LIBSERIALPORT_MSBUILD
#      define LIBSERIALPORT_MSBUILD
#    endif
#  elif defined(__APPLE__)
#  elif defined(__OpenBSD__)
#  endif
#endif // LIBSERIALPORT_CONFIG_H
