#ifndef PUDQ_LIB__VISIBILITY_CONTROL_H_
#define PUDQ_LIB__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PUDQ_LIB_EXPORT __attribute__ ((dllexport))
    #define PUDQ_LIB_IMPORT __attribute__ ((dllimport))
  #else
    #define PUDQ_LIB_EXPORT __declspec(dllexport)
    #define PUDQ_LIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef PUDQ_LIB_BUILDING_LIBRARY
    #define PUDQ_LIB_PUBLIC PUDQ_LIB_EXPORT
  #else
    #define PUDQ_LIB_PUBLIC PUDQ_LIB_IMPORT
  #endif
  #define PUDQ_LIB_PUBLIC_TYPE PUDQ_LIB_PUBLIC
  #define PUDQ_LIB_LOCAL
#else
  #define PUDQ_LIB_EXPORT __attribute__ ((visibility("default")))
  #define PUDQ_LIB_IMPORT
  #if __GNUC__ >= 4
    #define PUDQ_LIB_PUBLIC __attribute__ ((visibility("default")))
    #define PUDQ_LIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PUDQ_LIB_PUBLIC
    #define PUDQ_LIB_LOCAL
  #endif
  #define PUDQ_LIB_PUBLIC_TYPE
#endif

#endif  // PUDQ_LIB__VISIBILITY_CONTROL_H_
