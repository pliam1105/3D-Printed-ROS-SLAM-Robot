#ifndef SLAM_BOT_HARDWARE__VISIBILITY_CONTROL_H_
#define SLAM_BOT_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SLAM_BOT_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define SLAM_BOT_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define SLAM_BOT_HARDWARE_EXPORT __declspec(dllexport)
    #define SLAM_BOT_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef SLAM_BOT_HARDWARE_BUILDING_LIBRARY
    #define SLAM_BOT_HARDWARE_PUBLIC SLAM_BOT_HARDWARE_EXPORT
  #else
    #define SLAM_BOT_HARDWARE_PUBLIC SLAM_BOT_HARDWARE_IMPORT
  #endif
  #define SLAM_BOT_HARDWARE_PUBLIC_TYPE SLAM_BOT_HARDWARE_PUBLIC
  #define SLAM_BOT_HARDWARE_LOCAL
#else
  #define SLAM_BOT_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define SLAM_BOT_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define SLAM_BOT_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define SLAM_BOT_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SLAM_BOT_HARDWARE_PUBLIC
    #define SLAM_BOT_HARDWARE_LOCAL
  #endif
  #define SLAM_BOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // SLAM_BOT_HARDWARE__VISIBILITY_CONTROL_H_
