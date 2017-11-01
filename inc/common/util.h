#ifndef UTIL_H
#define UTIL_H

/* util.h defines a set of useful utility functions and macros */

#include <cmath>

/* macro for checking result of SimConnect operations */
#define ASSERT_SC_SUCCESS(expr)                                    \
        if (expr != S_OK) {                                        \
          printf("Error, " # expr " did not evaluate OK.\n");      \
          abort();                                                 \
        }

/* convert degrees to radians */
constexpr double radians(double degrees) {
  return degrees * 0.0174533;
}

/* convert radians to degrees */
constexpr double degrees(double rad) {
  return rad * 57.2958;
}

/* returns sign of a value */
constexpr double sign(double val) {
  return (val > 0) ? 1.0 : -1.0;
}

#endif
