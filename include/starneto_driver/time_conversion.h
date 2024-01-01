// Converts GPS timestamp from/to UNIX system timestamp.
// This helper is considering leap second.

#pragma once

#include <stdint.h>
#include <ros/ros.h>

// #include "glog/logging.h"
// #include "macros.h"

// File expires on:  December 2017

// There's two concept about 'time'.
// 1. Unix time : It counts seconds since Jan 1,1970.
//                Unix time doesn't include the leap seconds.
//
// 2. GPS time : It counts seconds since Jan 6,1980.
//               GPS time does include the leap seconds.
//
// Leap seconds was added in every a few years, See details:
// http://tycho.usno.navy.mil/leapsec.html
// https://en.wikipedia.org/wiki/Leap_second
//

/* leap_seconds table since 1980.
 +======+========+========+======+========+========+
 | Year | Jun 30 | Dec 31 | Year | Jun 30 | Dec 31 |
 +======+========+========+======+========+========+
 | 1980 | (already +19)   | 1994 | +1     | 0      |
 +------+--------+--------+------+--------+--------+
 | 1981 | +1     | 0      | 1995 | 0      | +1     |
 +------+--------+--------+------+--------+--------+
 | 1982 | +1     | 0      | 1997 | +1     | 0      |
 +------+--------+--------+------+--------+--------+
 | 1983 | +1     | 0      | 1998 | 0      | +1     |
 +------+--------+--------+------+--------+--------+
 | 1985 | +1     | 0      | 2005 | 0      | +1     |
 +------+--------+--------+------+--------+--------+
 | 1987 | 0      | +1     | 2008 | 0      | +1     |
 +------+--------+--------+------+--------+--------+
 | 1989 | 0      | +1     | 2012 | +1     | 0      |
 +------+--------+--------+------+--------+--------+
 | 1990 | 0      | +1     | 2015 | +1     | 0      |
 +------+--------+--------+------+--------+--------+
 | 1992 | +1     | 0      | 2016 | 0      | +1     |
 +------+--------+--------+------+--------+--------+
 | 1993 | +1     | 0      | 2017 | 0      | 0      |
 +------+--------+--------+------+--------+--------+

 Current TAI - UTC = 37. (mean that: 2017 - 1970/01/01 = 37 seconds)
*/

// We build a lookup table to describe relationship that between UTC and
// Leap_seconds.
//
// Column1: UTC time diff (second).
//          Shell Example:
//                 date +%s -d"Jan 1, 2017 00:00:00"
//          return 1483257600.
//
//                 date +%s -d"Jan 1, 1970 00:00:00"
//          return 28800.
//
//          The diff between 1970/01/01 and 2017/01/01 is 1483228800.
//          (1483257600-28800)
//
// Column2: Leap seconds diff with GPS basetime(Jan 6,1980).
//          We Know that The leap_seconds have been already added 37 seconds
//          util now(2017).
//          So we can calculate leap_seconds diff between GPS (from Jan 6,1980)
//          and UTC time.

// calc with the formula.
static const int32_t LEAP_SECONDS[][2] = {
    {1483228800, 18},  // 2017/01/01
    {1435708800, 17},  // 2015/07/01
    {1341100800, 16},  // 2012/07/01
    {1230768000, 15},  // 2009/01/01
    {1136073600, 14},  // 2006/01/01
    {915148800, 13},   // 1999/01/01
    {867711600, 12},   // 1997/07/01
    {820480320, 11},   // 1996/01/01 ;)
                       //....
                       //..
                       // etc.
};

// seconds that UNIX time afront of GPS, without leap seconds.

// Shell:
// time1 = date +%s -d"Jan 6, 1980 00:00:00"
// time2 = date +%s -d"Jan 1, 1970 00:00:00"
// dif_tick = time1-time2
// 315964800 = 315993600 - 28800

const int32_t GPS_AND_SYSTEM_DIFF_SECONDS = 315964800;

template <class T>
size_t array_size(T& array) {
  return (sizeof(array) / sizeof(array[0]));
}

template <typename T>
T unix2gps(const T unix_seconds) {
  // size_t n = sizeof(LEAP_SECONDS) / (2 * sizeof(typeof(int32_t)));
  for (size_t i = 0; i < array_size(LEAP_SECONDS); ++i) {
    if (unix_seconds >= LEAP_SECONDS[i][0]) {
      return unix_seconds - (GPS_AND_SYSTEM_DIFF_SECONDS - LEAP_SECONDS[i][1]);
    }
  }
  return static_cast<T>(0);
}

template <typename T>
T gps2unix(const T gps_seconds) {
  // size_t n = sizeof(LEAP_SECONDS) / (2 * sizeof(typeof(int32_t)));
  for (size_t i = 0; i < array_size(LEAP_SECONDS); ++i) {
    T result = gps_seconds + (GPS_AND_SYSTEM_DIFF_SECONDS - LEAP_SECONDS[i][1]);
    if (result >= LEAP_SECONDS[i][0]) {
      return result;
    }
  }
  return static_cast<T>(0);
}
