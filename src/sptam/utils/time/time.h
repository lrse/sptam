/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <cmath>
#include <limits>
#include <cstdint>
#include <stdexcept>
#include <boost/math/special_functions/round.hpp>
#include <iostream>
namespace ros
{

static void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
  uint64_t nsecpart = nsec % 1000000000UL;
  uint64_t secpart = nsec / 1000000000UL;

  if (std::numeric_limits<uint32_t>::max() < sec + secpart)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += secpart;
  nsec = nsecpart;
}

static void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}

static void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
  int64_t nsec_part = nsec % 1000000000L;
  int64_t sec_part = sec + nsec / 1000000000L;
  if (nsec_part < 0)
  {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > std::numeric_limits<uint32_t>::max())
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

class Time
{
  public:

    uint32_t sec, nsec;

    Time()
      : sec(0), nsec(0)
    {}

    explicit Time(double _sec)
    { fromSec( _sec ); }

    Time(uint32_t _sec, uint32_t _nsec)
      : sec(_sec), nsec(_nsec)
    { normalizeSecNSec(sec, nsec); }

    Time operator - (const Time& rhs) const
    {
      if( *this < rhs )
      {
        std::cout << *this << " - " << rhs << std::endl;
        throw std::runtime_error("Cannot substract a bigger time value");
      }

      uint32_t secs = sec - rhs.sec;

      uint64_t nsecs;
      if (nsec < rhs.nsec) {
        secs = secs - 1;
        nsecs = ((uint64_t)nsec + toNSec(1)) - (uint64_t)rhs.nsec;
      } else {
        nsecs = nsec - rhs.nsec;
      }

      if (std::numeric_limits<uint32_t>::max() < nsecs)
        throw std::runtime_error("Time is out of dual 32-bit range");

      return Time(secs, nsecs);
    }

    Time operator + (const Time& rhs) const
    {
      int64_t sec_sum  = (int64_t)sec  + (int64_t)rhs.sec;
      int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;

      // Throws an exception if we go out of 32-bit range
      normalizeSecNSecUnsigned(sec_sum, nsec_sum);

      // now, it's safe to downcast back to uint32 bits
      return Time((uint32_t)sec_sum, (uint32_t)nsec_sum);
   }

    Time& operator += (const Time& rhs)
    { *this = *this + rhs; return *this; }

    //~ Duration operator-(const Time &rhs) const;
    //~ Time& operator-=(const Time &rhs);

    bool operator == (const Time& rhs) const
    { return sec == rhs.sec && nsec == rhs.nsec; }

    inline bool operator != (const Time& rhs) const
    { return !(*this == rhs); }

    //~ bool operator>(const Time &rhs) const;

    bool operator < (const Time& rhs) const
    {
      if (sec < rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec < rhs.nsec)
        return true;
      return false;
    }

    //~ bool operator>=(const Time &rhs) const;

    bool operator <= (const Time& rhs) const
    {
      if (sec < rhs.sec)
        return true;
      else if (sec == rhs.sec && nsec <= rhs.nsec)
        return true;
      return false;
    }

    friend std::ostream& operator << (std::ostream& os, const Time& rhs)
    { os << rhs.sec << " " << rhs.nsec; return os; }

    double toSec() const
    { return (double)sec + 1e-9*(double)nsec; };

    uint64_t toNSec() const
    {return (uint64_t)sec*1000000000ull + (uint64_t)nsec; }

    static uint64_t toNSec(uint32_t sec)
    { return (uint64_t)sec*1000000000ull; }

  private:

    Time& fromSec(double t)
    {
      int64_t sec64 = (int64_t)std::floor(t);

      if (sec64 < 0 || sec64 > std::numeric_limits<uint32_t>::max())
        throw std::runtime_error("Time is out of dual 32-bit range");

      sec = sec64;
      nsec = boost::math::round((t-sec) * 1e9);

      // avoid rounding errors
      sec += (nsec / 1000000000ul);
      nsec %= 1000000000ul;

      return *this;
    }
};

} // ros
