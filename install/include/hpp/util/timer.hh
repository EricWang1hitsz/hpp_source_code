// Copyright (c) 2015, LAAS-CNRS
// Authors: Thomas Moulard, Joseph Mirabel
//
// This file is part of hpp-util.
// hpp-util is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-util is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-util.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_UTIL_TIMER_HH
# define HPP_UTIL_TIMER_HH

# include "boost/date_time/posix_time/posix_time_types.hpp"

# ifdef HPP_ENABLE_BENCHMARK
#  include <boost/date_time/posix_time/posix_time.hpp>
# endif // HPP_ENABLE_BENCHMARK

# include <hpp/util/config.hh>
# include <hpp/util/debug.hh>

namespace hpp
{
  namespace debug
  {
    class HPP_UTIL_DLLAPI Timer
    {
    public:
      typedef boost::posix_time::ptime ptime;
      typedef boost::posix_time::time_duration time_duration;
      typedef boost::posix_time::time_period time_period;

      explicit Timer (bool autoStart = false);
      Timer (const Timer&);
      Timer& operator= (const Timer&);
      ~Timer ();

      const ptime& start ();
      const ptime& stop ();
      time_duration duration () const;

      const ptime& getStart () const;
      const ptime& getStop () const;

      std::ostream& print (std::ostream&) const;
    private:
      ptime start_;
      ptime end_;
    };

# ifdef HPP_ENABLE_BENCHMARK

#  define hppStartBenchmark(ID)			\
    hppDout (benchmark, #ID << ": start");	\
    ::hpp::debug::Timer _##ID##_timer_ (true)

#  define hppStopBenchmark(ID)			\
    do {					\
      _##ID##_timer_.stop ();			\
      hppDout (benchmark, #ID << ": stop");	\
    } while (0)

#  define hppDisplayBenchmark(ID)					\
    hppDout (benchmark, #ID << ": "<< _##ID##_timer_.duration ());

#  define hppBenchmark(data)                                           \
  do {									\
    using namespace hpp;						\
    using namespace ::hpp::debug;					\
    std::stringstream __ss;						\
    __ss << data << iendl;						\
    logging.benchmark.write (__FILE__, __LINE__, __PRETTY_FUNCTION__,   \
			   __ss);					\
  } while (0)

# else
#  define hppStartBenchmark(ID)
#  define hppStopBenchmark(ID)
#  define hppDisplayBenchmark(ID)
#  define hppBenchmark(data)
# endif // HPP_ENABLE_BENCHMARK

    /// \brief Computation of min, max and mean time from a set of measurements.
    class HPP_UTIL_DLLAPI TimeCounter
    {
      public:
        struct Scope {
          Scope (TimeCounter& t) : tc(t) { t.start(); }
          ~Scope () { tc.stop(); }

          TimeCounter& tc;
        };

        typedef boost::posix_time::ptime ptime;
        typedef boost::posix_time::time_duration time_duration;

        TimeCounter (const std::string& name);

        void start ();
        time_duration stop ();
        time_duration last ();
        void reset ();

        time_duration min () const;
        time_duration max () const;
        time_duration mean () const;
        time_duration totalTime () const;

        std::ostream& print (std::ostream& os) const;

      private:
        std::string n_;
        unsigned long c_;
        time_duration t_, last_, min_, max_;
        ptime s_;
    };

    std::ostream& operator<< (std::ostream& os, const TimeCounter& tc);

# ifdef HPP_ENABLE_BENCHMARK

/// \addtogroup hpp_util_logging
/// \{

/// \brief Define a new TimeCounter
#  define HPP_DEFINE_TIMECOUNTER(name)                              \
    ::hpp::debug::TimeCounter _##name##_timecounter_  (#name)
/// \brief Compute the time spent in the current scope.
#  define HPP_SCOPE_TIMECOUNTER(name)                               \
    ::hpp::debug::TimeCounter::Scope _##name##_scopetimecounter_    \
    (_##name##_timecounter_)
/// \brief Start a watch.
#  define HPP_START_TIMECOUNTER(name)                               \
    _##name##_timecounter_.start ()
/// \brief Stop a watch and save elapsed time.
#  define HPP_STOP_TIMECOUNTER(name)                                \
    _##name##_timecounter_.stop()
/// \brief Print last elapsed time to the logs.
#  define HPP_DISPLAY_LAST_TIMECOUNTER(name)                        \
    do {                                                            \
      using namespace hpp;                                          \
      using namespace ::hpp::debug;                                 \
      std::stringstream __ss;                                       \
      __ss << #name << " last: "                                    \
      << _##name##_timecounter_.last() <<  iendl;                   \
      logging.benchmark.write (__FILE__, __LINE__,                  \
                               __PRETTY_FUNCTION__, __ss);          \
    } while (0)
/// \brief Print min, max and mean time of the time measurements.
#  define HPP_DISPLAY_TIMECOUNTER(name)                             \
    do {                                                            \
      using namespace hpp;                                          \
      using namespace ::hpp::debug;                                 \
      std::stringstream __ss;                                       \
      __ss << _##name##_timecounter_ << iendl;                      \
      logging.benchmark.write (__FILE__, __LINE__,                  \
                               __PRETTY_FUNCTION__, __ss);          \
    } while (0)
/// \brief Reset a TimeCounter.
#  define HPP_RESET_TIMECOUNTER(name)                               \
    _##name##_timecounter_.reset();
/// \brief Stream (\c operator<<) to the output stream.
#  define HPP_STREAM_TIMECOUNTER(os, name)                          \
    os << _##name##_timecounter_
/// \}
# else // HPP_ENABLE_BENCHMARK
#  define HPP_DEFINE_TIMECOUNTER(name)                              \
    struct _##name##_EndWithSemiColon_{}
#  define HPP_SCOPE_TIMECOUNTER(name)
#  define HPP_START_TIMECOUNTER(name)
#  define HPP_STOP_TIMECOUNTER(name)
#  define HPP_DISPLAY_LAST_TIMECOUNTER(name)
#  define HPP_DISPLAY_TIMECOUNTER(name)
#  define HPP_RESET_TIMECOUNTER(name)
#  define HPP_STREAM_TIMECOUNTER(os, name)                          \
    os
# endif // HPP_ENABLE_BENCHMARK

# define HPP_STOP_AND_DISPLAY_TIMECOUNTER(name)                     \
   HPP_STOP_TIMECOUNTER(name);                                      \
   HPP_DISPLAY_TIMECOUNTER(name)

  } // end of namespace debug
} // end of namespace hpp

#endif // HPP_UTIL_TIMER_HH
