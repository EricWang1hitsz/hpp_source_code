// Copyright (C) 2010 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_ASSERTION_HH
# define HPP_UTIL_ASSERTION_HH
# include <boost/scope_exit.hpp>

# include <hpp/util/config.hh>
# include <hpp/util/exception.hh>

// If debug mode is disabled and assertions are not already
// disabled, disable them automatically.
# if (defined HPP_DEBUG) && (!defined HPP_ENABLE_ASSERTIONS)
#  define HPP_ENABLE_ASSERTIONS
# endif // (!defined HPP_DEBUG) && (!defined HPP_ENABLE_ASSERTIONS)

namespace hpp
{
  HPP_MAKE_EXCEPTION (HPP_UTIL_DLLAPI, AssertionError);
} // end of namespace hpp.

/// \brief Define HPP_ASSERT.
///
/// Throw an ::hpp::AssertionError if macro argument evaluates to
/// false.
# ifdef HPP_ENABLE_ASSERTIONS
#  define HPP_ASSERT(CONDITION)					\
  do {								\
    bool _x = (CONDITION);					\
    if (!_x)							\
      HPP_THROW_EXCEPTION					\
	(::hpp::AssertionError,					\
	 #CONDITION " evaluates to false");			\
  } while (0)
# else
#  define HPP_ASSERT(CONDITION)
# endif // HPP_ENABLE_ASSERTIONS

/// \brief Define macro for precondition checking.
# define HPP_PRECONDITION(CONDITION) HPP_ASSERT (CONDITION)

#endif //! HPP_UTIL_ASSERTION_HH
