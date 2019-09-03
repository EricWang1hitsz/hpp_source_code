// Copyright (C) 2008, 2009 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_PORTABILITY_HH
# define HPP_PORTABILITY_HH

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define HPP_DLLIMPORT __declspec(dllimport)
#  define HPP_DLLEXPORT __declspec(dllexport)
#  define HPP_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define HPP_DLLIMPORT __attribute__ ((visibility("default")))
#   define HPP_DLLEXPORT __attribute__ ((visibility("default")))
#   define HPP_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define HPP_DLLIMPORT
#   define HPP_DLLEXPORT
#   define HPP_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__
#endif //! HPP_PORTABILITY_HH
