// Copyright (C) 2008-2018 LAAS-CNRS, JRL AIST-CNRS, INRIA.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef HPP_CONSTRAINTS_CONFIG_HH
# define HPP_CONSTRAINTS_CONFIG_HH

// Package version (header).
# define HPP_CONSTRAINTS_VERSION "4.6.0-36-gcfb7"
# define HPP_CONSTRAINTS_MAJOR_VERSION 4
# define HPP_CONSTRAINTS_MINOR_VERSION 6
# define HPP_CONSTRAINTS_PATCH_VERSION 0

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
#  define HPP_CONSTRAINTS_DLLIMPORT __declspec(dllimport)
#  define HPP_CONSTRAINTS_DLLEXPORT __declspec(dllexport)
#  define HPP_CONSTRAINTS_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define HPP_CONSTRAINTS_DLLIMPORT __attribute__ ((visibility("default")))
#   define HPP_CONSTRAINTS_DLLEXPORT __attribute__ ((visibility("default")))
#   define HPP_CONSTRAINTS_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define HPP_CONSTRAINTS_DLLIMPORT
#   define HPP_CONSTRAINTS_DLLEXPORT
#   define HPP_CONSTRAINTS_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef HPP_CONSTRAINTS_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HPP_CONSTRAINTS_DLLAPI
#  define HPP_CONSTRAINTS_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef hpp_constraints_EXPORTS
#   define HPP_CONSTRAINTS_DLLAPI HPP_CONSTRAINTS_DLLEXPORT
#  else
#   define HPP_CONSTRAINTS_DLLAPI HPP_CONSTRAINTS_DLLIMPORT
#  endif // HPP_CONSTRAINTS_EXPORTS
#  define HPP_CONSTRAINTS_LOCAL HPP_CONSTRAINTS_DLLLOCAL
# endif // HPP_CONSTRAINTS_STATIC
#endif //! HPP_CONSTRAINTS_CONFIG_HH