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

#ifndef GEPETTO_VIEWER_CORBA_CONFIG_HH
# define GEPETTO_VIEWER_CORBA_CONFIG_HH

// Package version (header).
# define GEPETTO_VIEWER_CORBA_VERSION "5.3.0-4-gdb21"
# define GEPETTO_VIEWER_CORBA_MAJOR_VERSION 5
# define GEPETTO_VIEWER_CORBA_MINOR_VERSION 3
# define GEPETTO_VIEWER_CORBA_PATCH_VERSION 0

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
#  define GEPETTO_VIEWER_CORBA_DLLIMPORT __declspec(dllimport)
#  define GEPETTO_VIEWER_CORBA_DLLEXPORT __declspec(dllexport)
#  define GEPETTO_VIEWER_CORBA_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define GEPETTO_VIEWER_CORBA_DLLIMPORT __attribute__ ((visibility("default")))
#   define GEPETTO_VIEWER_CORBA_DLLEXPORT __attribute__ ((visibility("default")))
#   define GEPETTO_VIEWER_CORBA_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define GEPETTO_VIEWER_CORBA_DLLIMPORT
#   define GEPETTO_VIEWER_CORBA_DLLEXPORT
#   define GEPETTO_VIEWER_CORBA_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef GEPETTO_VIEWER_CORBA_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define GEPETTO_VIEWER_CORBA_DLLAPI
#  define GEPETTO_VIEWER_CORBA_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef gepetto_viewer_corba_EXPORTS
#   define GEPETTO_VIEWER_CORBA_DLLAPI GEPETTO_VIEWER_CORBA_DLLEXPORT
#  else
#   define GEPETTO_VIEWER_CORBA_DLLAPI GEPETTO_VIEWER_CORBA_DLLIMPORT
#  endif // GEPETTO_VIEWER_CORBA_EXPORTS
#  define GEPETTO_VIEWER_CORBA_LOCAL GEPETTO_VIEWER_CORBA_DLLLOCAL
# endif // GEPETTO_VIEWER_CORBA_STATIC
#endif //! GEPETTO_VIEWER_CORBA_CONFIG_HH
