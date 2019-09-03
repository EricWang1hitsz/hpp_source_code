// Copyright (C) 2008, 2009 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_VERSION_HH
# define HPP_UTIL_VERSION_HH
# include <hpp/util/config.hh>

// Compare header version to library version.
# define HPP_UTIL_CHECK()			\
  ::hpp::util::checkVersion (HPP_UTIL_VERSION)

namespace hpp
{
  namespace util
  {
    // Package version (compiled).
    HPP_UTIL_DLLAPI extern const char* version;

    // Check that header and library version are compatible.
    //
    // Return 0 if equal, 1 if library is newer and -1 if library is
    // older. The last possibility is an error and program should be
    // terminated in this case.
    HPP_UTIL_DLLAPI int checkVersion (const char* header_version);
  } // end of namespace util.
} // end of namespace hpp.

#endif //! HPP_UTIL_VERSION_HH
