// Copyright (C) 2008, 2009 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_INDENT_HH
# define HPP_UTIL_INDENT_HH
# include <iosfwd>

# include <hpp/util/config.hh>

namespace hpp
{
  /// \brief The current indentation level for \a o.
  HPP_UTIL_DLLAPI long& indent (std::ostream& o);

  /// \brief Increment the indentation.
  HPP_UTIL_DLLAPI std::ostream& incindent (std::ostream& o);

  /// \brief Decrement the indentation.
  HPP_UTIL_DLLAPI std::ostream& decindent (std::ostream& o);

  /// \brief Reset the indentation.
  HPP_UTIL_DLLAPI std::ostream& resetindent (std::ostream& o);

  /// \brief Print an end of line, then set the indentation.
  HPP_UTIL_DLLAPI std::ostream& iendl (std::ostream& o);

  /// \brief Increment the indentation, print an end of line,
  /// and set the indentation.
  HPP_UTIL_DLLAPI std::ostream& incendl (std::ostream& o);

  /// \brief  Decrement the indentation, print an end of line,
  /// and set the indentation.
  HPP_UTIL_DLLAPI std::ostream& decendl (std::ostream& o);

} // end of namespace hpp.

#endif // !HPP_UTIL_INDENT_HH
