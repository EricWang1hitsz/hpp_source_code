// Copyright (C) 2010 by Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

/// \file

#ifndef HPP_UTIL_EXCEPTION_HH
# define HPP_UTIL_EXCEPTION_HH
# include <iosfwd>
# include <stdexcept>
# include <string>

# include <hpp/util/config.hh>

namespace hpp
{
  /// \brief Main exception class for HPP.
  ///
  /// All exceptions thrown in HPP must inherit this class.
  class HPP_UTIL_DLLAPI Exception : public std::exception
  {
  public:
    Exception (const std::string& message,
	       const std::string& file,
	       unsigned line) throw ();
    ~Exception () throw ();
    Exception (const Exception& exception) throw ();
    Exception& operator= (const Exception& exception) throw ();

    virtual const char* what () const throw ();

    /// \brief Display the exception on the specified output stream.
    ///
    /// \param o output stream used for display
    /// \return output stream
    virtual std::ostream& print (std::ostream& o) const throw ();
  private:
    std::string message_;
    std::string file_;
    unsigned line_;
  };

  /// \brief Override operator<< to handle exception display.
  ///
  /// \param o output stream used for display
  /// \param exception exception to be displayed
  /// \return output stream
  HPP_UTIL_DLLAPI std::ostream&
  operator<< (std::ostream& o, const Exception& exception);

} // end of namespace hpp.

/// \brief Launch a HPP exception.
# define HPP_THROW_EXCEPTION_(MSG)			\
  throw ::hpp::Exception (MSG, __FILE__, __LINE__)

/// \brief Launch a HPP exception (generic macro).
# define HPP_THROW_EXCEPTION(TYPE, MSG)		\
  throw TYPE (MSG, __FILE__, __LINE__)

/// \brief Define a custom exception.
# define HPP_MAKE_EXCEPTION(EXTRA_QUALIFIER, TYPE)	\
  class EXTRA_QUALIFIER TYPE : public ::hpp::Exception  \
  {							\
  public:						\
    TYPE (const std::string& message,			\
	  const std::string& file,			\
	  unsigned line) throw ()			\
      : ::hpp::Exception (message, file, line)		\
      {}						\
  }

/// \brief Define a custom exception without extra qualifier.
# define HPP_MAKE_EXCEPTION_NO_QUALIFIER(TYPE)		\
  class TYPE : public ::hpp::Exception			\
  {							\
  public:						\
    TYPE (const std::string& message,			\
	  const std::string& file,			\
	  unsigned line) throw ()			\
      : ::hpp::Exception (message, file, line)		\
      {}						\
  }

#endif //! HPP_UTIL_EXCEPTION_HH
