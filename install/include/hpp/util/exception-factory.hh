//
// Copyright (C) 2016 by Joseph Mirabel, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_EXCEPTION_FACTORY_HH
# define HPP_UTIL_EXCEPTION_FACTORY_HH

# include <sstream>

# include <hpp/util/config.hh>

namespace hpp
{
  /// \cond
  struct ThrowException {};

  template <typename exception> struct ExceptionFactory;

  namespace internal {
    template <typename exception, typename In>
      struct conditional_insertion_operator {
        typedef ExceptionFactory<exception>& type;

        static inline type run(ExceptionFactory<exception>& be, const In& t) { be.ss << t; return be; }
      };
  }
  /// \endcond

  /// \brief Class to ease exception creation.
  ///
  /// You can use equivalently
  /// \code
  ///   throw ::hpp::ExceptionFactory<std::runtime_error>() << "message" << variable << ::hpp::ThrowException();
  /// \endcode
  /// or
  /// \code
  ///   HPP_THROW(std::runtime_error>, "message" << variable);
  /// \endcode
  template <typename exception>
    struct HPP_UTIL_DLLAPI ExceptionFactory
    {
      std::stringstream ss;

      template <typename T> inline
        typename internal::conditional_insertion_operator<exception, T>::type
        operator<< (const T& t) {
        return internal::conditional_insertion_operator<exception, T>::run (*this, t);
      }
    };

  /// \cond
  // ----------------------------------------
  // ExceptionFactory - template specialization
  // ----------------------------------------
  namespace internal {
    template <typename exception>
      struct conditional_insertion_operator<exception, ThrowException> {
        typedef exception type;

        static inline type run(ExceptionFactory<exception>& be, const ThrowException&) { return exception(be.ss.str().c_str()); }
      };
  }
  /// \endcond
} // end of namespace hpp.

/// \addtogroup hpp_util_exceptions
/// \{

/// \brief Throw an exception of type using MSG as a string stream
/// \code
///   HPP_THROW(std::runtime_error, "message" << variable);
/// \endcode
# define HPP_THROW(TYPE, MSG)			\
  throw ::hpp::ExceptionFactory<TYPE>() << MSG << ::hpp::ThrowException()

/// \brief Throw an exception of type using MSG as a string stream
/// \code
///   HPP_THROW_WITH_LINEINFO(std::runtime_error>, "message" << variable);
/// \endcode
# define HPP_THROW_WITH_LINEINFO(TYPE, MSG)			\
  HPP_THROW(TYPE,MSG << " at " << __FILE__ << ":" << __LINE__)

/// \}

#endif //! HPP_UTIL_EXCEPTION_FACTORY_HH
