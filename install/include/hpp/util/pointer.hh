// Copyright (C) 2013 by Antonio El Khoury, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_POINTER_HH
# define HPP_UTIL_POINTER_HH

# include <boost/smart_ptr.hpp>

/// Defines the four types of smart pointers associated with type <tt>\a t</tt>.
/// If <tt>\a t</tt> is \c CMyClass then
///  - the type of a shared pointer to <tt>\a t</tt> is \c CMyClassShPtr
///  - the type of a weak pointer to <tt>\a t</tt> is \c CMyClassWkPtr_t
///  - the type of a shared pointer to <tt>\a t const</tt> is
///    \c CMyClassConstShPtr
///  - the type of a weak pointer to <tt>\a t const</tt> is
///    \c CMyClassConstWkPtr_t

# define HPP_POINTER_DEFS(t)		 \
  typedef boost::weak_ptr <t> t##WkPtr_t;		 \
  typedef boost::weak_ptr <const t> t##ConstWkPtr_t;	 \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

/** Makes a forward declaration of class <tt>\a t</tt> and of the four
*   types of shared pointers associated with it.
*/
# define HPP_PREDEF_CLASS(t)		\
  class t;					\
  HPP_POINTER_DEFS(t);			\
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

# define HPP_STATIC_PTR_CAST(t, x) boost::static_pointer_cast < t > (x)
# define HPP_DYNAMIC_PTR_CAST(t, x) boost::dynamic_pointer_cast < t > (x)
# ifndef NDEBUG
#  define HPP_STATIC_CAST_REF_CHECK(t, x)\
  try {\
    dynamic_cast < t& > (x);\
  } catch (const std::exception& exc) {\
      assert ("Cast in #t failed" && 0);\
  }
# else
#  define HPP_STATIC_CAST_REF_CHECK(t, x)
# endif // defined NDEBUG
#endif //! HPP_UTIL_POINTER_HH
