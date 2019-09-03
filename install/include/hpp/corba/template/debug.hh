// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of hpp-template-corba
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBA_DEBUG_HH
#define HPP_CORBA_DEBUG_HH

# if 0

# include <hpp/util/debug.hh>
#  define hppCorbaDebug(statement) hppDebug(statement)
#  define hppCorbaDebugStatement(statement) hppDebugStatement(statement)
#  define hppCorbaDout(channel, data) hppDout(channel, data)
#  define hppCorbaDoutFatal(channel, data) hppDoutFatal(channel, data)

#else

#  define hppCorbaDebug(statement)
#  define hppCorbaDebugStatement(statement)
#  define hppCorbaDout(channel, data) \
  std::cout << "CORBA: "  << data << std::endl
#  define hppCorbaDoutFatal(channel, data)

#endif

#endif //HPP_CORBA_DEBUG_HH
