// Copyright 2010, Thomas Moulard, CNRS/AIST
//
// This file is part of hpp-util.
// hpp-util is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-util is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-util.  If not, see <http://www.gnu.org/licenses/>.

/**
   \mainpage

   The hpp-util library gathers technical tools for the HPP project.

   This package does not target specifically path planning or robotics.

   It provides:
   - logging mechanisms (different channels are linked to several outputs).
   - benchmark tools
   - quality assurance tools (contract programming tools
     and "smart" assertions).

   The Doxygen documentation has been written to be a reference document.
   For tutorials and more user-friendly documentation, see the
   <a href="http://github.com/laas/hpp-util/wiki">hpp-util wiki</a>.

   Bug reporting can be done through the
   <a href="http://github.com/laas/hpp-util/issues">github ticketing
   system</a>.

   \defgroup hpp_util_debugging Macros for debugging

   \defgroup hpp_util_logging Macros for logging

   \defgroup hpp_util_exceptions Macros for \c std::exception

   It eases throwing exceptions built from string stream. You can use equivalently
   \code
     HPP_THROW(std::runtime_error, "message" << variable);
   \endcode
   or
   \code
     HPP_THROW_WITH_LINEINFO(std::runtime_error, "message" << variable);
   \endcode
   or
   \code
     throw ::hpp::ExceptionFactory<std::runtime_error>() << "message" << variable << ::hpp::ThrowException();
   \endcode
*/
