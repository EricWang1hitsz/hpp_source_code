// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation-urdf.
// hpp-manipulation-urdf is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-urdf is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-urdf. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_UTIL_FACTORIES_IGNORETAG_HH
# define HPP_UTIL_FACTORIES_IGNORETAG_HH

# include "hpp/util/parser.hh"

namespace hpp {
  namespace util {
    namespace parser {
      /// \addtogroup factories
      /// \{

      /// Class used to ignore a tag.
      /// If the parser knows it should ignore a tag, no warning will be
      /// printed in the logs. Moreover, its children won't be parsed.
      class IgnoreTagFactory : public ObjectFactory {
        public:
          IgnoreTagFactory (ObjectFactory* parent, const XMLElement* element) :
            ObjectFactory (parent, element) {}

          bool init ()
          {
            return false;
          }

          void impl_write (XMLElement* element) const
          {
            element->InsertEndChild (XMLComment ("This tag was ignored"));
          }
      };

      /// \}
    } // namespace parser
  } // namespace manipulation
} // namespace hpp

#endif // HPP_UTIL_FACTORIES_IGNORETAG_HH
