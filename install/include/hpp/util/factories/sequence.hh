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

#ifndef HPP_UTIL_FACTORIES_SEQUENCE_HH
# define HPP_UTIL_FACTORIES_SEQUENCE_HH

# include "hpp/util/parser.hh"

# include <vector>

namespace hpp {
  namespace util {
    namespace parser {
      /// \addtogroup factories
      /// \{

      /// \brief  Factory parsing sequence of values.
      /// \tparam ValueType one of (bool, int, unsigned int, double, float)
      ///
      /// A std::vector is built from a sequence of values separeted by
      /// white spaces.
      template <typename ValueType>
      class SequenceFactory : public ObjectFactory {
        public:
          typedef std::vector <ValueType> OutType;
          SequenceFactory (ObjectFactory* parent, const XMLElement* element, const unsigned int nbValue = 0) :
            ObjectFactory (parent, element), size_ (nbValue)
        {}

          virtual void addTextChild (const XMLText* text);

          const OutType& values () const
          {
            return values_;
          }

          SequenceFactory (const std::string& tagName, ObjectFactory* parent = NULL)
            : ObjectFactory (tagName, parent)
          {}

          void values (const OutType& v)
          {
            values_ = v;
          }

        protected:
          virtual void impl_write (XMLElement* element) const;

        private:
          std::vector <ValueType> values_;
          unsigned int size_;
      };

      /// \}
    } // namespace parser
  } // namespace manipulation
} // namespace hpp

#endif // HPP_UTIL_FACTORIES_SEQUENCE_HH
