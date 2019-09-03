/// Copyright (c) 2015 CNRS
/// Authors: Joseph Mirabel
///
///
// This file is part of hpp-wholebody-step.
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-wholebody-step-planner. If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WHOLEBODY_STEP_TIME_DEPENDANT_HH
# define HPP_WHOLEBODY_STEP_TIME_DEPENDANT_HH

# include <hpp/constraints/implicit.hh>

# include <hpp/wholebody-step/fwd.hh>

namespace hpp {
  namespace wholebodyStep {
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    struct HPP_WHOLEBODY_STEP_DLLAPI RightHandSideFunctor {
      virtual void operator() (vectorOut_t output, const value_type& input)
        const = 0;
    };
    typedef boost::shared_ptr <const RightHandSideFunctor> RightHandSideFunctorPtr_t;

    struct HPP_WHOLEBODY_STEP_DLLAPI TimeDependant
    {
      void rhsAbscissa (const value_type s) const
      {
        (*rhsFunc_) (eq_->rightHandSide(), s);
      }

      TimeDependant (const ImplicitPtr_t& eq,
          const RightHandSideFunctorPtr_t rhs):
        eq_ (eq), rhsFunc_ (rhs)
      {}

      TimeDependant (const TimeDependant& other) :
        eq_ (other.eq_), rhsFunc_ (other.rhsFunc_)
      {}

      ImplicitPtr_t eq_;
      RightHandSideFunctorPtr_t rhsFunc_;
    }; // class TimeDependant
  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_TIME_DEPENDANT_HH
