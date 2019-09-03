// Copyright (c) 2015 CNRS
// Authors: Joseph Mirabel
//
//
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

#ifndef HPP_WHOLEBODY_STEP_TIME_DEPENDANT_PATH_HH
# define HPP_WHOLEBODY_STEP_TIME_DEPENDANT_PATH_HH

# include <hpp/core/path.hh>
# include <hpp/wholebody-step/config.hh>
# include <hpp/wholebody-step/time-dependant.hh>

namespace hpp {
  namespace wholebodyStep {
    typedef constraints::Implicit Implicit;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    class HPP_WHOLEBODY_STEP_DLLAPI TimeDependantPath : public core::Path
    {
      public:
        static TimeDependantPathPtr_t create (const core::PathPtr_t path)
        {
          TimeDependantPathPtr_t ptr (new TimeDependantPath (path));
          ptr->init (ptr);
          return ptr;
        }

        static TimeDependantPathPtr_t create (const core::PathPtr_t path,
            const ConstraintSetPtr_t& c)
        {
          TimeDependantPathPtr_t ptr (new TimeDependantPath (path, c));
          ptr->init (ptr);
          return ptr;
        }

        static TimeDependantPathPtr_t createCopy (const TimeDependantPath& other)
        {
          TimeDependantPathPtr_t ptr (new TimeDependantPath (other));
          ptr->init (ptr);
          return ptr;
        }

        static TimeDependantPathPtr_t createCopy (const TimeDependantPath& other,
            const ConstraintSetPtr_t& c)
        {
          TimeDependantPathPtr_t ptr (new TimeDependantPath (other, c));
          ptr->init (ptr);
          return ptr;
        }

        void add (const TimeDependant& td)
        {
          tds_.push_back (td);
        }

        virtual ~TimeDependantPath () throw () {}

        Configuration_t initial () const {
          return path_->initial ();
        }

        Configuration_t end () const {
          return path_->end ();
        }

        core::PathPtr_t copy () const {
          return createCopy (*this);
        }

        core::PathPtr_t copy (const ConstraintSetPtr_t& c) const {
          return createCopy (*this, c);
        }

        void setAffineTransform (const value_type& a, const value_type& b) {
          a_ = a;
          b_ = b;
        }

        void updateAbscissa (value_type t) const
        {
          const value_type y = a_*t + b_;
          ConfigProjectorPtr_t cp;
          if (constraints()) cp = constraints()->configProjector ();
          for (TimeDependants_t::const_iterator it = tds_.begin ();
              it != tds_.end (); ++it) {
            it->rhsAbscissa (y);
            ImplicitPtr_t nm = HPP_DYNAMIC_PTR_CAST(Implicit, it->eq_);
            if (cp && nm) {
              cp->rightHandSide(nm, nm->rightHandSide());
            }
          }
        }

      protected:
        TimeDependantPath (const core::PathPtr_t path) :
          Path (path->timeRange (), path->outputSize (),
              path->outputDerivativeSize ()),
          path_ (path->copy ()), a_ (1), b_(0)
        {}

        TimeDependantPath (const core::PathPtr_t path, const ConstraintSetPtr_t& c) :
          Path (path->timeRange (), path->outputSize (),
              path->outputDerivativeSize ()),
          path_ (path->copy ()), a_ (1), b_(0)
        {
          constraints (c);
        }

        TimeDependantPath (const TimeDependantPath &other) :
          Path (other), path_ (other.path_->copy ()),
          tds_ (other.tds_), a_ (other.a_), b_(other.b_)
        {}

        TimeDependantPath (const TimeDependantPath &other, const ConstraintSetPtr_t& c) :
          Path (other), path_ (other.path_->copy ()),
          a_ (other.a_), b_(other.b_)
        {
          constraints (c);
        }

        virtual bool impl_compute (ConfigurationOut_t config, value_type t) const
        {
          updateAbscissa (t);
          return (*path_) (config, t);
        }

        void init (const TimeDependantPathPtr_t& self)
        {
          Path::init (self);
        }

        virtual std::ostream& print (std::ostream& os) const {
          return os << "TimeDependantPath: " << *path_;
        }

      private:
        typedef std::vector <TimeDependant> TimeDependants_t;

        core::PathPtr_t path_;
        TimeDependants_t tds_;
        value_type a_, b_;
    }; // class TimeDependantPath
  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_TIME_DEPENDANT_PATH_HH
