//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp-walkgen
// hpp-walkgen is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-walkgen is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-walkgen  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WALKGEN_FOOT_TRAJECTORY_HH
# define HPP_WALKGEN_FOOT_TRAJECTORY_HH

# include <stdexcept>
# include <hpp/core/path.hh>
# include <hpp/walkgen/foot-print.hh>

namespace hpp {
  namespace walkgen {
    DevicePtr_t createFootDevice ();

    /// A step as a trajectory of a flying foot
    ///
    /// This class derives from Path as a trajectory of a rigid body object.
    class Step : public Path
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// Create a step and return a shared pointer
      static StepPtr_t create (const FootPrint& start, const FootPrint& end,
			       const value_type& stepLow,
			       const value_type& stepHigh,
			       const value_type& duration);
      virtual PathPtr_t copy () const
      {
	return StepPtr_t (new Step (*this));
      }

      virtual PathPtr_t copy (const ConstraintSetPtr_t&) const
      {
	throw std::logic_error ("Providing constraints to foot trajectory is "
				"not supported.");
      }

      virtual ~Step () throw () {}
    protected:
      Step (const FootPrint& start, const FootPrint& end,
	    const value_type& stepLow, const value_type& stepHigh,
	    const value_type& duration);

      Step (const Step& step) : Path (step),
	h0_ (step.h0_), h1_ (step.h1_), h2_ (step.h2_), h3_ (step.h3_),
	v0_ (step.v0_), v1_ (step.v1_), v2_ (step.v2_), v3_ (step.v3_),
	initialOrientation_ (step.initialOrientation_), angle_ (step.angle_),
	omega0_ (step.omega0_), omega1_ (step.omega1_), omega2_ (step.omega2_),
	omega3_ (step.omega3_), initial_ (step.initial_), final_ (step.final_)

      {
      }

      virtual Configuration_t initial () const
      {
	return initial_;
      }

      virtual Configuration_t end () const
      {
	return final_;
      }

      virtual bool impl_compute (ConfigurationOut_t configuration,
				 value_type t) const;

      virtual std::ostream& print (std::ostream &os) const
      {
	os << "Step between:" << std::endl;
	os << "  (" << h0_.transpose () << ") with orientation "
	   << atan2 (initialOrientation_ [1], initialOrientation_ [0])
	   << std::endl;
	os << "and" << std::endl;
	os << "  (" << (h0_+h1_+h2_+h3_).transpose () << ") with orientation "
	   << atan2 (initialOrientation_ [1], initialOrientation_ [0]) + angle_
	   << std::endl;
	os << " with height " << v0_+v1_+v2_+v3_ << std::endl;
	return os;
      }

    private:
      value_type computeAngle (const FootPrint& start, const FootPrint& end)
	const;
      /// Coefficients of the horizontal polynomial motion
      vector2_t h0_, h1_, h2_, h3_;
      /// Coefficients of the vertical polynomial motion
      value_type v0_, v1_, v2_, v3_;
      /// Store initial orientation of the foot
      vector2_t initialOrientation_;
      /// Angle between start foot and end foot orientations
      value_type angle_;
      /// Coefficients of the polynomial orientation
      value_type omega0_, omega1_, omega2_, omega3_;
      /// initial configuration
      Configuration_t initial_;
      /// final configuration
      Configuration_t final_;      
    }; // class Step

    /// Static trajectory of a support foot
    ///
    /// This class derives from Path as a trajectory of a rigid body object.
    class SupportFoot : public Path
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// Create a support foot and return shared pointer
      static SupportFootPtr_t create (const FootPrint& position,
				      const value_type& footHeight,
				      const value_type& duration);

      virtual PathPtr_t copy () const
      {
	return SupportFootPtr_t (new SupportFoot (*this));
      }

      virtual PathPtr_t copy (const ConstraintSetPtr_t&) const
      {
	throw std::logic_error ("Providing constraints to foot trajectory is "
				"not supported.");
      }
      virtual ~SupportFoot () throw () {}
    protected:
      SupportFoot (const FootPrint& position, const value_type& footHeight,
		   const value_type& duration);
      SupportFoot (const SupportFoot& sp) : Path (sp),
					    h0_ (sp.h0_), v0_ (sp.v0_),
					    orientation_ (sp.orientation_),
					    configuration_ (sp.configuration_)
      {
      }
      virtual Configuration_t initial () const
      {
	return configuration_;
      }

      virtual Configuration_t end () const
      {
	return configuration_;
      }

      virtual bool impl_compute (ConfigurationOut_t configuration,
				 value_type t) const;
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "Support foot:" << std::endl;
	os << "  (" << h0_.transpose () << ") with orientation "
	   << atan2 (orientation_ [1], orientation_ [0]) << std::endl;
	return os;
      }
    private:
      /// Position of the support foot in horizontal plane
      vector2_t h0_;
      /// Height of the support foot
      value_type v0_;
      /// Store initial orientation of the foot
      vector2_t orientation_;
      /// Configuration of the foot
      Configuration_t configuration_;
    }; // class SupportFoot

  } // namespace walkgen
} // namespace hpp
#endif // HPP_WALKGEN_FOOT_TRAJECTORY_HH
