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

#ifndef HPP_WALKGEN_SPLINE_BASED_HH
# define HPP_WALKGEN_SPLINE_BASED_HH

# include<Eigen/StdVector>

# include <roboptim/trajectory/cubic-b-spline.hh>
# include <hpp/walkgen/foot-print.hh>

namespace hpp {
  namespace walkgen {

    typedef roboptim::trajectory::Polynomial3 Polynomial3;
    typedef
#ifdef ROBOPTIM_TRAJECTORY_32
      roboptim::trajectory::CubicBSpline::basisPolynomialsVector_t
#else
      roboptim::trajectory::CubicBSpline::polynomials3vectors_t
#endif
    polynomials3vectors_t;
    typedef
#ifdef ROBOPTIM_TRAJECTORY_32
      roboptim::trajectory::CubicBSpline::basisPolynomials_t
#else
      roboptim::trajectory::CubicBSpline::polynomials3vector_t
#endif
    polynomials3vector_t;

    typedef Eigen::Matrix <value_type, 7, 1> vector7_t;

    /// Polynomial function of degree 3 restricted to an interval
    ///
    /// Represented by the values of the function taken at 7 equally spaced
    /// parameter values containing the boundaries of the interval of
    /// definition.
    class PiecewisePoly3
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      /// Constructor
      ///
      /// \param tmin, tmax, interval of definition
      /// \param polynomial polynomial of degree 3 defining the spline
      ///        basis function.
      PiecewisePoly3 (const value_type& tmin, const value_type& tmax,
		      const Polynomial3& polynomial) : lower (tmin),
						       upper (tmax)
      {
	value_type delta = (upper-lower)/6;
	values_ [0] = polynomial (lower);
	values_ [6] = polynomial (upper);
	value_type t = lower + delta;
	for (unsigned int i=1; i<6; ++i) {
	  values_ [i] = polynomial (t);
	  t += delta;
	}
      }
      /// Constructor for affine function
      ///
      /// \param tmin, tmax, interval of definition
      /// \param v0, v1 values at tmin and tmax respectively.
      PiecewisePoly3 (const value_type& tmin, const value_type& tmax,
		      const value_type& v0, const value_type& v1) :
	lower (tmin), upper (tmax)
      {
	value_type delta_p = (upper-lower)/6;
	value_type delta_v = (v1-v0)/6;
	values_ [0] = v0;
	values_ [6] = v1;
	value_type t = lower + delta_p;
	value_type v = v0 + delta_v;
	for (unsigned int i=1; i<6; ++i) {
	  values_ [i] = v;
	  t += delta_p;
	  v += delta_v;
	}
      }
      /// Constructor with 7 values
      PiecewisePoly3 (const value_type& tmin, const value_type& tmax,
		      const vector7_t& values) : lower (tmin), upper (tmax),
						 values_ (values)
      {
      }
      /// Empty constructor
      /// set all fields to Nan
      PiecewisePoly3 () : lower (sqrt (-1.)), upper (sqrt (-1.))
      {
	values_.fill (sqrt (-1.));
      }
      /// Get value at equally spaced parameter
      ///
      /// \param index index of the parameter between 0 and 6.
      const value_type& operator[] (const size_type& index) const
      {
	return values_ [index];
      }
      value_type lower;
      value_type upper;
    private:
      vector7_t values_;
    }; // class PiecewisePoly3


    /// Walk motion generator for humanoid legged robot
    ///
    /// This class computes the reference trajectory of the center of mass
    /// of a humanoid robot, given as input a list of time-stamped foot-prints.
    ///
    /// The trajectory of the center of mass is obtained by optimization for the
    /// linear so called table cart model.
    class HPP_WALKGEN_DLLAPI SplineBased
    {
    public:
      typedef std::vector <PiecewisePoly3> ZmpTrajectory_t;
      typedef std::vector <ZmpTrajectory_t> ZmpTrajectories_t;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      static value_type gravity;
      static size_type l;
      struct BoundaryCondition
      {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	BoundaryCondition (const value_type& time, const vector2_t& pos,
			   const vector2_t& vel) : t (time),
						   position (pos),
						   velocity (vel)
	{
	}
	value_type t;
	vector2_t position;
	vector2_t velocity;
      }; // struct BoundaryCondition
      typedef std::vector <BoundaryCondition,
			   Eigen::aligned_allocator <BoundaryCondition> >
	BoundaryConditions_t;
      typedef std::vector <value_type> StepHeights_t;
      /// Create instance and return shared pointer
      ///
      /// \param height height of the center of mass
      static SplineBasedPtr_t create (const value_type& height);

      /// set time sequence
      ///
      /// \param times sequence of times corresponding to support phases.
      void timeSequence (const Times_t& times);

      /// get time sequence
      ///
      /// \return sequence of times corresponding to support phases.
      const Times_t& timeSequence () const
      {
	return tau_;
      }

      /// set sequence of foot prints
      ///
      /// \param footPrints a vector of foot prints
      /// \note number of foot prints should be set after time sequence and
      /// should fit
      /// length of time sequence. If \f$p\f$ is the number of foot prints, the
      /// length of the time sequence should be equal to \f$2p-2\f$.
      /// See <a href="figures/walkgen.pdf"> this document</a> for details.
      /// \note Sequence of step heights is set to the default value.
      void footPrintSequence (const FootPrints_t& footPrints);

      /// get sequence of foot prints
      const FootPrints_t& footPrintSequence () const
      {
	return footPrints_;
      }

      /// Set default step height
      ///
      void defaultStepHeight (const value_type& stepHeight)
      {
	defaultStepHeight_ = stepHeight;
      }
      /// Get default step height
      ///
      const value_type& defaultStepHeight () const
      {
	return defaultStepHeight_;
      }

      /// Set sequence of step heights
      ///
      /// \param stepHeights sequence of step heights
      /// \note Sequence of foot heights should be set after the sequence of
      /// foot prints.
      void stepHeights (const StepHeights_t& stepHeights)
      {
	stepHeights_ = stepHeights;
      }
      /// Get sequence of step heights
      const StepHeights_t& stepHeights () const
      {
	return stepHeights_;
      }

      /// Specify zmpref boundary conditions
      ///
      /// \param init, end initial and end values of zmpref trajectory
      /// \note If not specified, the middle of the first (respectively last)
      ///       foot prints.
      /// \note setting step sequence discards zmp boundary conditions.
      void zmpRefBoundaryConditions (const vector2_t& init,
				     const vector2_t& end)
      {
	zmpRefInit_ = init;
	zmpRefEnd_ = end;
      }
      /// Add a boundary condition on center of mass trajectory
      ///
      /// \param t time when position of center of mass is constrained,
      /// \param position required position of the center of mass.
      /// \note setting time or step sequence discards boundary conditions.
      void add (const BoundaryCondition& boundaryCondition);

      /// Set initial position and velocity of center of mass
      /// \param position position of the center of mass at start \f$t=\tau_0\f$
      /// \param velocity velocity of the center of mass at start \f$t=\tau_0\f$
      void setInitialComState (const vector2_t& position,
			       const vector2_t& velocity);
      /// Set final position and velocity of center of mass
      /// \param position position of the center of mass at end
      ///        \f$t=\tau_{2p-3}\f$
      /// \param velocity velocity of the center of mass at start
      ///        \f$t=\tau_{2p-3}\f$
      void setEndComState (const vector2_t& position,
			   const vector2_t& velocity);

      /// Solve quadratic program and return resulting cubic B spline
      ///
      CubicBSplinePtr_t solve () const;

      /// Get trajectory of left foot
      ///
      /// \precond Method solve should have been called first
      const PathVectorPtr_t& leftFootTrajectory () const
      {
	return leftFootTraj_;
      }

      /// Get trajectory of right foot
      ///
      /// \precond Method solve should have been called first
      const PathVectorPtr_t& rightFootTrajectory () const
      {
	return rightFootTraj_;
      }

      DevicePtr_t leftFoot () const
      {
        return leftFoot_;
      }

      DevicePtr_t rightFoot () const
      {
        return rightFoot_;
      }

      /// Getter to representation of zmp ref abscissa
      const ZmpTrajectory_t& zmpRefx () const
      {
	return zmpRef0_;
      }
      /// Getter to representation of zmp ref abscissa
      const ZmpTrajectory_t& zmpRefy () const
      {
	return zmpRef1_;
      }
      /// Compute the cost of a com trajectory defined by way points
      value_type cost (const vector_t& controlPoints);

      /// Compute the cost of the current com trajectory
      value_type cost () const;

      static value_type integral (value_type lower,value_type upper,
				  const PiecewisePoly3& P1,
				  const PiecewisePoly3& P2);

      /// Get base functions of Zmp
      const ZmpTrajectories_t& zmpBasisFunctions () const
      {
	return Z_;
      }
    protected:
      /// Constructor
      ///
      /// \param height height of the center of mass
      SplineBased (const value_type& height);
      /// initialization
      /// Store weak pointer to instance
      void init (const SplineBasedWkPtr_t& self);
      /// Compute trajectory of the feet
      void computeFootTrajectory () const;
    private:
      void defineProblem () const;
      void buildPolynomialVector () const;
      value_type height_;
      SplineBasedWkPtr_t weakPtr_;
      mutable size_type m_;
      Times_t tau_;
      FootPrints_t footPrints_;
      StepHeights_t stepHeights_;
      value_type defaultStepHeight_;
      BoundaryConditions_t boundaryConditions_;
      vector2_t zmpRefInit_;
      vector2_t zmpRefEnd_;
      DevicePtr_t leftFoot_;
      DevicePtr_t rightFoot_;
      mutable CubicBSplinePtr_t comTrajectory_;
      mutable matrix_t H0_;
      mutable vector_t b0_, b1_;
      mutable matrix_t A0_;
      mutable vector_t c0_, c1_;
      mutable ZmpTrajectories_t Z_;
      mutable ZmpTrajectory_t zmpRef0_;
      mutable ZmpTrajectory_t zmpRef1_;
      mutable PathVectorPtr_t leftFootTraj_;
      mutable PathVectorPtr_t rightFootTraj_;
    }; // class SplineBased
  } // namespace walkgen
} // namespace hpp
#endif // HPP_WALKGEN_SPLINE_BASED_HH
