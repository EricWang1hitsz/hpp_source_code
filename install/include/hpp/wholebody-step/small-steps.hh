///
/// Copyright (c) 2015 CNRS
/// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_WHOLEBODY_STEP_SMALL_STEPS_HH
# define HPP_WHOLEBODY_STEP_SMALL_STEPS_HH

# include <hpp/core/path-optimizer.hh>
# include <hpp/walkgen/foot-print.hh>
# include <hpp/wholebody-step/fwd.hh>
# include <hpp/wholebody-step/time-dependant.hh>

namespace hpp {
  namespace wholebodyStep {
    /// Plan a sliding path for a humanoid robot and generate a walk motion
    /// along this path.
    ///
    /// \note quasi-static sliding constraints should be inserted in the problem
    class HPP_WHOLEBODY_STEP_DLLAPI SmallSteps : public core::PathOptimizer
    {
    public:
      typedef walkgen::FootPrints_t FootPrints_t;
      typedef walkgen::Times_t Times_t;
      typedef std::map <value_type, value_type> TimeToParameterMap_t;

      /// Compute parameter along initial path with respect to time.
      struct PiecewiseAffine
      {
        typedef SmallSteps::TimeToParameterMap_t Pairs_t;

        value_type operator () (const value_type& t) const;
        /// Add a pair (time, parameter)
        void addPair (const value_type& t, const value_type value);

        Pairs_t pairs_;
      }; // class PiecewiseAffine

      /// Create instance and return shared pointer
      ///
      /// \param problem problem to solve,
      /// \param delegate path planner that will solve the initial path planning
      ///        problem for the sliding robot.
      /// \note the roadmap of this planner and of the delegate are the same.
      static SmallStepsPtr_t create (const Problem& problem);
      /// Call implementation of delegate path planner
      virtual PathVectorPtr_t optimize (const PathVectorPtr_t& path);

      enum HandConstraintType {
        FromStart,
        UntilEnd,
        All
      };
      struct HandConstraint {
        bool active;
        bool optional;
        HandConstraintType type;
        value_type abscissa;

        HandConstraint () :
          active (false), optional (false), type (All), abscissa (-1) {}
      };

      HandConstraint leftHand_, rightHand_;

    protected:
      /// Constructor with roadmap
      SmallSteps (const Problem& problem);
      /// Store weak pointer to itself
      void init (const SmallStepsWkPtr_t& weak);
    private:

      void getStepParameters (const PathVectorPtr_t& path);
      FootPrint footPrintAtParam (const PathPtr_t& path, value_type s, bool right) const;
      Times_t buildInitialTimes (const std::size_t& p);
      /// Build the optimized path from the original path, the step parameters
      /// and times (contained in TTP), the COM trajectory, both foot trajectory
      /// (contained in the member pg_)
      PathVectorPtr_t generateOptimizedPath (const PathVectorPtr_t path,
          const PiecewiseAffine& TTP, CubicBSplinePtr_t com,
          value_type comHeight, value_type ankleShift,
          value_type& failureParameter);
      ConstraintSetPtr_t copyNonStabilityConstraints (ConstraintSetPtr_t orig)
        const;
      /// \return false on error.
      bool narrowAtTime (const value_type& t, const PathPtr_t& path,
          const PiecewiseAffine& param, Times_t& times);

      HumanoidRobotPtr_t robot_;
      value_type minStepLength_;
      value_type maxStepLength_;
      value_type defaultStepHeight_;
      value_type defaultDoubleSupportTime_;
      value_type defaultSingleSupportTime_;
      // Transition time of the ZMP between initial COM position to first step.
      value_type defaultInitializationTime_;
      SplineBasedPtr_t pg_;
      // Parameters along the input path at which positions of the feet are
      // chosen as foot print
      std::vector <value_type> stepParameters_;
      FootPrints_t footPrints_;
      std::vector <bool> footPrintsIsRight_;
      SmallStepsWkPtr_t weakPtr_;
    }; // class SmallSteps
  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_SMALL_STEPS_HH
