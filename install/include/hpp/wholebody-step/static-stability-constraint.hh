///
/// Copyright (c) 2014 CNRS
/// Authors: Florent Lamiraux
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

#ifndef HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH
# define HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH

# include <hpp/wholebody-step/fwd.hh>
# include <hpp/wholebody-step/config.hh>
# include <hpp/wholebody-step/deprecated.hh>

namespace hpp {
  namespace wholebodyStep {
    extern const std::string STABILITY_CONTEXT;
    typedef constraints::Implicit Implicit;
    typedef constraints::ImplicitPtr_t ImplicitPtr_t;
    /// Create quasi-static stability constraints
    /// \param robot the robot,
    /// \param comc a hpp::pinocchio::CenterOfMassComputation that handle
    ///        COM computations.
    /// \param leftAnkle left ankle joint,
    /// \param rightAnkle right ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    ///
    /// The constraints make the feet of the robot slide on a horizontal ground
    /// and the center of mass project at a constant position with respect to
    /// the feet. Five constraints are returned:
    /// \li relative position of the center of mass (as defined with comc)
    ///     in the left ankle frame (dimension 3),
    /// \li relative orientation of the feet (dimension 3),
    /// \li relative position of the feet (dimension 3),
    /// \li orientation of the left foot (dimension 2),
    /// \li position of the left foot (dimension 1).
    ///
    /// All constraints are returned along with the
    /// hpp::core::ComparisonType::createDefault()
    std::vector <ImplicitPtr_t> createSlidingStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration) HPP_WHOLEBODY_STEP_DEPRECATED;

    /// Create quasi-static stability constraints
    /// The center of mass takes into account all the joints of the robot.
    /// \param robot, leftAnkle, rightAnkle, configuration
    ///        see createSlidingStabilityConstraint(const DevicePtr_t&,
    ///         const JointPtr_t&, const JointPtr_t&, ConfigurationIn_t)
    std::vector <ImplicitPtr_t> createSlidingStabilityConstraint
    (const DevicePtr_t& robot, const JointPtr_t& leftAnkle, 
     const JointPtr_t& rightAnkle, ConfigurationIn_t configuration)
      HPP_WHOLEBODY_STEP_DEPRECATED;

    /// Create quasi-static stability complementary constraints
    /// \param robot the robot,
    /// \param comc a hpp::pinocchio::CenterOfMassComputation that handle
    ///        COM computations.
    /// \param leftAnkle left ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    ///
    /// Used with the quasi-static stability constraints, the constraints 
    /// make the feet and the center of mass of the robot at a constant 
    /// position. Two constraints are returned:
    /// \li orientation of the left foot (dimension 1),
    /// \li position of the left foot (dimension 2).
    ///
    /// All constraints are returned along with the
    /// hpp::core::ComparisonType::createDefault()
    std::vector <ImplicitPtr_t>
    createSlidingStabilityConstraintComplement
    (const DevicePtr_t& robot, const JointPtr_t& leftAnkle,
     ConfigurationIn_t configuration);

    /// Constraints that ensure that the COM is between the two ankles
    /// \param robot the robot,
    /// \param comc computation of the center of mass (whole robot or subtree)
    /// \param leftAnkle left ankle joint,
    /// \param rightAnkle right ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    /// \param sliding whether the robot can slide on the ground
    ///
    /// Create the following set of constraints:
    /// \li ComBetweenFeet constraints (dimension 3)
    /// \li orientation of the right foot around x and y (dimension 2)
    ///     if sliding, dimension 3 otherwise
    /// \li position of the right foot along z (dimension 1) if sliding
    ///     dimension 3 otherwise,
    /// \li orientation of the left foot around x and y (dimension 2) if
    ///     sliding, dimension 3 otherwise
    /// \li position of the left foot along z (dimension 1) if sliding,
    ///     dimension 3 otherwise.
    std::vector <ImplicitPtr_t> createAlignedCOMStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration, bool sliding = true);

    /// Create quasi-static stability constraints
    /// \param robot the robot,
    /// \param comc computation of the center of mass (whole robot or subtree)
    /// \param leftAnkle left ankle joint,
    /// \param rightAnkle right ankle joint,
    /// \param configuration the configuration of the robot satisfying
    ///        the constraint,
    /// \param sliding whether the robot can slide on the ground
    ///
    /// If sliding is true, the constraints make the feet of the robot lie on
    /// a horizontal ground and the center of mass project at a constant
    /// position with respect to the feet. Five constraints are returned:
    /// \li relative position of the center of mass (as defined with comc)
    ///     in the left ankle frame (dimension 3),
    /// \li relative orientation of the feet (dimension 3),
    /// \li relative position of the feet (dimension 3),
    /// \li orientation of the left foot (dimension 2),
    /// \li position of the left foot (dimension 1).
    ///
    /// If sliding is false, 3 constraints are returned
    ///
    /// \li relative position of the center of mass (as defined with comc)
    ///     in the left ankle frame (dimension 3),
    /// \li absolute position and orientation of left foot,
    /// \li absolute position and orientation of right foot.
    ///
    /// All constraints are returned along with the
    /// hpp::core::ComparisonType::createDefault()
    std::vector <ImplicitPtr_t> createStaticStabilityConstraint
    (const DevicePtr_t& robot, const CenterOfMassComputationPtr_t& comc,
     const JointPtr_t& leftAnkle, const JointPtr_t& rightAnkle,
     ConfigurationIn_t configuration, bool sliding);

  } // namespace wholebodyStep
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_STATIC_STABILITY_CONSTRAINT_HH
