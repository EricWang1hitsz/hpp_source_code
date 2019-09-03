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

#ifndef HPP_WALKGEN_FWD_HH
# define HPP_WALKGEN_FWD_HH

# include <vector>
# include <Eigen/Core>
# include <hpp/core/fwd.hh>
# include <roboptim/trajectory/fwd.hh>

namespace hpp {
  namespace walkgen {
    typedef roboptim::trajectory::CubicBSpline CubicBSpline;
    typedef boost::shared_ptr <CubicBSpline> CubicBSplinePtr_t;
    HPP_PREDEF_CLASS (SplineBased);
    typedef boost::shared_ptr <SplineBased> SplineBasedPtr_t;
    HPP_PREDEF_CLASS (Step);
    typedef boost::shared_ptr <Step> StepPtr_t;
    HPP_PREDEF_CLASS (SupportFoot);
    typedef boost::shared_ptr <SupportFoot> SupportFootPtr_t;
    class FootPrint;

    typedef double value_type;
    typedef Eigen::Matrix <value_type, Eigen::Dynamic, 1> vector_t;
    typedef Eigen::Matrix<value_type, Eigen::Dynamic, Eigen::Dynamic> matrix_t;
    typedef matrix_t::Index size_type;
    typedef std::vector <value_type> Times_t;
    typedef Eigen::Matrix <value_type, 2, 1> vector2_t;
    typedef Eigen::Matrix <value_type, 1, 1> vector1_t;

    typedef core::Transform3f Transform3f;
    typedef core::Path Path;
    typedef core::PathPtr_t PathPtr_t;
    typedef core::PathVectorPtr_t PathVectorPtr_t;
    typedef core::PathVector PathVector;
    typedef core::ConfigurationOut_t ConfigurationOut_t;
    typedef core::Configuration_t Configuration_t;
    typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef core::DevicePtr_t DevicePtr_t;
  } // namespace walkgen
} // namespace hpp

#endif // HPP_WALKGEN_FWD_HH
