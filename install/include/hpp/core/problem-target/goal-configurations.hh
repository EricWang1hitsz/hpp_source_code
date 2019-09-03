//
// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH
# define HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH

# include <hpp/core/problem-target.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/config.hh>

namespace hpp {
  namespace core {
    namespace problemTarget {
      /// \addtogroup path_planning
      /// \{

      /// Goal configurations
      ///
      /// This class defines a goal as a list of goal configurations.
      class HPP_CORE_DLLAPI GoalConfigurations : public ProblemTarget {
        public:
          static GoalConfigurationsPtr_t create
            (const ProblemPtr_t& problem);

          /// Check if the problem target is well specified.
          void check (const RoadmapPtr_t& roadmap) const;

          /// Check if the initial configuration and one of the goal are
          /// in the same connected component.
          bool reached (const RoadmapPtr_t& roadmap) const;

          PathVectorPtr_t computePath(const RoadmapPtr_t& roadmap) const;

        protected:
          /// Constructor
          GoalConfigurations (const ProblemPtr_t& problem)
            : ProblemTarget (problem)
          {}
      }; // class GoalConfigurations
      /// \}
    } // namespace problemTarget
  } //   namespace core
} // namespace hpp
#endif // HPP_CORE_PROBLEM_TARGET_GOAL_CONFIGURATIONS_HH
