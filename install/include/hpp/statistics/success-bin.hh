// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-statistics.
// hpp-statistics is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-statistics is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-statistics. If not, see <http://www.gnu.org/licenses/>.


#ifndef HPP_STATISTICS_SUCCESSBIN_HH
# define HPP_STATISTICS_SUCCESSBIN_HH

# include <iostream>
# include <set>

# include <hpp/util/debug.hh>

# include "hpp/statistics/config.hh"
# include "hpp/statistics/bin.hh"
# include "hpp/statistics/fwd.hh"

# define HPP_DEFINE_REASON_FAILURE(ID, STRING) \
  const ::hpp::statistics::SuccessBin::Reason ID = \
    ::hpp::statistics::SuccessBin::createReason ( STRING ); \
  struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

namespace hpp {
  namespace statistics {
    /// This class count the number of success and failure.
    class HPP_STATISTICS_DLLAPI SuccessBin : public Bin
    {
      public:
        /// In case of failure, you can provide a reason.
        /// Use macro DEFINE_REASON_FAILURE (REASON_NAME, Reason string)
        ///       to define a new reason.
        struct Reason {
          std::size_t id;
          std::string what;
          Reason (std::size_t a_id, std::string a_what) :
            id (a_id), what (a_what) {}
        };

        /// The default reason for 'failure'.
        const static Reason REASON_UNKNOWN;

        /// Constructor
        SuccessBin (const bool success, const Reason& r = REASON_UNKNOWN) :
          Bin(), success_ (success), reason_(r)
        {
          if (success_)
            reason_ = REASON_SUCCESS;
        }

        /// Value of the bin.
        /// \return True is it counts "success", False otherwise.
        inline bool isSuccess () const
        {
          return success_;
        }

        /// If this bin represents 'failure', returns the reason.
        inline const Reason& reason () const
        {
          return reason_;
        }

        /// If this bin represents 'failure', returns the reason as a string.
        inline const std::string& reasonString () const;

        /// Check for equality.
        /// \return True if both are 'success' or if they are both 'failure'
        /// with the same Reason.
        inline bool operator == (const SuccessBin& other) const
        {
          return reason_.id == other.reason().id;
        }

        /// Comparison
        /// \return the comparison of their reason id.
        /// 'success' has a reason id of INT_MIN.
        inline bool operator < (const SuccessBin& other) const
        {
          return reason_.id < other.reason ().id;
        }

        /// Create a new Reason
        /// \param what The text associated with the reason.
        static Reason createReason (const std::string& what)
        {
          return Reason (reasonID_last++, what);
        }

      private:
        bool success_;
        size_t freq_;
        Reason reason_;

        /// The reason for 'success'.
        const static Reason REASON_SUCCESS;
        static std::size_t reasonID_last;

        inline std::ostream& printValue (std::ostream& os) const
        {
          os << "Event ";
          if (success_) os << "'Success'";
          else          os << "'Failure': " << reason_.what;
          return os;
        }
    };

    class HPP_STATISTICS_DLLAPI SuccessStatistics :
      public Statistics < SuccessBin >
    {
      public:
        typedef Statistics <SuccessBin> Parent;

        /// Constructor
        SuccessStatistics (const std::string name = "",
            const std::size_t& logRatio = 2)
          : name_ (name), logRatio_ (logRatio)
        {}

        /// Copy Constructor
        SuccessStatistics (const SuccessStatistics& other)
          : name_ (other.name_), logRatio_ (other.logRatio_)
        {}

        /// Add a 'success'
        void addSuccess ()
        {
          insert (SuccessBin (true));
        }

        /// Add a 'failure'
        /// \param r the reason of the 'failure'
        /// \note Use macro DEFINE_REASON_FAILURE (REASON_NAME, 'Reason details')
        ///       to define a new reason.
        void addFailure (const SuccessBin::Reason& r = SuccessBin::REASON_UNKNOWN)
        {
          insert (SuccessBin (false, r));
#ifdef HPP_DEBUG
          isLowRatio (true);
#endif
        }

        inline bool isLowRatio (const bool autoPrint = false) const
        {
          bool lowRatio = (logRatio_ * nbSuccess () < numberOfObservations());
          if (autoPrint && lowRatio)
            hppDout (info, name_ << ":\n" << *this);
          return lowRatio;
        }

        /// Count the number of success.
        std::size_t nbSuccess () const
        {
          return freq (SuccessBin(true));
        }

        /// Count the number of failure, in total.
        std::size_t nbFailure () const
        {
          return numberOfObservations() - nbSuccess();
        }

        /// Count the number of a particular failure.
        std::size_t nbFailure (const SuccessBin::Reason& r) const
        {
          return freq (SuccessBin (false, r));
        }

        std::string name_;

        /// If nbSuccess() * logRatio < numberOfObservations(), write to log.
        std::size_t logRatio_;
    };
  } // namespace statistics
} // namespace hpp

#endif // HPP_STATISTICS_SUCCESSBIN_HH
