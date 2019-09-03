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

#ifndef HPP_STATISTICS_BIN_HH
# define HPP_STATISTICS_BIN_HH

# include <ostream>
# include <list>
# include <algorithm>

# include "hpp/statistics/config.hh"
# include "hpp/statistics/fwd.hh"

namespace hpp {
  namespace statistics {
    /// Abstract class representing a bin.
    ///
    /// Bins are use for statistics. They keep the number of
    /// of apparition of a given value.
    /// Inherited class should also implement comparison and equality
    /// operators.
    class HPP_STATISTICS_DLLAPI Bin
    {
      public:
        /// Return the number of element in the bin.
        const std::size_t& freq () const
        {
          return freq_;
        }

        /// Add an occurence
        /// \return The frequency after increment;
        std::size_t operator ++()
        {
          return ++freq_;
        }

        /// Add an occurence
        /// \return The frequency before increment;
        std::size_t operator ++(int)
        {
          return freq_++;
        }

        /// Print the bin.
        virtual std::ostream& print (std::ostream& os) const
        {
          return printValue (os << freq () << " - ");
        }

        /// Print the inner value of the bin.
        virtual std::ostream& printValue (std::ostream& os) const = 0;

      protected:
        /// Constructor
        Bin () : freq_ (0) {}

      private:
        /// The number of occurence.
        std::size_t freq_;
    };

    inline std::ostream& operator<< (std::ostream& os, const hpp::statistics::Bin& b)
    {
      return b.print (os);
    }

    /// Template class to do statistics.
    /// You should derivate class Bin and construct a class
    /// Statistics < YourBin >.
    template < typename T >
    class HPP_STATISTICS_DLLAPI Statistics
    {
      public:
        typedef typename std::list < T > Container;
        typedef typename Container::iterator iterator;
        typedef typename Container::const_iterator const_iterator;

        /// Return the number of occurence of a Bin.
        /// \param bin a Bin for which only the value is useful.
        /// \note  It searches for the equivalent Bin is the set and
        ///        returns the frequency of the result.
        virtual std::size_t freq (const T& bin) const;

        /// Return the relative frequency of a Bin.
        /// \param bin a Bin for which only the value is useful.
        /// \note  It searches for the equivalent Bin is the set and
        ///        returns the frequency of the result.
        virtual Proba_t relativeFreq (const T& bin) const;

        /// Return the number of times an observation has recorded. It is the
        /// total number of observations.
        std::size_t numberOfObservations () const
        {
          return counts_;
        }

        /// Return the number of bins.
        unsigned int numberOfBins () const
        {
          return bins_.size ();
        }

        /// Put the results in a stream.
        virtual std::ostream& print (std::ostream& os) const;

        const_iterator find (const T& bin) const;

        template < typename U > const_iterator find (const U& value) const;

        /// Return an iterator pointing at the beginning of
        /// the set of bins.
        const_iterator begin() const
        {
          return bins_.begin();
        }

        /// Return an iterator pointing at the end of
        /// the set of bins.
        const_iterator end() const
        {
          return bins_.end();
        }

        /// Remove all element
        void clear ()
        {
          bins_.clear();
        }

      protected:
        /// Constructor
        Statistics();

        /// Increment a Bin
        /// \note bin is inserted in the set of bins if it was not
        /// already in the set.
        virtual T& increment (const T& bin) __attribute__ ((deprecated));

        /// insert a Bin.
        /// \note bin is inserted in the set of bins if it was not
        /// already in the set.
        virtual iterator insert (const T& bin);

      private:
        Container bins_;

        std::size_t counts_;
    };

    template < typename T >
      std::ostream& operator<< (std::ostream& os, const hpp::statistics::Statistics <T>& ss);
  } // namespace statistics
} // namespace hpp

/// Implementation

namespace hpp {
  namespace statistics {
    template < typename T >
      T& Statistics <T>::increment (const T& b)
    {
      counts_++;
      iterator it = bins_.begin ();
      for (; it != bins_.end (); it++) {
        if (! (*it < b)) {
          if (! (*it == b))
            it = bins_.insert (it, b);
          (*it)++;
          return *it;
        }
      }
      it = bins_.insert (it, b);
      (*it)++;
      return *it;
    }

    template < typename T >
      typename Statistics<T>::iterator Statistics <T>::insert (const T& b)
    {
      counts_++;
      iterator it = bins_.begin ();
      for (; it != bins_.end (); it++) {
        if (! (*it < b)) {
          if (! (*it == b))
            it = bins_.insert (it, b);
          (*it)++;
          return it;
        }
      }
      it = bins_.insert (it, b);
      (*it)++;
      return it;
    }

    template < typename T>
      typename Statistics<T>::const_iterator Statistics <T>::find (const T& b) const
    {
      for (const_iterator it = bins_.begin ();
          it != bins_.end (); it++) {
        if (*it < b)
          continue;
        if (*it == b)
          return it;
        break;
      }
      return bins_.end ();
    }

    template < typename T> template < typename U >
      typename Statistics<T>::const_iterator Statistics <T>::find (const U& v) const
    {
      return find (T (v));
    }

    template < typename T >
      size_t Statistics <T>::freq (const T& b) const
    {
      const_iterator it = std::find (bins_.begin (), bins_.end (), b);
      if (it == bins_.end ()) {
        return 0;
      }
      return it->freq ();
    }

    template < typename T >
      Proba_t Statistics <T>::relativeFreq (const T& b) const
    {
      const_iterator it = std::find (bins_.begin (), bins_.end (), b);
      if (it == bins_.end ()) {
        return 0;
      }
      return (Proba_t)it->freq () / (Proba_t)numberOfObservations();
    }

    template < typename T >
      Statistics <T>::Statistics () :bins_ (), counts_(0)
    {}

    template < typename T >
      std::ostream& Statistics<T>::print (std::ostream& os) const
    {
      const_iterator it;
      for (it = begin(); it != end(); it++) {
        it->print (os) << std::endl;
      }
      os << "Total number of observations: " << numberOfObservations ();
      return os;
    }

    template < typename T >
      std::ostream& operator<< (std::ostream& os, const hpp::statistics::Statistics <T>& ss)
      {
        return ss.print (os);
      }
  } // namespace statistics
} // namespace hpp

#endif // HPP_STATISTICS_BIN_HH
