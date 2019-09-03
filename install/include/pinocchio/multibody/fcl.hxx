//
// Copyright (c) 2015-2016 CNRS
//

#ifndef __pinocchio_fcl_hxx__
#define __pinocchio_fcl_hxx__

#include <ostream>

namespace pinocchio
{

  ///
  /// \brief Default constructor of a collision pair from two collision object indexes.
  ///        The indexes must be ordered such that co1 < co2. If not, the constructor reverts the indexes.
  ///
  /// \param[in] co1 Index of the first collision object
  /// \param[in] co2 Index of the second collision object
  ///
  inline CollisionPair::CollisionPair(const GeomIndex co1, const GeomIndex co2) 
    : Base(co1,co2)
  {
    assert(co1 != co2 && "The index of collision objects must not be equal.");
  }

  inline bool CollisionPair::operator== (const CollisionPair& rhs) const
  {
    return (first == rhs.first  && second == rhs.second)
      ||   (first == rhs.second && second == rhs.first );
  }

  inline void CollisionPair::disp(std::ostream & os) const
  { os << "collision pair (" << first << "," << second << ")\n"; }

  inline std::ostream & operator << (std::ostream & os, const CollisionPair & X)
  {
    X.disp(os); return os;
  } 
  

#ifdef PINOCCHIO_WITH_HPP_FCL  

  inline bool operator == (const fcl::CollisionObject & lhs, const fcl::CollisionObject & rhs)
  {
    return lhs.collisionGeometry() == rhs.collisionGeometry()
            && lhs.getAABB().min_ == rhs.getAABB().min_
            && lhs.getAABB().max_ == rhs.getAABB().max_;
  }
  
#endif // PINOCCHIO_WITH_HPP_FCL
  
  inline bool operator==(const GeometryObject & lhs, const GeometryObject & rhs)
  {
    return ( lhs.name           == rhs.name
            && lhs.parentFrame  == rhs.parentFrame
            && lhs.parentJoint  == rhs.parentJoint
            && lhs.fcl          == rhs.fcl
            && lhs.placement    == rhs.placement
            && lhs.meshPath     == rhs.meshPath
            && lhs.meshScale    == rhs.meshScale
            );
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object)
  {
    os  << "Name: \t \n" << geom_object.name << "\n"
        << "Parent frame ID: \t \n" << geom_object.parentFrame << "\n"
        << "Parent joint ID: \t \n" << geom_object.parentJoint << "\n"
        << "Position in parent frame: \t \n" << geom_object.placement << "\n"
        << "Absolute path to mesh file: \t \n" << geom_object.meshPath << "\n"
        << "Scale for transformation of the mesh: \t \n" << geom_object.meshScale.transpose() << "\n"
        << std::endl;
    return os;
  }


} // namespace pinocchio


#endif // ifndef __pinocchio_fcl_hxx__
