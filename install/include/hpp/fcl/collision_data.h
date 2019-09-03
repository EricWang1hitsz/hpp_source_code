/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */


#ifndef HPP_FCL_COLLISION_DATA_H
#define HPP_FCL_COLLISION_DATA_H

#include <hpp/fcl/collision_object.h>

#include <hpp/fcl/math/vec_3f.h>
#include <vector>
#include <set>
#include <limits>


namespace hpp
{
namespace fcl
{

/// @brief Type of narrow phase GJK solver
enum GJKSolverType {GST_INDEP};

/// @brief Contact information returned by collision
struct Contact
{
  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief contact primitive in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;


  /// @brief contact primitive in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;
 
  /// @brief contact normal, pointing from o1 to o2
  Vec3f normal;

  /// @brief contact position, in world space
  Vec3f pos;

  /// @brief penetration depth
  FCL_REAL penetration_depth;

 
  /// @brief invalid contact primitive information
  static const int NONE = -1;

  Contact() : o1(NULL),
              o2(NULL),
              b1(NONE),
              b2(NONE)
  {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_) : o1(o1_),
                                                                                          o2(o2_),
                                                                                          b1(b1_),
                                                                                          b2(b2_)
  {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_,
          const Vec3f& pos_, const Vec3f& normal_, FCL_REAL depth_) : o1(o1_),
                                                                      o2(o2_),
                                                                      b1(b1_),
                                                                      b2(b2_),
                                                                      normal(normal_),
                                                                      pos(pos_),
                                                                      penetration_depth(depth_)
  {}

  bool operator < (const Contact& other) const
  {
    if(b1 == other.b1)
      return b2 < other.b2;
    return b1 < other.b1;
  }

  bool operator == (const Contact& other) const
  {
    return o1 == other.o1
            && o2 == other.o2
            && b1 == other.b1
            && b2 == other.b2
            && normal == other.normal
            && pos == other.pos
            && penetration_depth == other.penetration_depth;
  }
};

struct CollisionResult;

/// @brief flag declaration for specifying required params in CollisionResult
enum CollisionRequestFlag
{
  CONTACT               = 0x00001,
  DISTANCE_LOWER_BOUND  = 0x00002,
  NO_REQUEST            = 0x01000
};

/// @brief request to the collision algorithm
struct CollisionRequest
{  
  /// @brief The maximum number of contacts will return
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and contact position) will return
  bool enable_contact;

  /// Whether a lower bound on distance is returned when objects are disjoint
  bool enable_distance_lower_bound;

  /// @brief narrow phase solver
  GJKSolverType gjk_solver_type;

  /// @brief whether enable gjk intial guess
  bool enable_cached_gjk_guess;
  
  /// @brief the gjk intial guess set by user
  Vec3f cached_gjk_guess;

  /// @brief Distance below which objects are considered in collision
  FCL_REAL security_margin;

  /// @brief Distance below which bounding volumes are break down
  FCL_REAL break_distance;

  explicit CollisionRequest(size_t num_max_contacts_,
                   bool enable_contact_ = false,
		   bool enable_distance_lower_bound_ = false,
                   size_t num_max_cost_sources_ = 1,
                   bool enable_cost_ = false,
                   bool use_approximate_cost_ = true,
                   GJKSolverType gjk_solver_type_ = GST_INDEP)
  HPP_FCL_DEPRECATED;

  explicit CollisionRequest(const CollisionRequestFlag flag, size_t num_max_contacts_) :
    num_max_contacts(num_max_contacts_),
    enable_contact(flag & CONTACT),
    enable_distance_lower_bound (flag & DISTANCE_LOWER_BOUND),
    gjk_solver_type(GST_INDEP),
    security_margin (0),
    break_distance (1e-3)
  {
    enable_cached_gjk_guess = false;
    cached_gjk_guess = Vec3f(1, 0, 0);
  }

  CollisionRequest() :
      num_max_contacts(1),
      enable_contact(false),
      enable_distance_lower_bound (false),
      gjk_solver_type(GST_INDEP),
      security_margin (0),
      break_distance (1e-3)
    {
      enable_cached_gjk_guess = false;
      cached_gjk_guess = Vec3f(1, 0, 0);
    }

  bool isSatisfied(const CollisionResult& result) const;
};

/// @brief collision result
struct CollisionResult
{
private:
  /// @brief contact information
  std::vector<Contact> contacts;

public:
  Vec3f cached_gjk_guess;

  /// Lower bound on distance between objects if they are disjoint
  /// \note computed only on request.
  FCL_REAL distance_lower_bound;

public:
  CollisionResult()
  {
  }


  /// @brief add one contact into result structure
  inline void addContact(const Contact& c) 
  {
    contacts.push_back(c);
  }

  /// @brief whether two CollisionResult are the same or not
  inline bool operator ==(const CollisionResult& other) const
  {
    return contacts == other.contacts 
            && distance_lower_bound == other.distance_lower_bound;
  }

  /// @brief return binary collision result
  bool isCollision() const
  {
    return contacts.size() > 0;
  }

  /// @brief number of contacts found
  size_t numContacts() const
  {
    return contacts.size();
  }

  /// @brief get the i-th contact calculated
  const Contact& getContact(size_t i) const
  {
    if(i < contacts.size()) 
      return contacts[i];
    else
      return contacts.back();
  }

  /// @brief get all the contacts
  void getContacts(std::vector<Contact>& contacts_)
  {
    contacts_.resize(contacts.size());
    std::copy(contacts.begin(), contacts.end(), contacts_.begin());
  }

  /// @brief clear the results obtained
  void clear()
  {
    contacts.clear();
  }

  /// @brief reposition Contact objects when fcl inverts them
  /// during their construction.
  friend void invertResults(CollisionResult& result);
};


struct DistanceResult;

/// @brief request to the distance computation
struct DistanceRequest
{
  /// @brief whether to return the nearest points
  bool enable_nearest_points;

  /// @brief error threshold for approximate distance
  FCL_REAL rel_err; // relative error, between 0 and 1
  FCL_REAL abs_err; // absoluate error

  /// @brief narrow phase solver type
  GJKSolverType gjk_solver_type;



  DistanceRequest(bool enable_nearest_points_ = false,
                  FCL_REAL rel_err_ = 0.0,
                  FCL_REAL abs_err_ = 0.0,
                  GJKSolverType gjk_solver_type_ = GST_INDEP) : enable_nearest_points(enable_nearest_points_),
                                                                rel_err(rel_err_),
                                                                abs_err(abs_err_),
                                                                gjk_solver_type(gjk_solver_type_)
  {
  }

  bool isSatisfied(const DistanceResult& result) const;
};


/// @brief distance result
struct DistanceResult
{

public:

  /// @brief minimum distance between two objects. if two objects are in collision, min_distance <= 0.
  FCL_REAL min_distance;

  /// @brief nearest points
  Vec3f nearest_points[2];

  /// In case both objects are in collision, store the normal
  Vec3f normal;

  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;
  
  DistanceResult(FCL_REAL min_distance_ =
                 std::numeric_limits<FCL_REAL>::max()):
  min_distance(min_distance_), o1(NULL), o2(NULL), b1(NONE), b2(NONE)
  {
    Vec3f nan (Vec3f::Constant(std::numeric_limits<FCL_REAL>::quiet_NaN()));
    nearest_points [0] = nearest_points [1] = normal = nan;
  }


  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_)
  {
    if(min_distance > distance)
    {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
    }
  }

  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_,
              const CollisionGeometry* o2_, int b1_, int b2_,
              const Vec3f& p1, const Vec3f& p2, const Vec3f& normal_)
  {
    if(min_distance > distance)
    {
      min_distance = distance;
      o1 = o1_;
      o2 = o2_;
      b1 = b1_;
      b2 = b2_;
      nearest_points[0] = p1;
      nearest_points[1] = p2;
      normal = normal_;
    }
  }

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result)
  {
    if(min_distance > other_result.min_distance)
    {
      min_distance = other_result.min_distance;
      o1 = other_result.o1;
      o2 = other_result.o2;
      b1 = other_result.b1;
      b2 = other_result.b2;
      nearest_points[0] = other_result.nearest_points[0];
      nearest_points[1] = other_result.nearest_points[1];
      normal = other_result.normal;
    }
  }

  /// @brief clear the result
  void clear()
  {
    min_distance = std::numeric_limits<FCL_REAL>::max();
    o1 = NULL;
    o2 = NULL;
    b1 = NONE;
    b2 = NONE;
  }

  /// @brief whether two DistanceResult are the same or not
  inline bool operator ==(const DistanceResult& other) const
  {
    bool is_same = min_distance == other.min_distance
                  && nearest_points[0] == other.nearest_points[0]
                  && nearest_points[1] == other.nearest_points[1]
                  && o1 == other.o1
                  && o2 == other.o2
                  && b1 == other.b1
                  && b2 == other.b2;

// TODO: check also that two GeometryObject are indeed equal.
    if ((o1 != NULL) xor (other.o1 != NULL)) return false;
    is_same &= (o1 == other.o1);
//    else if (o1 != NULL and other.o1 != NULL) is_same &= *o1 == *other.o1;

    if ((o2 != NULL) xor (other.o2 != NULL)) return false;
    is_same &= (o2 == other.o2);
//    else if (o2 != NULL and other.o2 != NULL) is_same &= *o2 == *other.o2;
  
    return is_same;
  }

};


inline CollisionRequestFlag operator~(CollisionRequestFlag a)
{return static_cast<CollisionRequestFlag>(~static_cast<const int>(a));}

inline CollisionRequestFlag operator|(CollisionRequestFlag a, CollisionRequestFlag b)
{return static_cast<CollisionRequestFlag>(static_cast<const int>(a) | static_cast<const int>(b));}

inline CollisionRequestFlag operator&(CollisionRequestFlag a, CollisionRequestFlag b)
{return static_cast<CollisionRequestFlag>(static_cast<const int>(a) & static_cast<const int>(b));}

inline CollisionRequestFlag operator^(CollisionRequestFlag a, CollisionRequestFlag b)
{return static_cast<CollisionRequestFlag>(static_cast<const int>(a) ^ static_cast<const int>(b));}

inline CollisionRequestFlag& operator|=(CollisionRequestFlag& a, CollisionRequestFlag b)
{return (CollisionRequestFlag&)((int&)(a) |= static_cast<const int>(b));}

inline CollisionRequestFlag& operator&=(CollisionRequestFlag& a, CollisionRequestFlag b)
{return (CollisionRequestFlag&)((int&)(a) &= static_cast<const int>(b));}

inline CollisionRequestFlag& operator^=(CollisionRequestFlag& a, CollisionRequestFlag b)
{return (CollisionRequestFlag&)((int&)(a) ^= static_cast<const int>(b));}

}

} // namespace hpp

#endif
