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


#ifndef HPP_FCL_TRAVERSAL_NODE_MESHES_H
#define HPP_FCL_TRAVERSAL_NODE_MESHES_H

#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/traversal/traversal_node_base.h>
#include <hpp/fcl/BV/BV_node.h>
#include <hpp/fcl/BV/BV.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/intersect.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/narrowphase/narrowphase.h>
#include <hpp/fcl/traversal/details/traversal.h>

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <limits>
#include <vector>
#include <cassert>


namespace hpp
{
namespace fcl
{
/// @brief Traversal node for collision between BVH models
template<typename BV>
class BVHCollisionTraversalNode : public CollisionTraversalNodeBase
{
public:
  BVHCollisionTraversalNode(const CollisionRequest& request) :
  CollisionTraversalNodeBase (request)
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(int b1, int b2) const
  {
    FCL_REAL sz1 = model1->getBV(b1).bv.size();
    FCL_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }
  
  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};

/// @brief Traversal node for collision between two meshes
template<typename BV, int _Options = RelativeTransformationIsIdentity>
class MeshCollisionTraversalNode : public BVHCollisionTraversalNode<BV>
{
public:
  enum {
    Options = _Options,
    RTIsIdentity = _Options & RelativeTransformationIsIdentity
  };

  MeshCollisionTraversalNode(const CollisionRequest& request) :
  BVHCollisionTraversalNode<BV> (request)
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;
  }

  /// @brief BV culling test in one BVTT node
  bool BVTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    if (RTIsIdentity)
      return !this->model1->getBV(b1).overlap(this->model2->getBV(b2));
    else
      return !overlap(RT._R(), RT._T(),
          this->model1->getBV(b1).bv, this->model2->getBV(b2).bv);
  }
  
  /// BV test between b1 and b2
  /// \param b1, b2 Bounding volumes to test,
  /// \retval sqrDistLowerBound square of a lower bound of the minimal
  ///         distance between bounding volumes.
  bool BVTesting(int b1, int b2, FCL_REAL& sqrDistLowerBound) const
  {
    if(this->enable_statistics) this->num_bv_tests++;
    if (RTIsIdentity)
      return !this->model1->getBV(b1).overlap(this->model2->getBV(b2),
          this->request, sqrDistLowerBound);
    else {
      bool res = !overlap(RT._R(), RT._T(),
          this->model1->getBV(b1).bv, this->model2->getBV(b2).bv,
          this->request, sqrDistLowerBound);
      assert (!res || sqrDistLowerBound > 0);
      return res;
    }
  }

  /// Intersection testing between leaves (two triangles)
  ///
  /// \param b1, b2 id of primitive in bounding volume hierarchy
  /// \retval sqrDistLowerBound squared lower bound of distance between
  ///         primitives if they are not in collision.
  ///
  /// This method supports a security margin. If the distance between
  /// the primitives is less than the security margin, the objects are
  /// considered as in collision. in this case a contact point is
  /// returned in the CollisionResult.
  ///
  /// \note If the distance between objects is less than the security margin,
  ///       and the object are not colliding, the penetration depth is
  ///       negative.
  void leafTesting(int b1, int b2, FCL_REAL& sqrDistLowerBound) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    const Vec3f& P1 = vertices1[tri_id1[0]];
    const Vec3f& P2 = vertices1[tri_id1[1]];
    const Vec3f& P3 = vertices1[tri_id1[2]];
    const Vec3f& Q1 = vertices2[tri_id2[0]];
    const Vec3f& Q2 = vertices2[tri_id2[1]];
    const Vec3f& Q3 = vertices2[tri_id2[2]];

    TriangleP tri1 (P1, P2, P3);
    TriangleP tri2 (Q1, Q2, Q3);
    GJKSolver_indep solver;
    Vec3f p1, p2; // closest points if no collision contact points if collision.
    Vec3f normal;
    FCL_REAL distance;
    solver.shapeDistance (tri1, this->tf1, tri2, this->tf2,
                          distance, p1, p2, normal);
    FCL_REAL distToCollision = distance - this->request.security_margin;
    sqrDistLowerBound = distance * distance;
    if (distToCollision <= 0) { // collision
      Vec3f p (p1); // contact point
      FCL_REAL penetrationDepth (0);
      if(this->result->numContacts() < this->request.num_max_contacts) {
        // How much (Q1, Q2, Q3) should be moved so that all vertices are
        // above (P1, P2, P3).
        penetrationDepth = -distance;
        if (distance > 0) {
          normal = (p2-p1).normalized ();
          p = .5* (p1+p2);
        }
        this->result->addContact(Contact(this->model1, this->model2,
                                         primitive_id1, primitive_id2,
                                         p, normal, penetrationDepth));
      }
    }
  }

  /// @brief Whether the traversal process can stop early
  bool canStop() const
  {
    return this->request.isSatisfied(*(this->result));
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  details::RelativeTransformation<!bool(RTIsIdentity)> RT;
};

/// @brief Traversal node for collision between two meshes if their underlying BVH node is oriented node (OBB, RSS, OBBRSS, kIOS)
typedef MeshCollisionTraversalNode<OBB   , 0> MeshCollisionTraversalNodeOBB   ;
typedef MeshCollisionTraversalNode<RSS   , 0> MeshCollisionTraversalNodeRSS   ;
typedef MeshCollisionTraversalNode<kIOS  , 0> MeshCollisionTraversalNodekIOS  ;
typedef MeshCollisionTraversalNode<OBBRSS, 0> MeshCollisionTraversalNodeOBBRSS;

namespace details
{
  template<typename BV> struct DistanceTraversalBVTesting_impl
  {
    static FCL_REAL run(const BVNode<BV>& b1, const BVNode<BV>& b2)
    {
      return b1.distance(b2);
    }
  };

  template<> struct DistanceTraversalBVTesting_impl<OBB>
  {
    static FCL_REAL run(const BVNode<OBB>& b1, const BVNode<OBB>& b2)
    {
      FCL_REAL sqrDistLowerBound;
      CollisionRequest request (DISTANCE_LOWER_BOUND, 0);
      // request.break_distance = ?
      if (b1.overlap(b2, request, sqrDistLowerBound)) {
        // TODO A penetration upper bound should be computed.
        return -1;
      }
      return sqrt (sqrDistLowerBound);
    }
  };
} // namespace details

/// @brief Traversal node for distance computation between BVH models
template<typename BV>
class BVHDistanceTraversalNode : public DistanceTraversalNodeBase
{
public:
  BVHDistanceTraversalNode() : DistanceTraversalNodeBase()
  {
    model1 = NULL;
    model2 = NULL;

    num_bv_tests = 0;
    num_leaf_tests = 0;
    query_time_seconds = 0.0;
  }

  /// @brief Whether the BV node in the first BVH tree is leaf
  bool isFirstNodeLeaf(int b) const
  {
    return model1->getBV(b).isLeaf();
  }

  /// @brief Whether the BV node in the second BVH tree is leaf
  bool isSecondNodeLeaf(int b) const
  {
    return model2->getBV(b).isLeaf();
  }

  /// @brief Determine the traversal order, is the first BVTT subtree better
  bool firstOverSecond(int b1, int b2) const
  {
    FCL_REAL sz1 = model1->getBV(b1).bv.size();
    FCL_REAL sz2 = model2->getBV(b2).bv.size();

    bool l1 = model1->getBV(b1).isLeaf();
    bool l2 = model2->getBV(b2).isLeaf();

    if(l2 || (!l1 && (sz1 > sz2)))
      return true;
    return false;
  }

  /// @brief Obtain the left child of BV node in the first BVH
  int getFirstLeftChild(int b) const
  {
    return model1->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the first BVH
  int getFirstRightChild(int b) const
  {
    return model1->getBV(b).rightChild();
  }

  /// @brief Obtain the left child of BV node in the second BVH
  int getSecondLeftChild(int b) const
  {
    return model2->getBV(b).leftChild();
  }

  /// @brief Obtain the right child of BV node in the second BVH
  int getSecondRightChild(int b) const
  {
    return model2->getBV(b).rightChild();
  }

  /// @brief BV culling test in one BVTT node
  FCL_REAL BVTesting(int b1, int b2) const
  {
    if(enable_statistics) num_bv_tests++;
    return details::DistanceTraversalBVTesting_impl<BV>
      ::run (model1->getBV(b1), model2->getBV(b2));
  }

  /// @brief The first BVH model
  const BVHModel<BV>* model1;
  /// @brief The second BVH model
  const BVHModel<BV>* model2;

  /// @brief statistical information
  mutable int num_bv_tests;
  mutable int num_leaf_tests;
  mutable FCL_REAL query_time_seconds;
};


/// @brief Traversal node for distance computation between two meshes
template<typename BV>
class MeshDistanceTraversalNode : public BVHDistanceTraversalNode<BV>
{
public:
  MeshDistanceTraversalNode() : BVHDistanceTraversalNode<BV>()
  {
    vertices1 = NULL;
    vertices2 = NULL;
    tri_indices1 = NULL;
    tri_indices2 = NULL;

    rel_err = this->request.rel_err;
    abs_err = this->request.abs_err;
  }

  /// @brief Distance testing between leaves (two triangles)
  void leafTesting(int b1, int b2) const
  {
    if(this->enable_statistics) this->num_leaf_tests++;

    const BVNode<BV>& node1 = this->model1->getBV(b1);
    const BVNode<BV>& node2 = this->model2->getBV(b2);

    int primitive_id1 = node1.primitiveId();
    int primitive_id2 = node2.primitiveId();

    const Triangle& tri_id1 = tri_indices1[primitive_id1];
    const Triangle& tri_id2 = tri_indices2[primitive_id2];

    const Vec3f& t11 = vertices1[tri_id1[0]];
    const Vec3f& t12 = vertices1[tri_id1[1]];
    const Vec3f& t13 = vertices1[tri_id1[2]];

    const Vec3f& t21 = vertices2[tri_id2[0]];
    const Vec3f& t22 = vertices2[tri_id2[1]];
    const Vec3f& t23 = vertices2[tri_id2[2]];

    // nearest point pair
    Vec3f P1, P2, normal;

    FCL_REAL d = sqrt (TriangleDistance::sqrTriDistance
		       (t11, t12, t13, t21, t22, t23, P1, P2));

    this->result->update(d, this->model1, this->model2, primitive_id1,
                         primitive_id2, P1, P2, normal);
  }

  /// @brief Whether the traversal process can stop early
  bool canStop(FCL_REAL c) const
  {
    if((c >= this->result->min_distance - abs_err) && (c * (1 + rel_err) >= this->result->min_distance))
      return true;
    return false;
  }

  Vec3f* vertices1;
  Vec3f* vertices2;

  Triangle* tri_indices1;
  Triangle* tri_indices2;

  /// @brief relative and absolute error, default value is 0.01 for both terms
  FCL_REAL rel_err;
  FCL_REAL abs_err;
};

/// @brief Traversal node for distance computation between two meshes if their underlying BVH node is oriented node (RSS, OBBRSS, kIOS)
class MeshDistanceTraversalNodeRSS : public MeshDistanceTraversalNode<RSS>
{
public:
  MeshDistanceTraversalNodeRSS();

  void preprocess();

  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};


class MeshDistanceTraversalNodekIOS : public MeshDistanceTraversalNode<kIOS>
{
public:
  MeshDistanceTraversalNodekIOS();

  void preprocess();
  
  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};

class MeshDistanceTraversalNodeOBBRSS : public MeshDistanceTraversalNode<OBBRSS>
{
public:
  MeshDistanceTraversalNodeOBBRSS();

  void preprocess();

  void postprocess();

  FCL_REAL BVTesting(int b1, int b2) const;

  FCL_REAL BVTesting(int b1, int b2, FCL_REAL& sqrDistLowerBound) const;

  void leafTesting(int b1, int b2) const;

  Matrix3f R;
  Vec3f T;
};

/// @brief for OBB and RSS, there is local coordinate of BV, so normal need to be transformed
namespace details
{

template<typename BV>
inline const Matrix3f& getBVAxes(const BV& bv)
{
  return bv.axes;
}

template<>
inline const Matrix3f& getBVAxes<OBBRSS>(const OBBRSS& bv)
{
  return bv.obb.axes;
}


}

}

} // namespace hpp

#endif
