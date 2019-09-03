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


#ifndef HPP_FCL_TRAVERSAL_RECURSE_H
#define HPP_FCL_TRAVERSAL_RECURSE_H

#include <hpp/fcl/traversal/traversal_node_base.h>
#include <hpp/fcl/traversal/traversal_node_bvhs.h>
#include <hpp/fcl/BVH/BVH_front.h>
#include <queue>

namespace hpp
{
namespace fcl
{

/// Recurse function for collision
/// \param node collision node,
/// \param b1, b2 ids of bounding volume nodes for object 1 and object 2
/// \retval sqrDistLowerBound squared lower bound on distance between objects.
void collisionRecurse(CollisionTraversalNodeBase* node, int b1, int b2,
		      BVHFrontList* front_list, FCL_REAL& sqrDistLowerBound);

/// @brief Recurse function for collision, specialized for OBB type
void collisionRecurse(MeshCollisionTraversalNodeOBB* node, int b1, int b2, const Matrix3f& R, const Vec3f& T, BVHFrontList* front_list);

/// @brief Recurse function for collision, specialized for RSS type
void collisionRecurse(MeshCollisionTraversalNodeRSS* node, int b1, int b2, const Matrix3f& R, const Vec3f& T, BVHFrontList* front_list);

/// @brief Recurse function for distance
void distanceRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list);

/// @brief Recurse function for distance, using queue acceleration
void distanceQueueRecurse(DistanceTraversalNodeBase* node, int b1, int b2, BVHFrontList* front_list, int qsize);

/// @brief Recurse function for front list propagation
void propagateBVHFrontListCollisionRecurse
  (CollisionTraversalNodeBase* node, const CollisionRequest& request,
   CollisionResult& result, BVHFrontList* front_list);


}

} // namespace hpp

#endif
