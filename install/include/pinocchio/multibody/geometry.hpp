//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_multibody_geometry_hpp__
#define __pinocchio_multibody_geometry_hpp__

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/container/aligned-vector.hpp"

#include <iostream>

#include <boost/foreach.hpp>
#include <map>
#include <list>
#include <utility>
#include <assert.h>

namespace pinocchio
{
  
  struct GeometryModel
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef double Scalar;
    enum { Options = 0 };
    
    typedef SE3Tpl<Scalar,Options> SE3;
    
    typedef container::aligned_vector<GeometryObject> GeometryObjectVector;
    typedef std::vector<CollisionPair> CollisionPairVector;
    
    typedef pinocchio::GeomIndex GeomIndex;
    
    /// \brief The number of GeometryObjects
    Index ngeoms;

    /// \brief Vector of GeometryObjects used for collision computations
    GeometryObjectVector geometryObjects;
    ///
    /// \brief Vector of collision pairs.
    ///
    CollisionPairVector collisionPairs;
  
    GeometryModel()
    : ngeoms(0)
    , geometryObjects()
    , collisionPairs()
    { 
      const std::size_t num_max_collision_pairs = (ngeoms * (ngeoms-1))/2;
      collisionPairs.reserve(num_max_collision_pairs);
    }

    ~GeometryModel() {};

    /**
     * @brief      Add a geometry object to a GeometryModel
     * @deprecated This method has been set to deprecated. Please use other signature of addGeometryObject.
     *
     * @param[in]  object     Object 
     * @param[in]  model      Corresponding model, used to assert the attributes of object.
     * @param[in]  autofillJointParent if true, set jointParent from frameParent.
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     * @note object is a nonconst copy to ease the insertion code.
     */
    template<typename S2, int O2, template<typename,int> class _JointCollectionTpl>
    PINOCCHIO_DEPRECATED
    GeomIndex addGeometryObject(GeometryObject object,
                                const ModelTpl<S2,O2,_JointCollectionTpl> & model,
                                const bool autofillJointParent)
    {
      if(autofillJointParent)
        return addGeometryObject(object,model);
      else
        return addGeometryObject(object);
    }
    
    /**
     * @brief      Add a geometry object to a GeometryModel and set its parent joint.
     *
     * @param[in]  object     Object
     * @param[in]  model      Corresponding model, used to assert the attributes of object.
     *
     * @return     The index of the new added GeometryObject in geometryObjects
     * @note object is a nonconst copy to ease the insertion code.
     */
    template<typename S2, int O2, template<typename,int> class _JointCollectionTpl>
    GeomIndex addGeometryObject(const GeometryObject & object,
                                const ModelTpl<S2,O2,_JointCollectionTpl> & model);
    
    /**
     * @brief      Add a geometry object to a GeometryModel.
     *
     * @param[in]  object     Object
     *
     * @return     The index of the new added GeometryObject in geometryObjects.
     */
    GeomIndex addGeometryObject(const GeometryObject & object);

    /**
     * @brief      Return the index of a GeometryObject given by its name.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     Index of the corresponding GeometryObject
     */
    GeomIndex getGeometryId(const std::string & name) const;

    
    /**
     * @brief      Check if a GeometryObject  given by its name exists.
     *
     * @param[in]  name  Name of the GeometryObject
     *
     * @return     True if the GeometryObject exists in the geometryObjects.
     */
    bool existGeometryName(const std::string & name) const;


    /**
     * @brief      Get the name of a GeometryObject given by its index.
     *
     * @param[in]  index  Index of the GeometryObject
     *
     * @return     Name of the GeometryObject
     */
    PINOCCHIO_DEPRECATED const std::string & getGeometryName(const GeomIndex index) const;

#ifdef PINOCCHIO_WITH_HPP_FCL
    ///
    /// \brief Add a collision pair into the vector of collision_pairs.
    ///        The method check before if the given CollisionPair is already included.
    ///
    /// \param[in] pair The CollisionPair to add.
    ///
    void addCollisionPair(const CollisionPair & pair);
    
    ///
    /// \brief Add all possible collision pairs.
    ///
    /// \note Collision pairs between geometries of having the same parent joint
    ///       are not added.
    ///
    void addAllCollisionPairs();
   
    ///
    /// \brief Remove if exists the CollisionPair from the vector collision_pairs.
    ///
    /// \param[in] pair The CollisionPair to remove.
    ///
    void removeCollisionPair(const CollisionPair& pair);
    
    ///
    /// \brief Remove all collision pairs from collisionPairs. Same as collisionPairs.clear().
    void removeAllCollisionPairs ();
   
    ///
    /// \brief Check if a collision pair exists in collisionPairs.
    ///        See also findCollisitionPair(const CollisionPair & pair).
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return True if the CollisionPair exists, false otherwise.
    ///
    bool existCollisionPair(const CollisionPair & pair) const;
    
    ///
    /// \brief Return the index of a given collision pair in collisionPairs.
    ///
    /// \param[in] pair The CollisionPair.
    ///
    /// \return The index of the CollisionPair in collisionPairs.
    ///
    PairIndex findCollisionPair(const CollisionPair & pair) const;
    
#endif // PINOCCHIO_WITH_HPP_FCL

    friend std::ostream& operator<<(std::ostream & os, const GeometryModel & model_geom);
  }; // struct GeometryModel

  struct GeometryData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef double Scalar;
    enum { Options = 0 };
    
    typedef SE3Tpl<Scalar,Options> SE3;
    
    ///
    /// \brief Vector gathering the SE3 placements of the geometry objects relative to the world.
    ///        See updateGeometryPlacements to update the placements.
    ///
    /// oMg is used for pinocchio (kinematics) computation but is translated to fcl type
    /// for fcl (collision) computation. The copy is done in collisionObjects[i]->setTransform(.)
    ///
    container::aligned_vector<SE3> oMg;

#ifdef PINOCCHIO_WITH_HPP_FCL
    ///
    /// \brief Collision objects (ie a fcl placed geometry).
    ///
    /// The object contains a pointer on the collision geometries contained in geomModel.geometryObjects.
    /// \sa GeometryModel::geometryObjects and GeometryObjects
    ///
    std::vector<fcl::CollisionObject> collisionObjects;

    ///
    /// \brief Vector of collision pairs.
    ///
    std::vector<bool> activeCollisionPairs;

    ///
    /// \brief Defines what information should be computed by distance computation.
    ///
    fcl::DistanceRequest distanceRequest;

    ///
    /// \brief Vector gathering the result of the distance computation for all the collision pairs.
    ///
    std::vector<fcl::DistanceResult> distanceResults;
    
    ///
    /// \brief Defines what information should be computed by collision test.
    ///
    fcl::CollisionRequest collisionRequest;

    ///
    /// \brief Vector gathering the result of the collision computation for all the collision pairs.
    ///
    std::vector<fcl::CollisionResult> collisionResults;

    ///
    /// \brief Radius of the bodies, i.e. distance of the further point of the geometry model
    /// attached to the body from the joint center.
    ///
    std::vector<double> radius;

    ///
    /// \brief index of the collision pair
    ///
    /// It is used by some method to return additional information. For instance,
    /// the algo computeCollisions() sets it to the first colliding pair.
    ///
    PairIndex collisionPairIndex;

    typedef std::vector<GeomIndex> GeomIndexList;

    /// \brief Map over vector GeomModel::geometryObjects, indexed by joints.
    /// 
    /// The map lists the collision GeometryObjects associated to a given joint Id.
    ///  Inner objects can be seen as geometry objects that directly move when the associated joint moves
    std::map < JointIndex, GeomIndexList >  innerObjects;

    /// \brief A list of associated collision GeometryObjects to a given joint Id
    ///
    /// Outer objects can be seen as geometry objects that may often be
    /// obstacles to the Inner objects of given joint
    std::map < JointIndex, GeomIndexList >  outerObjects;
#endif // PINOCCHIO_WITH_HPP_FCL   

    GeometryData(const GeometryModel & geomModel);
    ~GeometryData() {};

#ifdef PINOCCHIO_WITH_HPP_FCL

    /// Fill both innerObjects and outerObjects maps, from vectors collisionObjects and 
    /// collisionPairs. 
    ///
    /// This simply corresponds to storing in a re-arranged manner the information stored
    /// in geomModel.geometryObjects and geomModel.collisionPairs.
    /// \param[in] geomModel the geometry model (const)
    ///
    /// \warning Outer objects are not duplicated (i.e. if a is in outerObjects[b], then
    /// b is not in outerObjects[a]).
    void fillInnerOuterObjectMaps(const GeometryModel & geomModel);

    /// Activate a collision pair, for which collisions and distances would now be computed.
    ///
    /// A collision (resp distance) between to geometries of GeomModel::geometryObjects
    /// is computed *iff* the corresponding pair has been added in GeomModel::collisionPairs *AND*
    /// it is active, i.e. the corresponding boolean in GeomData::activePairs is true. The second
    /// condition can be used to temporarily remove a pair without touching the model, in a versatile
    /// manner. 
    /// \param[in] pairId the index of the pair in GeomModel::collisionPairs vector.
    /// \param[in] flag value of the activation boolean (true by default).
    PINOCCHIO_DEPRECATED
    void activateCollisionPair(const PairIndex pairId, const bool flag);
    
    ///
    /// Activate a collision pair, for which collisions and distances would now be computed.
    ///
    /// A collision (resp distance) between to geometries of GeomModel::geometryObjects
    /// is computed *iff* the corresponding pair has been added in GeomModel::collisionPairs *AND*
    /// it is active, i.e. the corresponding boolean in GeomData::activePairs is true. The second
    /// condition can be used to temporarily remove a pair without touching the model, in a versatile
    /// manner.
    ///
    /// \param[in] pairId the index of the pair in GeomModel::collisionPairs vector.
    ///
    /// \sa GeomData
    ///
    void activateCollisionPair(const PairIndex pairId);

    ///
    /// Deactivate a collision pair.
    ///
    /// Calls indeed GeomData::activateCollisionPair(pairId)
    ///
    /// \param[in] pairId the index of the pair in GeomModel::collisionPairs vector.
    ///
    /// \sa GeomData::activateCollisionPair
    ///
    void deactivateCollisionPair(const PairIndex pairId);

#endif //PINOCCHIO_WITH_HPP_FCL
    friend std::ostream & operator<<(std::ostream & os, const GeometryData & geomData);
    
  }; // struct GeometryData

} // namespace pinocchio

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/geometry.hxx"

#endif // ifndef __pinocchio_multibody_geometry_hpp__
