#ifndef hpp_core_idl__distances_hpp__
#define hpp_core_idl__distances_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/distances.idl
//

#include <hpp/core_idl/distances-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/distance.hh>
#include <hpp/core/weighed-distance.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::Distance
//
namespace core_impl {
template <typename _Base, typename _Storage>
class DistanceServant: public ServantBase<hpp::core::Distance, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::Distance HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::Distance, HppBase);

public:
  // standard constructor
  DistanceServant(Server* server, const _Storage& s);
  virtual ~DistanceServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::value_type call (const hpp::floatSeq& q1, const hpp::floatSeq& q2);


};

typedef DistanceServant<POA_hpp::core_idl::Distance,hpp::core::DistancePtr_t > Distance;
} // namespace core_impl

//
// Class implementing IDL interface hpp::core_idl::WeighedDistance
//
namespace core_impl {
template <typename _Base, typename _Storage>
class WeighedDistanceServant: public hpp::corbaServer::core_impl::DistanceServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::DistanceServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::WeighedDistance, HppBase);

public:
  // standard constructor
  WeighedDistanceServant(Server* server, const _Storage& s);
  virtual ~WeighedDistanceServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::floatSeq* getWeights ();


  void setWeights (const hpp::floatSeq& weights);


};

typedef WeighedDistanceServant<POA_hpp::core_idl::WeighedDistance,hpp::core::WeighedDistancePtr_t > WeighedDistance;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__distances_hpp__

