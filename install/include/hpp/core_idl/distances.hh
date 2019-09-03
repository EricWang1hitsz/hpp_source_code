#ifndef hpp_core_idl__distances_hxx__
#define hpp_core_idl__distances_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/distances.idl
//

#include <hpp/core_idl/distances-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::Distance
//
namespace core_impl {
template <typename _Base, typename _Storage>
DistanceServant<_Base, _Storage>::DistanceServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::Distance, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
DistanceServant<_Base, _Storage>::~DistanceServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::value_type DistanceServant<_Base, _Storage>::call (const hpp::floatSeq& q1, const hpp::floatSeq& q2)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _q1 = hpp::corbaServer::floatSeqToVector (q1);
      hpp::core::vector_t _q2 = hpp::corbaServer::floatSeqToVector (q2);
    return (getT()->operator() (_q1, _q2));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

//
// Implementational code for IDL interface hpp::core_idl::WeighedDistance
//
namespace core_impl {
template <typename _Base, typename _Storage>
WeighedDistanceServant<_Base, _Storage>::WeighedDistanceServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::DistanceServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
WeighedDistanceServant<_Base, _Storage>::~WeighedDistanceServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::floatSeq* WeighedDistanceServant<_Base, _Storage>::getWeights ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->weights ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void WeighedDistanceServant<_Base, _Storage>::setWeights (const hpp::floatSeq& weights)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _weights = hpp::corbaServer::floatSeqToVector (weights);
     (getT()->weights (_weights));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__distances_hxx__

