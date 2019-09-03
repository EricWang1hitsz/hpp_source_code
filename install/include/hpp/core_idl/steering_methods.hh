#ifndef hpp_core_idl__steering__methods_hxx__
#define hpp_core_idl__steering__methods_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/steering_methods.idl
//

#include <hpp/core_idl/steering_methods-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::SteeringMethod
//
namespace core_impl {
template <typename _Base, typename _Storage>
SteeringMethodServant<_Base, _Storage>::SteeringMethodServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::SteeringMethod, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
SteeringMethodServant<_Base, _Storage>::~SteeringMethodServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::core_idl::Path_ptr SteeringMethodServant<_Base, _Storage>::call (const hpp::floatSeq& q1, const hpp::floatSeq& q2)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _q1 = hpp::corbaServer::floatSeqToVector (q1);
      hpp::core::vector_t _q2 = hpp::corbaServer::floatSeqToVector (q2);
    hpp::core::PathPtr_t __return__ (getT()->operator() (_q1, _q2));
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::Path>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void SteeringMethodServant<_Base, _Storage>::setConstraints (hpp::core_idl::ConstraintSet_ptr constraints)
{
  try {
    // automatically generated code.
    hpp::core::ConstraintSetPtr_t _constraints = reference_to_servant_base<hpp::core::ConstraintSet>(server_, constraints)->get();
     (getT()->constraints (_constraints));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::Constraint_ptr SteeringMethodServant<_Base, _Storage>::getConstraints ()
{
  try {
    // automatically generated code.
    
    hpp::core::ConstraintPtr_t __return__ (getT()->constraints ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Constraint,hpp::corbaServer::core_impl::Constraint>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__steering__methods_hxx__

