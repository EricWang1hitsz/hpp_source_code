#ifndef hpp_manipulation_idl__steering__methods_hxx__
#define hpp_manipulation_idl__steering__methods_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/steering_methods.idl
//

#include <hpp/manipulation_idl/steering_methods-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::manipulation_idl::steeringMethod::EndEffectorTrajectory
//
namespace manipulation_impl {

namespace steeringMethod {
template <typename _Base, typename _Storage>
EndEffectorTrajectoryServant<_Base, _Storage>::EndEffectorTrajectoryServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::SteeringMethodServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
EndEffectorTrajectoryServant<_Base, _Storage>::~EndEffectorTrajectoryServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
void EndEffectorTrajectoryServant<_Base, _Storage>::trajectoryConstraint (hpp::constraints_idl::Implicit_ptr c)
{
  try {
    // automatically generated code.
    hpp::constraints::ImplicitPtr_t _c = reference_to_servant_base<hpp::constraints::Implicit>(server_, c)->get();
     (getT()->trajectoryConstraint (_c));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void EndEffectorTrajectoryServant<_Base, _Storage>::trajectory (hpp::core_idl::Path_ptr eeTraj, ::CORBA::Boolean se3Output)
{
  try {
    // automatically generated code.
    hpp::core::PathPtr_t _eeTraj = reference_to_servant_base<hpp::core::Path>(server_, eeTraj)->get();
     (getT()->trajectory (_eeTraj, se3Output));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace steeringMethod



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl__steering__methods_hxx__

