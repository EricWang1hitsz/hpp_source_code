#ifndef hpp_manipulation_idl__steering__methods_hpp__
#define hpp_manipulation_idl__steering__methods_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/steering_methods.idl
//

#include <hpp/manipulation_idl/steering_methods-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/constraints_idl/constraints.hh>
#include <hpp/core_idl/steering_methods.hh>
#include <hpp/manipulation/steering-method/end-effector-trajectory.hh>


namespace hpp {

namespace corbaServer {

//
// Class implementing IDL interface hpp::manipulation_idl::steeringMethod::EndEffectorTrajectory
//
namespace manipulation_impl {

namespace steeringMethod {
template <typename _Base, typename _Storage>
class EndEffectorTrajectoryServant: public hpp::corbaServer::core_impl::SteeringMethodServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::SteeringMethodServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::steeringMethod::EndEffectorTrajectory, HppBase);

public:
  // standard constructor
  EndEffectorTrajectoryServant(Server* server, const _Storage& s);
  virtual ~EndEffectorTrajectoryServant();

  // methods corresponding to defined IDL attributes and operations
  
  void trajectoryConstraint (hpp::constraints_idl::Implicit_ptr c);


  void trajectory (hpp::core_idl::Path_ptr eeTraj, ::CORBA::Boolean se3Output);


};

typedef EndEffectorTrajectoryServant<POA_hpp::manipulation_idl::steeringMethod::EndEffectorTrajectory,hpp::manipulation::steeringMethod::EndEffectorTrajectoryPtr_t > EndEffectorTrajectory;
} // namespace manipulation_impl

} // namespace steeringMethod



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl__steering__methods_hpp__

