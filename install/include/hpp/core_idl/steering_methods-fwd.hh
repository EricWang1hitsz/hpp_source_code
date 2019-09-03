#ifndef hpp_core_idl__steering__methods_hpp__
#define hpp_core_idl__steering__methods_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/steering_methods.idl
//

#include <hpp/core_idl/steering_methods-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/steering-method.hh>
#include <hpp/core_idl/paths.hh>
#include <hpp/core_idl/_constraints.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::SteeringMethod
//
namespace core_impl {
template <typename _Base, typename _Storage>
class SteeringMethodServant: public ServantBase<hpp::core::SteeringMethod, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::SteeringMethod HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::SteeringMethod, HppBase);

public:
  // standard constructor
  SteeringMethodServant(Server* server, const _Storage& s);
  virtual ~SteeringMethodServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::core_idl::Path_ptr call (const hpp::floatSeq& q1, const hpp::floatSeq& q2);


  void setConstraints (hpp::core_idl::ConstraintSet_ptr constraints);


  hpp::core_idl::Constraint_ptr getConstraints ();


};

typedef SteeringMethodServant<POA_hpp::core_idl::SteeringMethod,hpp::core::SteeringMethodPtr_t > SteeringMethod;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__steering__methods_hpp__

