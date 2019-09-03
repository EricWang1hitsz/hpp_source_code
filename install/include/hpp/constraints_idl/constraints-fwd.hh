#ifndef hpp_constraints_idl__constraints_hpp__
#define hpp_constraints_idl__constraints_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl
//

#include <hpp/constraints_idl/constraints-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/constraints/differentiable-function.hh>
#include <hpp/constraints/implicit.hh>
#include <hpp/constraints/locked-joint.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::constraints_idl::DifferentiableFunction
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
class DifferentiableFunctionServant: public ServantBase<hpp::constraints::DifferentiableFunction, _Storage>, public virtual _Base
{
public:
  typedef hpp::constraints::DifferentiableFunction HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::DifferentiableFunction, HppBase);

public:
  // standard constructor
  DifferentiableFunctionServant(Server* server, const _Storage& s);
  virtual ~DifferentiableFunctionServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::floatSeq* value (const hpp::floatSeq& arg);


  hpp::floatSeqSeq* jacobian (const hpp::floatSeq& arg);


  hpp::size_type inputSize ();


  hpp::size_type inputDerivativeSize ();


  hpp::size_type outputSize ();


  hpp::size_type outputDerivativeSize ();


  char* name ();


  char* str ();


};

typedef DifferentiableFunctionServant<POA_hpp::constraints_idl::DifferentiableFunction,hpp::constraints::DifferentiableFunctionPtr_t > DifferentiableFunction;
} // namespace constraints_impl


//
// Class implementing IDL interface hpp::constraints_idl::Implicit
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
class ImplicitServant: public ServantBase<hpp::constraints::Implicit, _Storage>, public virtual _Base
{
public:
  typedef hpp::constraints::Implicit HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::Implicit, HppBase);

public:
  // standard constructor
  ImplicitServant(Server* server, const _Storage& s);
  virtual ~ImplicitServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::constraints_idl::DifferentiableFunction_ptr function ();


  void setRightHandSideFromConfig (const hpp::floatSeq& config);


  void setRightHandSide (const hpp::floatSeq& rhs);


  hpp::floatSeq* getRightHandSide ();


  hpp::size_type rightHandSideSize ();


  hpp::size_type parameterSize ();


  hpp::floatSeq* rightHandSideAt (hpp::value_type s);


};

typedef ImplicitServant<POA_hpp::constraints_idl::Implicit,hpp::constraints::ImplicitPtr_t > Implicit;
} // namespace constraints_impl

//
// Class implementing IDL interface hpp::constraints_idl::LockedJoint
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
class LockedJointServant: public hpp::corbaServer::constraints_impl::ImplicitServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::constraints_impl::ImplicitServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::constraints_idl::LockedJoint, HppBase);

public:
  // standard constructor
  LockedJointServant(Server* server, const _Storage& s);
  virtual ~LockedJointServant();

  // methods corresponding to defined IDL attributes and operations
  
  char* jointName ();


};

typedef LockedJointServant<POA_hpp::constraints_idl::LockedJoint,hpp::constraints::LockedJointPtr_t > LockedJoint;
} // namespace constraints_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_constraints_idl__constraints_hpp__

